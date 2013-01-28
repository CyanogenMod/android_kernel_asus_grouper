/*
 * EHCI-compliant USB host controller driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2009 - 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <mach/usb_phy.h>
#include <mach/iomap.h>
#include <mach/board-grouper-misc.h>

#define TEGRA_USB_PORTSC_PHCD		(1 << 23)

#define TEGRA_USB_SUSP_CTRL_OFFSET	0x400
#define TEGRA_USB_SUSP_CLR			(1 << 5)
#define TEGRA_USB_PHY_CLK_VALID			(1 << 7)
#define TEGRA_USB_SRT				(1 << 25)
#define TEGRA_USB_PHY_CLK_VALID_INT_ENB		(1 << 9)
#define TEGRA_USB_PHY_CLK_VALID_INT_STS		(1 << 8)

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define TEGRA_USB_PORTSC1_OFFSET	0x184
#else
#define TEGRA_USB_PORTSC1_OFFSET	0x174
#endif
#define TEGRA_USB_PORTSC1_WKCN			(1 << 20)

#define TEGRA_LVL2_CLK_GATE_OVRB	0xfc
#define TEGRA_USB2_CLK_OVR_ON			(1 << 10)

#define TEGRA_USB_DMA_ALIGN 32

#define STS_SRI	(1<<7)	/*	SOF Recieved	*/

#define HOSTPC_REG_OFFSET		0x1b4

#define HOSTPC1_DEVLC_STS 		(1 << 28)
#define HOSTPC1_DEVLC_PTS(x)		(((x) & 0x7) << 29)

#define USB1_PREFETCH_ID               6
#define USB2_PREFETCH_ID               18
#define USB3_PREFETCH_ID               17

extern void baseband_xmm_L3_resume_check(void);
extern volatile int smb347_deep_sleep;  // tmtmtm: from smb347-charger.c
//extern volatile int host_mode_charging_state; // tmtmtm: from smb347-charger.c
//extern int fixed_install_mode; // tmtmtm: from smb347-charger.c
static struct usb_hcd *modem_ehci_handle;

struct tegra_ehci_hcd {
	struct ehci_hcd *ehci;
	struct tegra_usb_phy *phy;
	struct clk *clk;
	struct clk *emc_clk;
	struct clk *sclk_clk;
	struct otg_transceiver *transceiver;
	int host_resumed;
	int bus_suspended;
	int port_resuming;
	int power_down_on_bus_suspend;
	int default_enable;
	enum tegra_usb_phy_port_speed port_speed;
	struct work_struct clk_timer_work;
	struct timer_list clk_timer;
	bool clock_enabled;
	bool timer_event;
	struct mutex tegra_ehci_hcd_mutex;
	unsigned int irq;
	bool bus_suspended_fail;
};

int use_hsic_controller(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	return (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC);
}
EXPORT_SYMBOL(use_hsic_controller);

static void tegra_ehci_power_up(struct usb_hcd *hcd, bool is_dpd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);

	if (!tegra->default_enable)
		clk_enable(tegra->clk);
	tegra_usb_phy_power_on(tegra->phy, is_dpd);
	tegra->host_resumed = 1;
}

static void tegra_ehci_power_down(struct usb_hcd *hcd, bool is_dpd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);

	tegra->host_resumed = 0;
	tegra_usb_phy_power_off(tegra->phy, is_dpd);
	if (!tegra->default_enable)
		clk_disable(tegra->clk);
}

static int tegra_ehci_internal_port_reset(
	struct ehci_hcd	*ehci,
	u32 __iomem	*portsc_reg
)
{
	u32		temp;
	unsigned long	flags;
	int		retval = 0;
	int		i, tries;
	u32		saved_usbintr;

	spin_lock_irqsave(&ehci->lock, flags);
	saved_usbintr = ehci_readl(ehci, &ehci->regs->intr_enable);
	/* disable USB interrupt */
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	spin_unlock_irqrestore(&ehci->lock, flags);

	/*
	 * Here we have to do Port Reset at most twice for
	 * Port Enable bit to be set.
	 */
	for (i = 0; i < 2; i++) {
		temp = ehci_readl(ehci, portsc_reg);
		temp |= PORT_RESET;
		ehci_writel(ehci, temp, portsc_reg);
		mdelay(10);
		temp &= ~PORT_RESET;
		ehci_writel(ehci, temp, portsc_reg);
		mdelay(1);
		tries = 100;
		do {
			mdelay(1);
			/*
			 * Up to this point, Port Enable bit is
			 * expected to be set after 2 ms waiting.
			 * USB1 usually takes extra 45 ms, for safety,
			 * we take 100 ms as timeout.
			 */
			temp = ehci_readl(ehci, portsc_reg);
		} while (!(temp & PORT_PE) && tries--);
		if (temp & PORT_PE)
			break;
	}
	if (i == 2)
		retval = -ETIMEDOUT;

	/*
	 * Clear Connect Status Change bit if it's set.
	 * We can't clear PORT_PEC. It will also cause PORT_PE to be cleared.
	 */
	if (temp & PORT_CSC)
		ehci_writel(ehci, PORT_CSC, portsc_reg);

	/*
	 * Write to clear any interrupt status bits that might be set
	 * during port reset.
	 */
	temp = ehci_readl(ehci, &ehci->regs->status);
	ehci_writel(ehci, temp, &ehci->regs->status);

	/* restore original interrupt enable bits */
	ehci_writel(ehci, saved_usbintr, &ehci->regs->intr_enable);
	return retval;
}

static irqreturn_t tegra_ehci_irq (struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci (hcd);
	struct ehci_regs __iomem *hw = ehci->regs;
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	u32 val;
	irqreturn_t irq_status;
	bool pmc_remote_wakeup = false;

	/* Fence read for coherency of AHB master intiated writes */
	if (tegra->phy->instance == 0)
		readb(IO_ADDRESS(IO_PPCS_PHYS + USB1_PREFETCH_ID));
	else if (tegra->phy->instance == 1)
		readb(IO_ADDRESS(IO_PPCS_PHYS + USB2_PREFETCH_ID));
	else if (tegra->phy->instance == 2)
		readb(IO_ADDRESS(IO_PPCS_PHYS + USB3_PREFETCH_ID));

	if ((tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) &&
		(tegra->ehci->has_hostpc)) {
		/* check if there is any remote wake event */
		if (tegra_usb_phy_is_remotewake_detected(tegra->phy)) {
			pmc_remote_wakeup = true;
			spin_lock (&ehci->lock);
			usb_hcd_resume_root_hub(hcd);
			spin_unlock (&ehci->lock);
		}
	}
	if (tegra->phy->hotplug) {
		spin_lock(&ehci->lock);
		val = readl(hcd->regs + TEGRA_USB_SUSP_CTRL_OFFSET);
		if ((val  & TEGRA_USB_PHY_CLK_VALID_INT_STS)) {
			val &= ~TEGRA_USB_PHY_CLK_VALID_INT_ENB |
				TEGRA_USB_PHY_CLK_VALID_INT_STS;
			writel(val , (hcd->regs + TEGRA_USB_SUSP_CTRL_OFFSET));
			val = readl(&hw->status);
			if (!(val  & STS_PCD)) {
				spin_unlock(&ehci->lock);
				return 0;
			}
			val = readl(hcd->regs + TEGRA_USB_PORTSC1_OFFSET);
			val &= ~(TEGRA_USB_PORTSC1_WKCN | PORT_RWC_BITS);
			writel(val , (hcd->regs + TEGRA_USB_PORTSC1_OFFSET));
		}
		else if (tegra->bus_suspended &&
				tegra->port_speed > TEGRA_USB_PHY_PORT_SPEED_HIGH) {
		  // tmtmtm: OTG UNPLUG
		  // original intent: when waking up from deep sleep, skip the default return, 
		  //                  if host_mode_charging AND fixed_install_mode are set
		  //if(host_mode_charging_state && fixed_install_mode) {
    	  //  printk("ehci-tegra %s waking up with host_mode_charging: special\n", __func__);
		  if(smb347_deep_sleep) {
    	    printk("ehci-tegra %s wake-up/OTG-UNPLUG with smb347_deep_sleep: special\n", __func__);
    	    // fix: skip default return
    	    
    	    // FIXME
    	    // DAS EINSCHRÄNKEN AUF DEN fixed_install_mode löst das problem nur im MOBILE kernel
    	    // ECHTE LÖSUNG: ONLY skip default return when really waking up from deep sleep
    	    // das kann sowohl bei unplug als auch bei plug passieren
    	  } else {
            printk("ehci-tegra %s wake-up/OTG-PLUG without smb347_deep_sleep: normal return\n", __func__);
			spin_unlock(&ehci->lock);
			return 0;
		  }
		}
		spin_unlock(&ehci->lock);
        //printk("ehci-tegra %s post spin_unlock\n", __func__);
	}

	irq_status = ehci_irq(hcd);

	if (pmc_remote_wakeup || tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC) {
		ehci->controller_remote_wakeup = false;
	}

	if (ehci->controller_remote_wakeup) {
        //printk("ehci-tegra %s ehci->controller_remote_wakeup\n", __func__);
		ehci->controller_remote_wakeup = false;
		/* disable interrupts */
		ehci_writel(ehci, 0, &ehci->regs->intr_enable);
		tegra_usb_phy_preresume(tegra->phy, true);
		tegra->port_resuming = 1;
        //printk("ehci-tegra %s ehci->controller_remote_wakeup done\n", __func__);
	}
    //printk("ehci-tegra %s return irq_status=%d\n", __func__,irq_status);
	return irq_status;
}

static int tegra_ehci_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int		ports = HCS_N_PORTS(ehci->hcs_params);
	u32		temp, status, cmd_run;
	u32 __iomem	*status_reg;
	u32		usbsts_reg;

	unsigned long	flags;
	int		retval = 0;
	unsigned	selector;
	struct		tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	bool		hsic = false;

	mutex_lock(&tegra->tegra_ehci_hcd_mutex);
	if (!tegra->host_resumed) {
		if (buf)
			memset (buf, 0, wLength);
		mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
		return retval;
	}

	hsic = (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC);

	status_reg = &ehci->regs->port_status[(wIndex & 0xff) - 1];

	spin_lock_irqsave(&ehci->lock, flags);

	/*
	 * In ehci_hub_control() for USB_PORT_FEAT_ENABLE clears the other bits
	 * that are write on clear, by writing back the register read value, so
	 * USB_PORT_FEAT_ENABLE is handled by masking the set on clear bits
	 */
	if (typeReq == ClearPortFeature && wValue == USB_PORT_FEAT_ENABLE) {
		temp = ehci_readl(ehci, status_reg) & ~PORT_RWC_BITS;
		ehci_writel(ehci, temp & ~PORT_PE, status_reg);
		goto done;
	} else if (typeReq == GetPortStatus) {
		temp = ehci_readl(ehci, status_reg);
		/* check port is in resume state */
		if (tegra->port_resuming) {
			int delay = ehci->reset_done[wIndex-1] - jiffies;
			/* Sometimes it seems we get called too soon... In that case, wait.*/
			if (delay > 0) {
				ehci_dbg(ehci, "GetPortStatus called too soon, waiting %dms...\n", delay);
				mdelay(jiffies_to_msecs(delay));
			}
			/* Ensure the port PORT_SUSPEND and PORT_RESUME has cleared */
			if (handshake(ehci, status_reg, (PORT_SUSPEND | PORT_RESUME), 0, 25000)) {
				pr_err("%s: timeout waiting for SUSPEND to clear\n", __func__);
			}
			tegra->port_resuming = 0;
			tegra_usb_phy_postresume(tegra->phy, false);
			if (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) {
				ehci->command |= CMD_RUN;
				cmd_run = ehci_readl(ehci, &ehci->regs->command);
				cmd_run |= CMD_RUN;
				/*
				 * ehci run bit is disabled to avoid SOF.
				 * 2LS WAR is executed by now enable the run bit.
				 */
				ehci_writel(ehci, cmd_run, &ehci->regs->command);
				/* Now we can safely re-enable irqs */
				ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
			}
		}

	} else if (typeReq == SetPortFeature && wValue == USB_PORT_FEAT_SUSPEND) {
		temp = ehci_readl(ehci, status_reg);
		if ((temp & PORT_PE) == 0 || (temp & PORT_RESET) != 0) {
			retval = -EPIPE;
			goto done;
		}

		temp &= ~PORT_WKCONN_E;
		temp |= PORT_WKDISC_E | PORT_WKOC_E;
		ehci_writel(ehci, temp | PORT_SUSPEND, status_reg);

		/* Need a 4ms delay before the controller goes to suspend */
		mdelay(4);

		/*
		 * If a transaction is in progress, there may be a delay in
		 * suspending the port. Poll until the port is suspended.
		 */
		if (handshake(ehci, status_reg, PORT_SUSPEND,
						PORT_SUSPEND, 5000))
			pr_err("%s: timeout waiting for SUSPEND\n", __func__);

		set_bit((wIndex & 0xff) - 1, &ehci->suspended_ports);
		/*
		 * If RUN bit is disabled interrupt is not generated after suspend.
		 * This change on T20 will allow ASE interrupt generated after suspend
		 * which will unlink the qheads.
		 */
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		if (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) {
			/* Disable RUN bit. */
			ehci->command &= ~CMD_RUN;
			cmd_run = ehci_readl(ehci, &ehci->regs->command);
			cmd_run &= ~CMD_RUN;
			ehci_writel(ehci, cmd_run, &ehci->regs->command);
			if (handshake (ehci, &ehci->regs->status,
						  STS_HALT, STS_HALT, 16 * 125))
				pr_err("%s() timeout waiting for STS_HALT\n", __func__);
		}
#endif
		tegra_usb_phy_postsuspend(tegra->phy, false);

		goto done;
	}

	/* For USB1 port we need to issue Port Reset twice internally */
	if (tegra->phy->instance == 0 &&
	   (typeReq == SetPortFeature && wValue == USB_PORT_FEAT_RESET)) {
		spin_unlock_irqrestore(&ehci->lock, flags);
		mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
		return tegra_ehci_internal_port_reset(ehci, status_reg);
	}

	/*
	 * Tegra host controller will time the resume operation to clear the bit
	 * when the port control state switches to HS or FS Idle. This behavior
	 * is different from EHCI where the host controller driver is required
	 * to set this bit to a zero after the resume duration is timed in the
	 * driver.
	 */
	else if (typeReq == ClearPortFeature &&
					wValue == USB_PORT_FEAT_SUSPEND) {
		temp = ehci_readl(ehci, status_reg);
		if ((temp & PORT_RESET) || !(temp & PORT_PE)) {
			retval = -EPIPE;
			goto done;
		}

		if (!(temp & PORT_SUSPEND))
			goto done;

		tegra->port_resuming = 1;

		if (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) {
			/* disable interrupts */
			ehci_writel(ehci, 0, &ehci->regs->intr_enable);
			/* Disable RUN bit. */
			ehci->command &= ~CMD_RUN;
			cmd_run = ehci_readl(ehci, &ehci->regs->command);
			cmd_run &= ~CMD_RUN;
			ehci_writel(ehci, cmd_run, &ehci->regs->command);
			if (handshake (ehci, &ehci->regs->status,
						  STS_HALT, STS_HALT, 16 * 125))
				pr_err("%s() timeout waiting for STS_HALT\n", __func__);
		}

		/* Disable disconnect detection during port resume */
		tegra_usb_phy_preresume(tegra->phy, false);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		if (tegra->phy->usb_phy_type != TEGRA_USB_PHY_TYPE_UTMIP) {
#endif
		ehci_dbg(ehci, "%s:USBSTS = 0x%x", __func__,
			ehci_readl(ehci, &ehci->regs->status));
		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		ehci_writel(ehci, usbsts_reg, &ehci->regs->status);
		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		udelay(20);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, STS_SRI, 2000))
			pr_err("%s: timeout set for STS_SRI\n", __func__);

		usbsts_reg = ehci_readl(ehci, &ehci->regs->status);
		ehci_writel(ehci, usbsts_reg, &ehci->regs->status);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, 0, 2000))
			pr_err("%s: timeout clear STS_SRI\n", __func__);

		if (handshake(ehci, &ehci->regs->status, STS_SRI, STS_SRI, 2000))
			pr_err("%s: timeout set STS_SRI\n", __func__);

		udelay(20);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		}
#endif
		temp &= ~(PORT_RWC_BITS | PORT_WAKE_BITS);
		/* start resume signaling */
		ehci_writel(ehci, temp | PORT_RESUME, status_reg);

		ehci->reset_done[wIndex-1] = jiffies + msecs_to_jiffies(25);
		/* whoever resumes must GetPortStatus to complete it!! */
		goto done;
	}

	/* Handle port reset here */
	if ((hsic) && (typeReq == SetPortFeature) &&
		((wValue == USB_PORT_FEAT_RESET) || (wValue == USB_PORT_FEAT_POWER))) {
		selector = wIndex >> 8;
		wIndex &= 0xff;
		if (!wIndex || wIndex > ports) {
			retval = -EPIPE;
			goto done;
		}
		wIndex--;
		status = 0;
		temp = ehci_readl(ehci, status_reg);
		if (temp & PORT_OWNER)
			goto done;
		temp &= ~PORT_RWC_BITS;

		switch (wValue) {
		case USB_PORT_FEAT_RESET:
		{
			if (temp & PORT_RESUME) {
				retval = -EPIPE;
				goto done;
			}
			/* line status bits may report this as low speed,
			* which can be fine if this root hub has a
			* transaction translator built in.
			*/
			if ((temp & (PORT_PE|PORT_CONNECT)) == PORT_CONNECT
					&& !ehci_is_TDI(ehci) && PORT_USB11 (temp)) {
				ehci_dbg (ehci, "port %d low speed --> companion\n", wIndex + 1);
				temp |= PORT_OWNER;
				ehci_writel(ehci, temp, status_reg);
			} else {
				ehci_vdbg(ehci, "port %d reset\n", wIndex + 1);
				temp &= ~PORT_PE;
				/*
				* caller must wait, then call GetPortStatus
				* usb 2.0 spec says 50 ms resets on root
				*/
				ehci->reset_done[wIndex] = jiffies + msecs_to_jiffies(50);
				ehci_writel(ehci, temp, status_reg);
				if (hsic && (wIndex == 0))
					tegra_usb_phy_bus_reset(tegra->phy);
			}

			break;
		}
		case USB_PORT_FEAT_POWER:
		{
			if (HCS_PPC(ehci->hcs_params))
				ehci_writel(ehci, temp | PORT_POWER, status_reg);
			if (hsic && (wIndex == 0))
				tegra_usb_phy_bus_connect(tegra->phy);
			break;
		}
		}
		goto done;
	}

	spin_unlock_irqrestore(&ehci->lock, flags);

	/* Handle the hub control events here */
	retval = ehci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);
	mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
	return retval;
done:
	spin_unlock_irqrestore(&ehci->lock, flags);
	mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
	return retval;
}

#ifdef CONFIG_PM
static void tegra_ehci_restart(struct usb_hcd *hcd, bool is_dpd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	unsigned int temp;

	ehci->controller_resets_phy = 0;
	tegra_ehci_pre_reset(tegra->phy, false);
	ehci_reset(ehci);
	tegra_ehci_post_reset(tegra->phy, false);

	if (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_NULL_ULPI)
		ehci->controller_resets_phy = 1;

	/* setup the frame list and Async q heads */
	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32)ehci->async->qh_dma, &ehci->regs->async_next);
	/* setup the command register and set the controller in RUN mode */
	ehci->command &= ~(CMD_LRESET|CMD_IAAD|CMD_PSE|CMD_ASE|CMD_RESET);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* dont start RS here for HSIC, it will be set by bus_reset */
	if (tegra->phy->usb_phy_type != TEGRA_USB_PHY_TYPE_HSIC)
#endif
		ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	/* Enable the root Port Power */
	if (HCS_PPC(ehci->hcs_params)) {
		temp = ehci_readl(ehci, &ehci->regs->port_status[0]);
		ehci_writel(ehci, temp | PORT_POWER, &ehci->regs->port_status[0]);
	}

	down_write(&ehci_cf_port_reset_rwsem);
	if(is_dpd)
		hcd->state = HC_STATE_SUSPENDED;
	else
		hcd->state = HC_STATE_RUNNING;
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	/* flush posted writes */
	ehci_readl(ehci, &ehci->regs->command);
	up_write(&ehci_cf_port_reset_rwsem);

	/* Turn On Interrupts */
	ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
}

static int tegra_usb_suspend(struct usb_hcd *hcd, bool is_dpd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	struct ehci_regs __iomem *hw = tegra->ehci->regs;
	unsigned long flags;
	int hsic = 0;

	hsic = (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC);

	spin_lock_irqsave(&tegra->ehci->lock, flags);

	if (tegra->ehci->has_hostpc)
		tegra->port_speed = (readl(hcd->regs + HOSTPC_REG_OFFSET) >> 25) & 0x3;
	else
		tegra->port_speed = (readl(&hw->port_status[0]) >> 26) & 0x3;
	ehci_halt(tegra->ehci);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	if (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) {
		/*
		 * Ehci run bit is disabled by now read this into command variable
		 * so that bus resume will not enable run bit immedialty.
		 * this is required for 2LS WAR on UTMIP interface.
		 */
		tegra->ehci->command = ehci_readl(tegra->ehci,
						&tegra->ehci->regs->command);
	}
#endif

	spin_unlock_irqrestore(&tegra->ehci->lock, flags);

	tegra_ehci_power_down(hcd, is_dpd);
	return 0;
}

static int tegra_usb_resume(struct usb_hcd *hcd, bool is_dpd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	struct usb_device *udev = hcd->self.root_hub;
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	struct ehci_regs __iomem *hw = ehci->regs;
	unsigned long val;
	bool hsic;
	bool null_ulpi;
	bool utmip_remote_wakeup = false;

	null_ulpi = (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_NULL_ULPI);
	hsic = (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC);

	tegra_ehci_power_up(hcd, is_dpd);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

    // tmtmtm: OTG PLUG
    // original intent: skip the default restart, if host_mode_charging is set
    // FIXME: hier keine einschränkung auf fixed_install_mode?
	//if(host_mode_charging_state) {
  	//	printk("ehci-tegra ######### tegra_usb_resume host_mode_charging: special\n");
    if(smb347_deep_sleep) {
        printk("ehci-tegra %s wake-up/OTG-PLUG with smb347_deep_sleep: special\n", __func__);
		// kommt u.a. 
		// wenn der gepowerte OTG adapter gesteckt wird (mobile-use kernel)
  		// fix: skip default restart
	} else
	if ((tegra->port_speed > TEGRA_USB_PHY_PORT_SPEED_HIGH) || (hsic) ||
	    (null_ulpi))
	{
   		//printk("ehci-tegra #### tegra_usb_resume !host_mode_charging: restart\n");
        printk("ehci-tegra %s wake-up/OTG-PLUG without smb347_deep_sleep: normal restart\n", __func__);
		goto restart;
	}

	/* Force the phy to keep data lines in suspend state */
	tegra_ehci_phy_restore_start(tegra->phy, tegra->port_speed);

	if (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) {
		ehci_reset(ehci);
	}

	/* Enable host mode */
	tdi_reset(ehci);

	if ((tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) &&
		(tegra->ehci->has_hostpc)) {
		val = readl(hcd->regs + HOSTPC_REG_OFFSET);
		val &= ~HOSTPC1_DEVLC_PTS(~0);
		val |= HOSTPC1_DEVLC_STS;
		writel(val, hcd->regs + HOSTPC_REG_OFFSET);
	}

	/* Enable Port Power */
	val = readl(&hw->port_status[0]);
	val |= PORT_POWER;
	writel(val, &hw->port_status[0]);
	udelay(10);

	if ((tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) &&
		(tegra->ehci->has_hostpc) && (tegra->phy->remote_wakeup)) {
		utmip_remote_wakeup = true;
	}

	/* Check if the phy resume from LP0. When the phy resume from LP0
	 * USB register will be reset. */
	if (!readl(&hw->async_next)) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		/* Start the controller */
		val = readl(&hw->command);
		writel((val | CMD_RUN), &hw->command);
#endif
		/* Program the field PTC based on the saved speed mode */
		val = readl(&hw->port_status[0]);
		val &= ~PORT_TEST(~0);
		if (tegra->port_speed == TEGRA_USB_PHY_PORT_SPEED_HIGH)
			val |= PORT_TEST_FORCE;
		else if (tegra->port_speed == TEGRA_USB_PHY_PORT_SPEED_FULL)
			val |= PORT_TEST(6);
		else if (tegra->port_speed == TEGRA_USB_PHY_PORT_SPEED_LOW)
			val |= PORT_TEST(7);
		writel(val, &hw->port_status[0]);
		udelay(10);

		/* Disable test mode by setting PTC field to NORMAL_OP */
		val = readl(&hw->port_status[0]);
		val &= ~PORT_TEST(~0);
		writel(val, &hw->port_status[0]);
		udelay(10);
	}

	/* Poll until CCS is enabled */
	if (handshake(ehci, &hw->port_status[0], PORT_CONNECT,
						 PORT_CONNECT, 2000)) {
		pr_err("%s: timeout waiting for PORT_CONNECT\n", __func__);
		goto restart;
	}

	/* Poll until PE is enabled */
	if (handshake(ehci, &hw->port_status[0], PORT_PE,
						 PORT_PE, 2000)) {
		pr_err("%s: timeout waiting for USB_PORTSC1_PE\n", __func__);
		goto restart;
	}

	/* Clear the PCI status, to avoid an interrupt taken upon resume */
	val = readl(&hw->status);
	val |= STS_PCD;
	writel(val, &hw->status);

	/* Put controller in suspend mode by writing 1 to SUSP bit of PORTSC */
	val = readl(&hw->port_status[0]);
	if ((val & PORT_POWER) && (val & PORT_PE)) {
		val |= PORT_SUSPEND;
		writel(val, &hw->port_status[0]);

		/* Need a 4ms delay before the controller goes to suspend */
		mdelay(4);

		/* Wait until port suspend completes */
		if (handshake(ehci, &hw->port_status[0], PORT_SUSPEND,
							 PORT_SUSPEND, 1000)) {
			pr_err("%s: timeout waiting for PORT_SUSPEND\n",
								__func__);
			goto restart;
		}
	}

	tegra_ehci_phy_restore_end(tegra->phy);
	if (utmip_remote_wakeup) {
		ehci->command |= CMD_RUN;
		ehci_writel(ehci, ehci->command, &ehci->regs->command);
	}
	return 0;

restart:
	if (null_ulpi) {
		bool LP0 = !readl(&hw->async_next);

		if (LP0) {
			static int cnt = 1;

			pr_info("LP0 restart %d\n", cnt++);
			tegra_ehci_phy_restore_start(tegra->phy,
						     tegra->port_speed);
		}

		val = readl(&hw->port_status[0]);
		if (!((val & PORT_POWER) && (val & PORT_PE))) {
			tegra_ehci_restart(hcd, is_dpd);
		}

		if (LP0)
			tegra_ehci_phy_restore_end(tegra->phy);

		return 0;
	}

	if ((tegra->port_speed <= TEGRA_USB_PHY_PORT_SPEED_HIGH) && (!hsic))
		tegra_ehci_phy_restore_end(tegra->phy);
	if (hsic) {
		val = readl(&hw->port_status[0]);
		if (!((val & PORT_POWER) && (val & PORT_PE)))
			tegra_ehci_restart(hcd, false);

		tegra_usb_phy_bus_idle(tegra->phy);
		if (!tegra_usb_phy_is_device_connected(tegra->phy))
			pr_err("%s: no hsic device conenction\n", __func__);
	} else {
		tegra_ehci_restart(hcd, false);
	}

	return 0;
}
#endif

/*
 * Disable PHY clock valid interrupts and wait for the interrupt handler to
 * finish.
 *
 * Requires a lock on tegra_ehci_hcd_mutex
 * Must not be called with a lock on ehci->lock
 */
static void tegra_ehci_disable_phy_interrupt(struct usb_hcd *hcd) {
	struct tegra_ehci_hcd *tegra;
	u32 val;
	if (hcd->irq >= 0) {
		tegra = dev_get_drvdata(hcd->self.controller);
		if (tegra->phy->hotplug) {
			/* Disable PHY clock valid interrupts */
			val = readl(hcd->regs + TEGRA_USB_SUSP_CTRL_OFFSET);
			val &= ~TEGRA_USB_PHY_CLK_VALID_INT_ENB;
			writel(val , (hcd->regs + TEGRA_USB_SUSP_CTRL_OFFSET));
		}
		/* Wait for the interrupt handler to finish */
		synchronize_irq(hcd->irq);
	}
}

void tegra_usb_suspend_hsic(void)
{
	tegra_usb_suspend(modem_ehci_handle ,false);
}
EXPORT_SYMBOL(tegra_usb_suspend_hsic);

void tegra_usb_resume_hsic(void)
{
	tegra_usb_resume(modem_ehci_handle ,false);
}
EXPORT_SYMBOL(tegra_usb_resume_hsic);

static void tegra_ehci_shutdown(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);

	mutex_lock(&tegra->tegra_ehci_hcd_mutex);
	tegra_ehci_disable_phy_interrupt(hcd);
	/* ehci_shutdown touches the USB controller registers, make sure
	 * controller has clocks to it */
	if (!tegra->host_resumed)
		tegra_ehci_power_up(hcd, false);

	ehci_shutdown(hcd);

	/* we are ready to shut down, powerdown the phy */
	tegra_ehci_power_down(hcd, false);
	mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
}

static int tegra_ehci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
		HC_LENGTH(ehci, readl(&ehci->caps->hc_capbase));

	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	ehci->has_hostpc = 1;
#endif
	hcd->has_tt = 1;

	if (tegra->phy->usb_phy_type != TEGRA_USB_PHY_TYPE_NULL_ULPI) {
		ehci_reset(ehci);
		tegra_ehci_post_reset(tegra->phy, false);
	}

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci->sbrn = 0x20;
	ehci->controller_remote_wakeup = false;

	if (tegra->phy->usb_phy_type == TEGRA_USB_PHY_TYPE_NULL_ULPI) {
		tegra_ehci_pre_reset(tegra->phy, false);
		ehci_reset(ehci);
		tegra_ehci_post_reset(tegra->phy, false);

		/*
		 * Resetting the controller has the side effect of resetting the PHY.
		 * So, never reset the controller after the calling
		 * tegra_ehci_reinit API.
		 */
		ehci->controller_resets_phy = 1;
	}

	ehci_port_power(ehci, 1);
	return retval;
}

#ifdef CONFIG_PM
static int tegra_ehci_bus_suspend(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	int error_status = 0;

	mutex_lock(&tegra->tegra_ehci_hcd_mutex);
	tegra->bus_suspended_fail = false;
	tegra_ehci_disable_phy_interrupt(hcd);
	/* ehci_shutdown touches the USB controller registers, make sure
	 * controller has clocks to it */
	if (!tegra->host_resumed)
		tegra_ehci_power_up(hcd, false);
	error_status = ehci_bus_suspend(hcd);
	if (error_status)
		tegra->bus_suspended_fail = true;
	if (!error_status && tegra->power_down_on_bus_suspend) {
		tegra_usb_suspend(hcd, false);
		tegra->bus_suspended = 1;
	}
	mutex_unlock(&tegra->tegra_ehci_hcd_mutex);

	return error_status;
}

static int tegra_ehci_bus_resume(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	int ehci_bus_resumed;

	mutex_lock(&tegra->tegra_ehci_hcd_mutex);
	if (tegra->bus_suspended && tegra->power_down_on_bus_suspend) {
		tegra_usb_resume(hcd, false);
		tegra->bus_suspended = 0;
	}

	ehci_bus_resumed = ehci_bus_resume(hcd);
	mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
	return ehci_bus_resumed;
}
#endif

struct dma_aligned_buffer {
	void *kmalloc_ptr;
	void *old_xfer_buffer;
	u8 data[0];
};

static void free_dma_aligned_buffer(struct urb *urb)
{
	struct dma_aligned_buffer *temp = container_of(urb->transfer_buffer,
		struct dma_aligned_buffer, data);

	if (!(urb->transfer_flags & URB_ALIGNED_TEMP_BUFFER))
		return;

	if(usb_urb_dir_in(urb))
		memcpy(temp->old_xfer_buffer, temp->data,
		       urb->transfer_buffer_length);
	urb->transfer_buffer = temp->old_xfer_buffer;
	kfree(temp->kmalloc_ptr);
	urb->transfer_flags &= ~URB_ALIGNED_TEMP_BUFFER;
}

static int alloc_dma_aligned_buffer(struct urb *urb, gfp_t mem_flags)
{
	struct dma_aligned_buffer *temp, *kmalloc_ptr;
	size_t kmalloc_size;

	if (urb->num_sgs || urb->sg ||
	    urb->transfer_buffer_length == 0 ||
	    !((uintptr_t)urb->transfer_buffer & (TEGRA_USB_DMA_ALIGN - 1)))
		return 0;

	/* Allocate a buffer with enough padding for alignment */
	kmalloc_size = urb->transfer_buffer_length +
		sizeof(struct dma_aligned_buffer) + TEGRA_USB_DMA_ALIGN - 1;

	kmalloc_ptr = kmalloc(kmalloc_size, mem_flags);
	if (!kmalloc_ptr)
		return -ENOMEM;

	/* Position our struct dma_aligned_buffer such that data is aligned */
	temp = PTR_ALIGN(kmalloc_ptr + 1, TEGRA_USB_DMA_ALIGN) - 1;
	temp->kmalloc_ptr = kmalloc_ptr;
	temp->old_xfer_buffer = urb->transfer_buffer;
	if (!usb_urb_dir_in(urb))
		memcpy(temp->data, urb->transfer_buffer,
		       urb->transfer_buffer_length);
	urb->transfer_buffer = temp->data;
	urb->transfer_flags |= URB_ALIGNED_TEMP_BUFFER;

	return 0;
}

static int tegra_ehci_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
				      gfp_t mem_flags)
{
	int ret;

	ret = alloc_dma_aligned_buffer(urb, mem_flags);
	if (ret)
		return ret;

	ret = usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);
	if (ret)
		free_dma_aligned_buffer(urb);

	return ret;
}

static void tegra_ehci_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
	usb_hcd_unmap_urb_for_dma(hcd, urb);
	free_dma_aligned_buffer(urb);
}

void clk_timer_callback(unsigned long data)
{
	struct tegra_ehci_hcd *tegra = (struct tegra_ehci_hcd*) data;
	unsigned long flags;

	if (!timer_pending(&tegra->clk_timer)) {
		spin_lock_irqsave(&tegra->ehci->lock, flags);
		tegra->timer_event = 1;
		spin_unlock_irqrestore(&tegra->ehci->lock, flags);
		schedule_work(&tegra->clk_timer_work);
	}
}

static void clk_timer_work_handler(struct work_struct* clk_timer_work) {
	struct tegra_ehci_hcd *tegra = container_of(clk_timer_work,
						struct tegra_ehci_hcd, clk_timer_work);
	int ret;
	unsigned long flags;
	bool clock_enabled, timer_event;

	spin_lock_irqsave(&tegra->ehci->lock, flags);
	clock_enabled = tegra->clock_enabled;
	timer_event = tegra->timer_event;
	spin_unlock_irqrestore(&tegra->ehci->lock, flags);

	if (timer_event) {
		spin_lock_irqsave(&tegra->ehci->lock, flags);
		tegra->clock_enabled = 0;
		tegra->timer_event = 0;
		spin_unlock_irqrestore(&tegra->ehci->lock, flags);
		clk_disable(tegra->emc_clk);
		clk_disable(tegra->sclk_clk);
		return;
	}

	if ((!clock_enabled)) {
		ret = mod_timer(&tegra->clk_timer, jiffies + msecs_to_jiffies(2000));
		if (ret)
			pr_err("tegra_ehci_urb_enqueue timer modify failed \n");
		clk_enable(tegra->emc_clk);
		clk_enable(tegra->sclk_clk);
		spin_lock_irqsave(&tegra->ehci->lock, flags);
		tegra->clock_enabled = 1;
		spin_unlock_irqrestore(&tegra->ehci->lock, flags);
	} else {
		if (timer_pending(&tegra->clk_timer)) {
			mod_timer_pending (&tegra->clk_timer, jiffies
						+ msecs_to_jiffies(2000));
		}
	}
}

static int tegra_ehci_urb_enqueue (
	struct usb_hcd	*hcd,
	struct urb	*urb,
	gfp_t		mem_flags)
{
	struct tegra_ehci_hcd *pdata;
	int xfertype;
	int transfer_buffer_length;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	unsigned long flags;
	pdata = dev_get_drvdata(hcd->self.controller);

	xfertype = usb_endpoint_type(&urb->ep->desc);
	transfer_buffer_length = urb->transfer_buffer_length;
	spin_lock_irqsave(&ehci->lock,flags);
	/* Turn on the USB busy hints */
	switch (xfertype) {
		case USB_ENDPOINT_XFER_INT:
		if (transfer_buffer_length < 255) {
		/* Do nothing for interrupt buffers < 255 */
		} else {
			/* signal to set the busy hints */
			schedule_work(&pdata->clk_timer_work);
		}
		break;
		case USB_ENDPOINT_XFER_ISOC:
		case USB_ENDPOINT_XFER_BULK:
			/* signal to set the busy hints */
			schedule_work(&pdata->clk_timer_work);
		break;
		case USB_ENDPOINT_XFER_CONTROL:
		default:
			/* Do nothing special here */
		break;
	}
	spin_unlock_irqrestore(&ehci->lock,flags);
	return ehci_urb_enqueue(hcd, urb, mem_flags);
}

static const struct hc_driver tegra_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "Tegra EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	.flags			= HCD_USB2 | HCD_MEMORY,

	.reset			= tegra_ehci_setup,
	.irq			= tegra_ehci_irq,

	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= tegra_ehci_shutdown,
	.urb_enqueue		= tegra_ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.map_urb_for_dma	= tegra_ehci_map_urb_for_dma,
	.unmap_urb_for_dma	= tegra_ehci_unmap_urb_for_dma,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= tegra_ehci_hub_control,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
#ifdef CONFIG_PM
	.bus_suspend		= tegra_ehci_bus_suspend,
	.bus_resume		= tegra_ehci_bus_resume,
#endif
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,
};

static int tegra_ehci_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct usb_hcd *hcd;
	struct tegra_ehci_hcd *tegra;
	struct tegra_ehci_platform_data *pdata;
	int err = 0;
	int irq;
	int instance = pdev->id;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Platform data missing\n");
		return -EINVAL;
	}

	tegra = kzalloc(sizeof(struct tegra_ehci_hcd), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	mutex_init(&tegra->tegra_ehci_hcd_mutex);

	hcd = usb_create_hcd(&tegra_ehci_hc_driver, &pdev->dev,
					dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		err = -ENOMEM;
		goto fail_hcd;
	}

	platform_set_drvdata(pdev, tegra);
	tegra->default_enable = pdata->default_enable;

	tegra->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(tegra->clk)) {
		dev_err(&pdev->dev, "Can't get ehci clock\n");
		err = PTR_ERR(tegra->clk);
		goto fail_clk;
	}

	err = clk_enable(tegra->clk);
	if (err)
		goto fail_clken;


	tegra->sclk_clk = clk_get(&pdev->dev, "sclk");
	if (IS_ERR(tegra->sclk_clk)) {
		dev_err(&pdev->dev, "Can't get sclk clock\n");
		err = PTR_ERR(tegra->sclk_clk);
		goto fail_sclk_clk;
	}

	clk_set_rate(tegra->sclk_clk, 80000000);

	tegra->emc_clk = clk_get(&pdev->dev, "emc");
	if (IS_ERR(tegra->emc_clk)) {
		dev_err(&pdev->dev, "Can't get emc clock\n");
		err = PTR_ERR(tegra->emc_clk);
		goto fail_emc_clk;
	}
	init_timer(&tegra->clk_timer);
	tegra->clk_timer.function = clk_timer_callback;
	tegra->clk_timer.data = (unsigned long) tegra;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	/* Set DDR busy hints to 150MHz. For Tegra 2x SOC, DDR rate is half of EMC rate */
	clk_set_rate(tegra->emc_clk, 300000000);
#else
	/* Set DDR busy hints to 100MHz. For Tegra 3x SOC DDR rate equals to EMC rate */
	clk_set_rate(tegra->emc_clk, 100000000);
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		err = -ENXIO;
		goto fail_io;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = ioremap(res->start, resource_size(res));
	if (!hcd->regs) {
		dev_err(&pdev->dev, "Failed to remap I/O memory\n");
		err = -ENOMEM;
		goto fail_io;
	}

	INIT_WORK(&tegra->clk_timer_work, clk_timer_work_handler);

	tegra->phy = tegra_usb_phy_open(instance, hcd->regs, pdata->phy_config,
					TEGRA_USB_PHY_MODE_HOST, pdata->phy_type);
	if (IS_ERR(tegra->phy)) {
		dev_err(&pdev->dev, "Failed to open USB phy\n");
		err = -ENXIO;
		goto fail_phy;
	}
	tegra->phy->hotplug = pdata->hotplug;

	err = tegra_usb_phy_power_on(tegra->phy, true);
	if (err) {
		dev_err(&pdev->dev, "Failed to power on the phy\n");
		goto fail;
	}

	tegra->host_resumed = 1;
	tegra->power_down_on_bus_suspend = pdata->power_down_on_bus_suspend;
	tegra->ehci = hcd_to_ehci(hcd);

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENODEV;
		goto fail;
	}
	set_irq_flags(irq, IRQF_VALID);
	tegra->irq = irq;

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->operating_mode == TEGRA_USB_OTG) {
		tegra->transceiver = otg_get_transceiver();
		if (tegra->transceiver)
			otg_set_host(tegra->transceiver, &hcd->self);
	}
#endif

	err = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (err) {
		dev_err(&pdev->dev, "Failed to add USB HCD error = %d\n", err);
		goto fail;
	}

	err = enable_irq_wake(tegra->irq);
	if (err < 0) {
		dev_warn(&pdev->dev,
			"Couldn't enable USB host mode wakeup, irq=%d, "
			"error=%d\n", tegra->irq, err);
		err = 0;
		tegra->irq = 0;
	}

	if (instance == 1) {
		modem_ehci_handle = hcd;
	}

	return err;

fail:
#ifdef CONFIG_USB_OTG_UTILS
	if (tegra->transceiver) {
		otg_set_host(tegra->transceiver, NULL);
		otg_put_transceiver(tegra->transceiver);
	}
#endif
	tegra_usb_phy_close(tegra->phy);
fail_phy:
	iounmap(hcd->regs);
fail_io:
	clk_disable(tegra->emc_clk);
	clk_put(tegra->emc_clk);
fail_emc_clk:
	clk_disable(tegra->sclk_clk);
	clk_put(tegra->sclk_clk);
fail_sclk_clk:
	clk_disable(tegra->clk);
fail_clken:
	clk_put(tegra->clk);
fail_clk:
	usb_put_hcd(hcd);
fail_hcd:
	kfree(tegra);
	return err;
}

#ifdef CONFIG_PM
static int tegra_ehci_resume(struct platform_device *pdev)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);
	int ret;
	u32 project_info = grouper_get_project_id();

	if (project_info == GROUPER_PROJECT_NAKASI_3G)
		baseband_xmm_L3_resume_check();

	mutex_lock(&tegra->tegra_ehci_hcd_mutex);
	if ((tegra->bus_suspended) && (tegra->power_down_on_bus_suspend)) {
		if (tegra->default_enable)
			clk_enable(tegra->clk);
		mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
		return 0;
	}

	if (tegra->default_enable)
		clk_enable(tegra->clk);

	ret = tegra_usb_resume(hcd, true);
	mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
	return ret;
}

static int tegra_ehci_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);
	int ret;
	u32 val;

	mutex_lock(&tegra->tegra_ehci_hcd_mutex);
	/* if bus suspend is failed means there is remote wakeup resume,
		then abort the PM suspend */
	if (tegra->bus_suspended_fail) {
		tegra->bus_suspended_fail = false;
		pr_err("%s: bus suspend failed, aborting driver suspend\n", __func__);
		mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
		return -EBUSY;
	}
	if (tegra->phy->hotplug) {
		/* Disable PHY clock valid interrupts while going into suspend*/
		val = readl(hcd->regs + TEGRA_USB_SUSP_CTRL_OFFSET);
		val &= ~TEGRA_USB_PHY_CLK_VALID_INT_ENB;
		writel(val , (hcd->regs + TEGRA_USB_SUSP_CTRL_OFFSET));
	}

	if ((tegra->bus_suspended) && (tegra->power_down_on_bus_suspend)) {
		if (tegra->default_enable)
			clk_disable(tegra->clk);
		mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
		return 0;
	}

	if (time_before(jiffies, tegra->ehci->next_statechange))
		msleep(10);

	ret = tegra_usb_suspend(hcd, true);
	if (tegra->default_enable)
		clk_disable(tegra->clk);
	mutex_unlock(&tegra->tegra_ehci_hcd_mutex);
	return ret;
}
#endif

static int tegra_ehci_remove(struct platform_device *pdev)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);

	if (tegra == NULL || hcd == NULL)
		return -EINVAL;
	/* make sure controller is on as we will touch its registers */
	if (!tegra->host_resumed)
		tegra_ehci_power_up(hcd, true);

#ifdef CONFIG_USB_OTG_UTILS
	if (tegra->transceiver) {
		otg_set_host(tegra->transceiver, NULL);
		otg_put_transceiver(tegra->transceiver);
	}
#endif
	if (tegra->phy->instance == 1) {
		modem_ehci_handle = NULL;
	}

	/* Turn Off Interrupts */
	ehci_writel(tegra->ehci, 0, &tegra->ehci->regs->intr_enable);
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	if (tegra->irq)
		disable_irq_wake(tegra->irq);
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	tegra_usb_phy_power_off(tegra->phy, true);
	tegra_ehci_disable_phy_interrupt(hcd);
	tegra_usb_phy_close(tegra->phy);
	iounmap(hcd->regs);

	del_timer_sync(&tegra->clk_timer);

	clk_disable(tegra->clk);
	clk_put(tegra->clk);

	if (tegra->clock_enabled) {
		clk_disable(tegra->sclk_clk);
		clk_disable(tegra->emc_clk);
	}
	clk_put(tegra->sclk_clk);
	clk_put(tegra->emc_clk);

	kfree(tegra);
	return 0;
}

static void tegra_ehci_hcd_shutdown(struct platform_device *pdev)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static struct platform_driver tegra_ehci_driver = {
	.probe		= tegra_ehci_probe,
	.remove		= tegra_ehci_remove,
#ifdef CONFIG_PM
	.suspend	= tegra_ehci_suspend,
	.resume		= tegra_ehci_resume,
#endif
	.shutdown	= tegra_ehci_hcd_shutdown,
	.driver		= {
		.name	= "tegra-ehci",
	}
};
