/*
 * arch/arm/mach-tegra/baseband-xmm-power.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/pm_runtime.h>
#include <mach/usb_phy.h>
#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "baseband-xmm-power.h"
#include "pm-irq.h"
MODULE_LICENSE("GPL");

unsigned long modem_ver = XMM_MODEM_VER_1130;
EXPORT_SYMBOL(modem_ver);

unsigned long modem_flash = 1;
EXPORT_SYMBOL(modem_flash);

unsigned long modem_pm = 1;
EXPORT_SYMBOL(modem_pm);

unsigned long enum_delay_ms = 1000; /* ignored if !modem_flash */

module_param(modem_ver, ulong, 0644);
MODULE_PARM_DESC(modem_ver,
	"baseband xmm power - modem software version");
module_param(modem_flash, ulong, 0644);
MODULE_PARM_DESC(modem_flash,
	"baseband xmm power - modem flash (1 = flash, 0 = flashless)");
module_param(modem_pm, ulong, 0644);
MODULE_PARM_DESC(modem_pm,
	"baseband xmm power - modem power management (1 = pm, 0 = no pm)");
module_param(enum_delay_ms, ulong, 0644);
MODULE_PARM_DESC(enum_delay_ms,
	"baseband xmm power - delay in ms between modem on and enumeration");

static struct usb_device_id xmm_pm_ids[] = {
	{ USB_DEVICE(VENDOR_ID, PRODUCT_ID),
	.driver_info = 0 },
	{}
};


static struct gpio tegra_baseband_gpios[] = {
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_RSTn" },
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_ON"   },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_BB_WAKE" },
	{ -1, GPIOF_IN,            "IPC_AP_WAKE" },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_HSIC_ACTIVE" },
	{ -1, GPIOF_IN,            "IPC_HSIC_SUS_REQ" },
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_VBAT" },
	{ -1, GPIOF_IN,            "IPC_BB_RST_IND" },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_BB_FORCE_CRASH" },
};

static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state;

enum baseband_xmm_powerstate_t baseband_xmm_powerstate;
static struct workqueue_struct *workqueue;
static struct work_struct init1_work;
static struct work_struct init2_work;
static struct work_struct L2_resume_work;
static struct baseband_power_platform_data *baseband_power_driver_data;
static struct regulator *reg_grouper_hsic = NULL;    /* LDO7 */
static int waiting_falling_flag = 0;
static bool register_hsic_device;
static struct wake_lock wakelock;
static struct wake_lock modem_recovery_wakelock;
static struct usb_device *usbdev;
static bool CP_initiated_L2toL0;
static bool modem_power_on;
static int power_onoff;
static int reenable_autosuspend;
static struct work_struct autopm_resume_work;
static bool wakeup_pending;
static bool modem_sleep_flag;
static spinlock_t xmm_lock;
static DEFINE_MUTEX(xmm_onoff_mutex);
static int modem_reset_flag = 0;
bool resume_from_l3 = false;

#define MOD_HANG        TEGRA_GPIO_PN2

static void baseband_xmm_power_reset_on(void);
static void baseband_xmm_power_L2_resume(void);
static int baseband_xmm_power_driver_handle_resume(
			struct baseband_power_platform_data *data);
extern void tegra_usb_suspend_hsic(void);
extern void tegra_usb_resume_hsic(void);
extern void ril_change_modem_crash_mode(void);

int baseband_xmm_enable_hsic_power(int enable)
{
	int ret = 0;

	if(!reg_grouper_hsic) {
		pr_err("%s reg_grouper_hsic is NULL!\n", __func__);
		ret = -1;
		goto exit;
	}

	pr_info("enable: %d\n", enable);
	if (enable > 0) {
		ret = regulator_enable(reg_grouper_hsic);
	} else {
		ret = regulator_disable(reg_grouper_hsic);
	}

exit:
	return ret;
}

static int baseband_modem_power_on(struct baseband_power_platform_data *data)
{
	/* set IPC_HSIC_ACTIVE active */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	mdelay(20);

	/* reset / power on sequence */
	mdelay(40);
	gpio_set_value(data->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(data->modem.xmm.bb_on, 1);
	udelay(40);
	gpio_set_value(data->modem.xmm.bb_on, 0);

	return 0;
}

static int baseband_xmm_power_on(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	int ret;

	pr_debug("%s {\n", __func__);

	/* check for platform data */
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}
	if (baseband_xmm_powerstate != BBXMM_PS_UNINIT)
		return -EINVAL;

	/* reset the state machine */
	baseband_xmm_powerstate = BBXMM_PS_INIT;
	modem_sleep_flag = false;

	if (modem_ver < XMM_MODEM_VER_1130)
		ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
	else
		ipc_ap_wake_state = IPC_AP_WAKE_INIT2;

	pr_debug("%s wake_st(%d) modem version %ld\n", __func__,
					ipc_ap_wake_state, modem_ver);

	/* register usb host controller */
	if (!modem_flash) {
		pr_debug("%s - %d\n", __func__, __LINE__);
		/* register usb host controller only once */
		if (register_hsic_device) {
			pr_debug("%s: register usb host controller\n",
				__func__);
			modem_power_on = true;
			if (data->hsic_register)
				data->modem.xmm.hsic_device =
						data->hsic_register();
			else
				pr_err("%s: hsic_register is missing\n",
					__func__);
			register_hsic_device = false;
		} else {
			/* register usb host controller */
			if (data->hsic_register)
				data->modem.xmm.hsic_device =
							data->hsic_register();
			/* turn on modem */
			pr_debug("%s call baseband_modem_power_on\n", __func__);
			baseband_modem_power_on(data);
		}
	}else {
		/* reset flashed modem then it will respond with
		 * ap-wake rising followed by falling gpio
		 */

		pr_debug("%s: reset flash modem\n", __func__);
		modem_power_on = false;
		ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
		gpio_set_value(data->modem.xmm.ipc_hsic_active, 0);
		waiting_falling_flag = 0;

		baseband_xmm_power_reset_on();
	}
	ret = enable_irq_wake(gpio_to_irq(data->modem.xmm.ipc_ap_wake));
	if (ret < 0)
		pr_err("%s: enable_irq_wake error\n", __func__);

	pr_debug("%s }\n", __func__);

	return 0;
}

static int baseband_xmm_power_off(struct platform_device *device)
{
	struct baseband_power_platform_data *data;
	int ret;
	unsigned long flags;

	pr_debug("%s {\n", __func__);

	if (baseband_xmm_powerstate == BBXMM_PS_UNINIT)
		return -EINVAL;
	/* check for device / platform data */
	if (!device) {
		pr_err("%s: !device\n", __func__);
		return -EINVAL;
	}
	data = (struct baseband_power_platform_data *)
		device->dev.platform_data;
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

	ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
	ret = disable_irq_wake(gpio_to_irq(data->modem.xmm.ipc_ap_wake));
	if (ret < 0)
		pr_err("%s: disable_irq_wake error\n", __func__);

	/* unregister usb host controller */
	if (data->hsic_unregister) {
		if (!register_hsic_device) {
			register_hsic_device = true;
			data->hsic_unregister(data->modem.xmm.hsic_device);
		} else
			pr_err("%s: hsic contorller was not registered\n", __func__);
	} else
		pr_err("%s: hsic_unregister is missing\n", __func__);


	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	/* wait 20 ms */
	mdelay(20);

	/* drive bb_rst low */
	gpio_set_value(data->modem.xmm.bb_rst, 0);
	mdelay(1);
	gpio_set_value(data->modem.xmm.bb_vbat, 0);
	mdelay(1);

	spin_lock_irqsave(&xmm_lock, flags);
	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	wakeup_pending = false;
	modem_sleep_flag = false;
	spin_unlock_irqrestore(&xmm_lock, flags);
	/* start registration process once again on xmm on */
	register_hsic_device = true;
	pr_debug("%s }\n", __func__);

	return 0;
}

static ssize_t baseband_xmm_onoff(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int pwr;
	int size;
	struct platform_device *device = to_platform_device(dev);

	mutex_lock(&xmm_onoff_mutex);

	pr_debug("%s\n", __func__);

	/* check input */
	if (buf == NULL) {
		pr_err("%s: buf NULL\n", __func__);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}
	pr_debug("%s: count=%d\n", __func__, count);

	/* parse input */
	size = sscanf(buf, "%d", &pwr);
	if (size != 1) {
		pr_err("%s: size=%d -EINVAL\n", __func__, size);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}

	if (power_onoff == pwr) {
		pr_err("%s: Ignored, due to same CP power state(%d)\n",
						__func__, power_onoff);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}
	power_onoff = pwr;
	pr_debug("%s power_onoff=%d\n", __func__, power_onoff);

	if (power_onoff == 0)
		baseband_xmm_power_off(device);
	else if (power_onoff == 1)
		baseband_xmm_power_on(device);

	mutex_unlock(&xmm_onoff_mutex);

	return count;
}

static ssize_t store_dwd_reset_modem(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int state;
	struct platform_device *device = to_platform_device(dev);
	struct baseband_power_platform_data *data;

	sscanf(buf, "%d", &state);

	if (state > 0) {
		pr_info("++ dwd_reset_modem ++\n");
		/* check for device / platform data */
		if (!device) {
			pr_err("%s: !device\n", __func__);
			return -EINVAL;
		}
		data = (struct baseband_power_platform_data *)
				device->dev.platform_data;
		if (!data) {
			pr_err("%s: !data\n", __func__);
			return -EINVAL;
		}

		if (!(data->hsic_unregister && data->hsic_register)) {
			pr_err("%s: hsic_unregister or hsic_register missing\n", __func__);
			return -EINVAL;
		}
		if (!register_hsic_device) {
			register_hsic_device = true;
			data->hsic_unregister(data->modem.xmm.hsic_device);
			mdelay(20);
		}
		gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 0);
		ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
		mdelay(1);
		data->modem.xmm.hsic_device = data->hsic_register();
		register_hsic_device = false;
		baseband_xmm_power_reset_on();
		pr_info("-- dwd_reset_modem --\n");
	}
	return count;
}

static ssize_t store_nml_reset_modem(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
       int state;
	struct platform_device *device = to_platform_device(dev);

       sscanf(buf, "%d", &state);

       pr_info("++ nml_reset_modem ++\n");
       if (state > 0) {
		baseband_xmm_power_off(device);
		msleep(50);
		baseband_xmm_power_on(device);
		modem_reset_flag = 1;
       }
       pr_info("-- nml_reset_modem --\n");
       return count;
}

static ssize_t store_force_crash_modem(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int state;
	struct platform_device *device = to_platform_device(dev);
	struct baseband_power_platform_data *data;
	int value = 0;
	int delay = 5;

	sscanf(buf, "%d", &state);

	pr_info("++ force_crash_modem ++\n");
	/* check for device / platform data */
	if (!device) {
		pr_err("%s: !device\n", __func__);
		return -EINVAL;
	}
	data = (struct baseband_power_platform_data *)
		device->dev.platform_data;
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

	if (state > 0) {
		do {
			gpio_set_value(data->modem.xmm.ipc_bb_force_crash, 1);
			msleep(50);
			gpio_set_value(data->modem.xmm.ipc_hsic_active, 1);
			msleep(50);
			gpio_set_value(data->modem.xmm.ipc_hsic_active, 0);
			msleep(50);
			value = gpio_get_value(MOD_HANG);
			delay--;
		} while ((!value) && (delay));
		if (delay)
			pr_info("%s: modem crash is normal\n", __func__);
		else {
			pr_info("%s: modem crash is abnormal and force change the modem crash mode\n", __func__);
			ril_change_modem_crash_mode();
		}
	}
	pr_info("-- force_crash_modem --\n");

	return count;
}

/*
static DEVICE_ATTR(xmm_onoff, S_IRUSR | S_IWUSR | S_IRGRP,
		NULL, baseband_xmm_onoff);
*/
static struct device_attribute xmm_device_attr[] = {
       __ATTR(xmm_onoff, S_IRUSR | S_IWUSR | S_IRGRP, NULL, baseband_xmm_onoff),
       __ATTR(xmm_dwd_reset, S_IRUSR | S_IWUSR | S_IRGRP, NULL, store_dwd_reset_modem),
       __ATTR(xmm_nml_reset, S_IRUSR | S_IWUSR | S_IRGRP, NULL, store_nml_reset_modem),
       __ATTR(xmm_force_crash, S_IRUSR | S_IWUSR | S_IRGRP, NULL, store_force_crash_modem),
       __ATTR_NULL,
};

/* baseband_usb_utmip_host_register functions
	1 -> register utmip host controller
	0 -> unregister utmip host controller
*/
int baseband_usb_utmip_host_register(int enable)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;

	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

	if (!(data->hsic_unregister && data->hsic_register && data->utmip_unregister && data->utmip_register)) {
		pr_err("%s: utmip_unregister or utmip_register missing\n", __func__);
		return -EINVAL;
	}

	if(!enable) {
		if (!register_hsic_device) {
			register_hsic_device = true;
			data->utmip_unregister(data->modem.xmm.hsic_device);
		}
	} else if (enable)
		data->modem.xmm.hsic_device = data->utmip_register();

	return 0;
 }

/* baseband_usb_hsic_host_register functions
	1 -> register hsic host controller
	0 -> unregister hsic host controller
*/
int baseband_usb_hsic_host_register(int enable)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;

	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

	if (!(data->hsic_unregister && data->hsic_register && data->utmip_unregister && data->utmip_register)) {
		pr_err("%s: hsic_unregister or hsic_register missing\n", __func__);
		return -EINVAL;
	}

	if(!enable) {
		if (!register_hsic_device) {
			register_hsic_device = true;
			data->hsic_unregister(data->modem.xmm.hsic_device);
		}
	} else if (enable)
		data->modem.xmm.hsic_device = data->hsic_register();

	return 0;
}

void baseband_xmm_L3_resume_check(void)
{
       struct baseband_power_platform_data *data = baseband_power_driver_data;
       if (modem_sleep_flag) {
               pr_info("%s Resume from L3 without calling resume"
                      "function\n",  __func__);
               baseband_xmm_power_driver_handle_resume(data);
       }
}
EXPORT_SYMBOL_GPL(baseband_xmm_L3_resume_check);

void baseband_xmm_set_power_status(unsigned int status)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	int value = 0;
	unsigned long flags;

	pr_debug("%s\n", __func__);

	if (baseband_xmm_powerstate == status)
		return;

	switch (status) {
	case BBXMM_PS_L0:
		/*if (modem_sleep_flag) {
			pr_info("%s Resume from L3 without calling resume"
						"function\n",  __func__);
			baseband_xmm_power_driver_handle_resume(data);
		}*/
		pr_info("L0\n");
		baseband_xmm_powerstate = status;
		value = gpio_get_value(data->modem.xmm.ipc_hsic_active);
		pr_debug("before L0 ipc_hsic_active=%d\n", value);
		if (!value) {
			pr_debug("before L0 gpio set ipc_hsic_active=1 ->\n");
			gpio_set_value(data->modem.xmm.ipc_hsic_active, 1);
		}
		if (modem_power_on) {
			modem_power_on = false;
			baseband_modem_power_on(data);
		}

		if (!wake_lock_active(&wakelock))
			wake_lock(&wakelock);

		pr_debug("gpio host active high->\n");
		break;
	case BBXMM_PS_L2:
		pr_info("L2\n");
		baseband_xmm_powerstate = status;
		wake_unlock(&wakelock);
		modem_sleep_flag = true;
		break;
	case BBXMM_PS_L3:
		if (baseband_xmm_powerstate == BBXMM_PS_L2TOL0) {
			if (!gpio_get_value(data->modem.xmm.ipc_ap_wake)) {
				spin_lock_irqsave(&xmm_lock, flags);
				wakeup_pending = true;
				spin_unlock_irqrestore(&xmm_lock, flags);
				pr_info("%s: L2 race condition-CP wakeup"
						" pending\n", __func__);
			}
		}
		pr_info("L3\n");
		baseband_xmm_powerstate = status;
		if (wake_lock_active(&wakelock)) {
			pr_info("%s: releasing wakelock before L3\n",
				__func__);
			wake_unlock(&wakelock);
		}
		if (register_hsic_device != true && wakeup_pending == false) {
			gpio_set_value(data->modem.xmm.ipc_hsic_active, 0);
			pr_debug("gpio host active low->\n");
		}
		break;
	case BBXMM_PS_L2TOL0:
		/* do this only from L2 state */
		if (baseband_xmm_powerstate == BBXMM_PS_L2) {
			baseband_xmm_powerstate = status;
			pr_debug("BB XMM POWER STATE = %d\n", status);
			baseband_xmm_power_L2_resume();
		} else
			goto exit_without_state_change;
	default:
		baseband_xmm_powerstate = status;
		break;
	}
	pr_debug("BB XMM POWER STATE = %d\n", status);
	return;

exit_without_state_change:
	pr_info("BB XMM POWER STATE = %d (not change to %d)\n",
			baseband_xmm_powerstate, status);
	return;
}
EXPORT_SYMBOL_GPL(baseband_xmm_set_power_status);

void baseband_xmm_ap_resume_work(void)
{
	pr_debug("%s: AP resume\n",__func__);
	queue_work(workqueue, &L2_resume_work);
}
EXPORT_SYMBOL( baseband_xmm_ap_resume_work);

irqreturn_t baseband_xmm_power_ipc_ap_wake_irq(int irq, void *dev_id)
{
	int value;

	value = gpio_get_value(baseband_power_driver_data->
				modem.xmm.ipc_ap_wake);

	pr_debug("%s g(%d), wake_st(%d)\n", __func__, value, ipc_ap_wake_state);

	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - got falling edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
			/* queue work */
			queue_work(workqueue, &init1_work);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - wait for falling edge\n",
				__func__);
		}
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_INIT1) {
		if (value) {
			pr_debug("%s - IPC_AP_WAKE_INIT2"
				" - wait for falling edge\n",
				__func__);
			waiting_falling_flag = 1;
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT2"
				" - got falling edge\n",
				__func__);
			if (waiting_falling_flag == 0) {
				pr_debug("%s return because irq must get the rising event at first\n", __func__);
				return IRQ_HANDLED;
			}
			/* go to IPC_AP_WAKE_INIT2 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			/* queue work */
			queue_work(workqueue, &init2_work);
		}
	} else {
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			/* [ver < 1130] gpio protocol falling edge */
			if (modem_ver < XMM_MODEM_VER_1130) {
				pr_debug("gpio host wakeup done <-\n");
				value = gpio_get_value
					(baseband_power_driver_data->
					modem.xmm.ipc_bb_wake);
				if (value) {
					/* Clear the slave wakeup request */
					gpio_set_value
						(baseband_power_driver_data->
						modem.xmm.ipc_bb_wake, 0);
					pr_debug("gpio slave wakeup done ->\n");
				}
			}
			/* [ver >= 1130] gpio protocol falling edge */
			if (modem_ver >= XMM_MODEM_VER_1130) {
				if (baseband_xmm_powerstate == BBXMM_PS_L2) {
					CP_initiated_L2toL0 = true;
					baseband_xmm_set_power_status
						(BBXMM_PS_L2TOL0);
				} else if (baseband_xmm_powerstate ==
								BBXMM_PS_L3) {
					spin_lock(&xmm_lock);
					wakeup_pending = true;
					spin_unlock(&xmm_lock);
					pr_info("CP L3 -> L0\n");
					ril_change_modem_crash_mode();
				}
			}
			/* save gpio state */
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			/* [ver >= 1130] gpio protocol rising edge */
			if (modem_ver >= XMM_MODEM_VER_1130) {
				pr_debug("gpio host wakeup done <-\n");
				value = gpio_get_value
					(baseband_power_driver_data->
					modem.xmm.ipc_bb_wake);
				if (value) {
					/* Clear the slave wakeup request */
					gpio_set_value
						(baseband_power_driver_data->
						modem.xmm.ipc_bb_wake, 0);
					pr_debug("gpio slave wakeup done ->\n");
					if (reenable_autosuspend && usbdev) {
						reenable_autosuspend = false;
						queue_work(workqueue,
							&autopm_resume_work);
					}
				}
				if ((baseband_xmm_powerstate ==
							BBXMM_PS_L2TOL0) ||
					(baseband_xmm_powerstate ==
							BBXMM_PS_L3TOL0))
					baseband_xmm_set_power_status(
							BBXMM_PS_L0);
				else {
					pr_info("%s:no state"
						"change required\n", __func__);
					resume_from_l3 = false;
				}
			}
			/* save gpio state */
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
	}

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(baseband_xmm_power_ipc_ap_wake_irq);

static void baseband_xmm_power_init1_work(struct work_struct *work)
{
	int value;

	pr_debug("%s {\n", __func__);

	/* check if IPC_HSIC_ACTIVE high */
	value = gpio_get_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active);
	if (value != 1) {
		pr_err("%s - expected IPC_HSIC_ACTIVE high!\n", __func__);
		return;
	}

	/* wait 100 ms */
	mdelay(100);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	/* wait 10 ms */
	mdelay(10);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	mdelay(20);

	pr_debug("%s }\n", __func__);
}

static void baseband_xmm_power_init2_work(struct work_struct *work)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;

	pr_debug("%s\n", __func__);

	/* check input */
	if (!data)
		return;

	/* register usb host controller only once */
	if (register_hsic_device) {
		if (data->hsic_register) {
			data->modem.xmm.hsic_device = data->hsic_register();
			if(modem_reset_flag == 1) {
				tegra_usb_suspend_hsic();
				mdelay(1000);
				tegra_usb_resume_hsic();
			}
		} else
			pr_err("%s: hsic_register is missing\n", __func__);
		register_hsic_device = false;
		modem_reset_flag = 0;
	}

}

static void baseband_xmm_power_autopm_resume(struct work_struct *work)
{
	struct usb_interface *intf;

	pr_debug("%s\n", __func__);
	if (usbdev) {
		usb_lock_device(usbdev);
		intf = usb_ifnum_to_if(usbdev, 0);
		usb_autopm_get_interface(intf);
		usb_autopm_put_interface(intf);
		usb_unlock_device(usbdev);
	}
}


/* Do the work for AP/CP initiated L2->L0 */
static void baseband_xmm_power_L2_resume(void)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	int value;
	int delay = 1000; /* maxmum delay in msec */

	pr_debug("%s\n", __func__);

	if (!baseband_power_driver_data)
		return;

	modem_sleep_flag = false;

	if (CP_initiated_L2toL0)  {
		pr_info("CP L2->L0\n");
		CP_initiated_L2toL0 = false;
		queue_work(workqueue, &L2_resume_work);
	} else {
		/* set the slave wakeup request */
		pr_info("AP L2->L0\n");
		gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);
		pr_debug("waiting for host wakeup from CP...\n");
		do {
			mdelay(1);
			value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
			delay--;
		} while ((value) && (delay));
		if (delay)
			pr_debug("gpio host wakeup low <-\n");
		else
			pr_info("!!AP L2->L0 Failed\n");
	}
}

/* Do the work for CP initiated L2->L0 */
static void baseband_xmm_power_L2_resume_work(struct work_struct *work)
{
	struct usb_interface *intf;

	pr_debug("%s {\n", __func__);

	if (!usbdev)
		return;
	usb_lock_device(usbdev);
	intf = usb_ifnum_to_if(usbdev, 0);

	if (!intf) {
		pr_err("%s: the interface is NULL\n", __func__);
		usb_unlock_device(usbdev);
		return;
	}

	if (usb_autopm_get_interface(intf) == 0)
		usb_autopm_put_interface(intf);
	usb_unlock_device(usbdev);

	pr_debug("} %s\n", __func__);
}

static void baseband_xmm_power_reset_on(void)
{
	pr_info("baseband_xmm_power_reset_on\n");

	/* Enable VBAT */
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_vbat, 1);

	/* reset / power on sequence */
	mdelay(40);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 1);
	udelay(40);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 0);
}

static struct baseband_xmm_power_work_t *baseband_xmm_power_work;

static void baseband_xmm_power_work_func(struct work_struct *work)
{
	struct baseband_xmm_power_work_t *bbxmm_work
		= (struct baseband_xmm_power_work_t *) work;

	pr_debug("%s\n", __func__);

	switch (bbxmm_work->state) {
	case BBXMM_WORK_UNINIT:
		pr_debug("BBXMM_WORK_UNINIT\n");
		break;
	case BBXMM_WORK_INIT:
		pr_debug("BBXMM_WORK_INIT\n");
		/* go to next state */
		bbxmm_work->state = (modem_flash && !modem_pm)
			? BBXMM_WORK_INIT_FLASH_STEP1
			: (modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASH_PM_STEP1
			: (!modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASHLESS_PM_STEP1
			: BBXMM_WORK_UNINIT;
		pr_debug("Go to next state %d\n", bbxmm_work->state);
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASH_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_STEP1\n");
		/* register usb host controller */
		pr_debug("%s: register usb host controller\n", __func__);
		if (baseband_power_driver_data->hsic_register)
			baseband_power_driver_data->modem.xmm.hsic_device =
				baseband_power_driver_data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_STEP1\n");
		/* [modem ver >= 1130] start with IPC_HSIC_ACTIVE low */
		if (modem_ver >= XMM_MODEM_VER_1130) {
			pr_debug("%s: ver > 1130:"
				" ipc_hsic_active -> 0\n", __func__);
			gpio_set_value(baseband_power_driver_data->
				modem.xmm.ipc_hsic_active, 0);
		}
		/* reset / power on sequence */
		baseband_xmm_power_reset_on();
		/* set power status as on */
		power_onoff = 1;
		/* optional delay
		 * 0 = flashless
		 *   ==> causes next step to enumerate modem boot rom
		 *       (058b / 0041)
		 * some delay > boot rom timeout
		 *   ==> causes next step to enumerate modem software
		 *       (1519 / 0020)
		 *       (requires modem to be flash version, not flashless
		 *       version)
		 */
		if (enum_delay_ms)
			mdelay(enum_delay_ms);
		/* register usb host controller */
		/* This is done on ap_wake falling edge
		pr_debug("%s: register usb host controller\n", __func__);
		if (baseband_power_driver_data->hsic_register)
			baseband_power_driver_data->modem.xmm.hsic_device =
				baseband_power_driver_data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		*/

		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1
			: BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		pr_debug("Go to next state %d\n", bbxmm_work->state);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_STEP1\n");
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ
			: BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1\n");
		break;
	default:
		break;
	}

}

static void baseband_xmm_device_add_handler(struct usb_device *udev)
{
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id;

	if (intf == NULL)
		return;

	id = usb_match_id(intf, xmm_pm_ids);

	if (id) {
		pr_debug("persist_enabled: %u\n", udev->persist_enabled);
		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		usbdev = udev;
		usb_enable_autosuspend(udev);
		pr_info("enable autosuspend\n");
		wake_unlock(&modem_recovery_wakelock);
	}
}

static void baseband_xmm_device_remove_handler(struct usb_device *udev)
{
	if (usbdev == udev) {
		wake_lock_timeout(&modem_recovery_wakelock, HZ * 60);
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		usbdev = 0;
	}

}

static int usb_xmm_notify(struct notifier_block *self, unsigned long action,
			void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		baseband_xmm_device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		baseband_xmm_device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}


static struct notifier_block usb_xmm_nb = {
	.notifier_call = usb_xmm_notify,
};

static int baseband_xmm_power_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	struct device *dev = &device->dev;
	unsigned long flags;
	int err, i;
	int ap_wake_irq;

	pr_info("%s\n", __func__);
	pr_info("[XMM] enum_delay_ms=%ld\n", enum_delay_ms);

	/* check for platform data */
	if (!data)
		return -ENODEV;

	/* check if supported modem */
	if (data->baseband_type != BASEBAND_XMM) {
		pr_err("unsuppported modem\n");
		return -ENODEV;
	}

	/* save platform data */
	baseband_power_driver_data = data;

	/* create device file */
	/*err = device_create_file(dev, &dev_attr_xmm_onoff);
	if (err < 0) {
		pr_err("%s - device_create_file failed\n", __func__);
		return -ENODEV;
	}*/

	/* init wake lock */
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "baseband_xmm_power");
	wake_lock_init(&modem_recovery_wakelock, WAKE_LOCK_SUSPEND, "modem_recovery");

	/* init spin lock */
	spin_lock_init(&xmm_lock);
	/* request baseband gpio(s) */
	tegra_baseband_gpios[0].gpio = baseband_power_driver_data
		->modem.xmm.bb_rst;
	tegra_baseband_gpios[1].gpio = baseband_power_driver_data
		->modem.xmm.bb_on;
	tegra_baseband_gpios[2].gpio = baseband_power_driver_data
		->modem.xmm.ipc_bb_wake;
	tegra_baseband_gpios[3].gpio = baseband_power_driver_data
		->modem.xmm.ipc_ap_wake;
	tegra_baseband_gpios[4].gpio = baseband_power_driver_data
		->modem.xmm.ipc_hsic_active;
	tegra_baseband_gpios[5].gpio = baseband_power_driver_data
		->modem.xmm.ipc_hsic_sus_req;
	tegra_baseband_gpios[6].gpio = baseband_power_driver_data
		->modem.xmm.bb_vbat;
	tegra_baseband_gpios[7].gpio = baseband_power_driver_data
		->modem.xmm.ipc_bb_rst_ind;
	tegra_baseband_gpios[8].gpio = baseband_power_driver_data
		->modem.xmm.ipc_bb_force_crash;
	err = gpio_request_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));
	if (err < 0) {
		pr_err("%s - request gpio(s) failed\n", __func__);
		return -ENODEV;
	}

	/* location is at /sys/devices/platform/baseband_xmm_power */
	for (i = 0; i < (ARRAY_SIZE(xmm_device_attr) - 1); i++) {
		err = device_create_file(dev, &xmm_device_attr[i]);
		if (err) {
			pr_err("create file %d failed, err = %d\n", i, err);
			goto failed_create_file;
		}
	}

	/* get regulator LDO7 for hsic power */
	if (!reg_grouper_hsic) {
		reg_grouper_hsic = regulator_get(NULL, "vddio_hsic");
		if (IS_ERR_OR_NULL(reg_grouper_hsic)) {
			pr_err("grouper 3G HSIC power on LDO7 failed\n");
			reg_grouper_hsic = NULL;
			return PTR_ERR(reg_grouper_hsic);
		}
		regulator_set_voltage(reg_grouper_hsic, 1200000, 1200000);
	}

	/* request baseband irq(s) */
	if (modem_flash && modem_pm) {
		pr_debug("%s: request_irq IPC_AP_WAKE_IRQ\n", __func__);
		ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
		err = request_threaded_irq(
			gpio_to_irq(data->modem.xmm.ipc_ap_wake),
			NULL,
			baseband_xmm_power_ipc_ap_wake_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"IPC_AP_WAKE_IRQ",
			NULL);
		if (err < 0) {
			pr_err("%s - request irq IPC_AP_WAKE_IRQ failed\n",
				__func__);
			return err;
		}
		ap_wake_irq = enable_irq_wake(gpio_to_irq(data->modem.xmm.ipc_ap_wake));
                tegra_pm_irq_set_wake_type(ap_wake_irq, IRQF_TRIGGER_FALLING);
                enable_irq_wake(ap_wake_irq);
		if (err < 0)
			 pr_err("%s: enable_irq_wake error\n", __func__);
		ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
		if (modem_ver >= XMM_MODEM_VER_1130) {
			pr_debug("%s: ver > 1130: AP_WAKE_INIT1\n", __func__);
			/* ver 1130 or later starts in INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
		}
	}

	/* init work queue */
	workqueue = create_singlethread_workqueue
		("baseband_xmm_power_workqueue");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -1;
	}
	baseband_xmm_power_work = (struct baseband_xmm_power_work_t *)
		kmalloc(sizeof(struct baseband_xmm_power_work_t), GFP_KERNEL);
	if (!baseband_xmm_power_work) {
		pr_err("cannot allocate baseband_xmm_power_work\n");
		return -1;
	}
	INIT_WORK((struct work_struct *) baseband_xmm_power_work,
		baseband_xmm_power_work_func);
	baseband_xmm_power_work->state = BBXMM_WORK_INIT;
	queue_work(workqueue,
		(struct work_struct *) baseband_xmm_power_work);

	/* init work objects */
	INIT_WORK(&init1_work, baseband_xmm_power_init1_work);
	INIT_WORK(&init2_work, baseband_xmm_power_init2_work);
	INIT_WORK(&L2_resume_work, baseband_xmm_power_L2_resume_work);
	INIT_WORK(&autopm_resume_work, baseband_xmm_power_autopm_resume);

	/* init state variables */
	register_hsic_device = true;
	CP_initiated_L2toL0 = false;
	spin_lock_irqsave(&xmm_lock, flags);
	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	wakeup_pending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	usb_register_notify(&usb_xmm_nb);

	pr_debug("%s }\n", __func__);
	return 0;

failed_create_file:
	while (i--)
		device_remove_file(dev, &xmm_device_attr[i]);
	return err;
}

static int baseband_xmm_power_driver_remove(struct platform_device *device)
{
	int i;
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	struct device *dev = &device->dev;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	usb_unregister_notify(&usb_xmm_nb);

	/* free work structure */
	kfree(baseband_xmm_power_work);
	baseband_xmm_power_work = (struct baseband_xmm_power_work_t *) 0;

	/* free baseband irq(s) */
	if (modem_flash && modem_pm) {
		free_irq(gpio_to_irq(baseband_power_driver_data
			->modem.xmm.ipc_ap_wake), NULL);
	}

	/* free baseband gpio(s) */
	gpio_free_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));

	/* disable regulator LDO7 for hsic power*/
	regulator_disable(reg_grouper_hsic);
	regulator_put(reg_grouper_hsic);
	reg_grouper_hsic = NULL;

	/* destroy wake lock */
	wake_lock_destroy(&wakelock);
	wake_lock_destroy(&modem_recovery_wakelock);

	/* delete device file */
	/*
	device_remove_file(dev, &dev_attr_xmm_onoff);
	*/
	for (i = 0; i < (ARRAY_SIZE(xmm_device_attr) - 1); i++) {
		device_remove_file(dev, &xmm_device_attr[i]);
	}

	/* unregister usb host controller */
	if (data->hsic_unregister && (!register_hsic_device)) {
		register_hsic_device = true;
		data->hsic_unregister(data->modem.xmm.hsic_device);
	}else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	return 0;
}

static int baseband_xmm_power_driver_handle_resume(
			struct baseband_power_platform_data *data)
{
	int value;
	int delay = 1000; /* maxmum delay in msec */
	unsigned long flags;

	pr_debug("%s\n", __func__);
	if (!data)
		return 0;

	/* check if modem is on */
	if (power_onoff == 0) {
		pr_debug("%s - flight mode - nop\n", __func__);
		return 0;
	}

	modem_sleep_flag = false;
	resume_from_l3 = true;

	/* L3->L0 */
	baseband_xmm_set_power_status(BBXMM_PS_L3TOL0);
	value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
	if (value) {
		pr_info("AP L3 -> L0\n");
		/* wake bb */
		gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);

		pr_debug("waiting for host wakeup...\n");
		do {
			mdelay(1);
			value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
			delay--;
		} while ((value) && (delay));
		if (delay)
			pr_debug("gpio host wakeup low <-\n");
	} else {
		pr_info("CP L3 -> L0\n");
		spin_lock_irqsave(&xmm_lock, flags);
		/* Clear wakeup pending flag */
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
	}
	reenable_autosuspend = true;

	return 0;

}

#ifdef CONFIG_PM
static int baseband_xmm_power_driver_suspend(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int baseband_xmm_power_driver_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			pdev->dev.platform_data;

	pr_debug("%s\n", __func__);
	baseband_xmm_power_driver_handle_resume(data);

	return 0;
}

static int baseband_xmm_power_suspend_noirq(struct device *dev)
{
	unsigned long flags;

	pr_debug("%s\n", __func__);
	spin_lock_irqsave(&xmm_lock, flags);
	if (wakeup_pending) {
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		pr_info("%s:**Abort Suspend: reason CP WAKEUP**\n", __func__);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&xmm_lock, flags);
	return 0;
}

static int baseband_xmm_power_resume_noirq(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops baseband_xmm_power_dev_pm_ops = {
	.suspend_noirq = baseband_xmm_power_suspend_noirq,
	.resume_noirq = baseband_xmm_power_resume_noirq,
	.suspend = baseband_xmm_power_driver_suspend,
	.resume = baseband_xmm_power_driver_resume,
};
#endif

static struct platform_driver baseband_power_driver = {
	.probe = baseband_xmm_power_driver_probe,
	.remove = baseband_xmm_power_driver_remove,
	.driver = {
		.name = "baseband_xmm_power",
#ifdef CONFIG_PM
		.pm   = &baseband_xmm_power_dev_pm_ops,
#endif
	},
};

static int __init baseband_xmm_power_init(void)
{
	pr_debug("%s\n", __func__);
	return platform_driver_register(&baseband_power_driver);
}

static void __exit baseband_xmm_power_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&baseband_power_driver);
}

module_init(baseband_xmm_power_init)
module_exit(baseband_xmm_power_exit)
