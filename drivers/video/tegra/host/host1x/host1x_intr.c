/*
 * drivers/video/tegra/host/host1x/host1x_intr.c
 *
 * Tegra Graphics Host Interrupt Management
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2010-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/mach/irq.h>

#include "nvhost_intr.h"
#include "dev.h"

/* Spacing between sync registers */
#define REGISTER_STRIDE 4

/*** HW host sync management ***/

static void syncpt_thresh_mask(struct irq_data *data)
{
	(void)data;
}

static void syncpt_thresh_unmask(struct irq_data *data)
{
	(void)data;
}

static void syncpt_thresh_cascade(unsigned int irq, struct irq_desc *desc)
{
	struct nvhost_master *dev = irq_desc_get_handler_data(desc);
	void __iomem *sync_regs = dev->sync_aperture;
	unsigned long reg;
	int i, id;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	for (i = 0; i < dev->info.nb_pts / BITS_PER_LONG; i++) {
		reg = readl(sync_regs +
				host1x_sync_syncpt_thresh_cpu0_int_status_r() +
				i * REGISTER_STRIDE);
		for_each_set_bit(id, &reg, BITS_PER_LONG)
			generic_handle_irq(id +
					dev->intr.host_syncpt_irq_base +
					i * BITS_PER_LONG);
	}

	chained_irq_exit(chip, desc);
}

static struct irq_chip syncpt_thresh_irq = {
	.name		= "syncpt",
	.irq_mask	= syncpt_thresh_mask,
	.irq_unmask	= syncpt_thresh_unmask
};

static void t20_intr_init_host_sync(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;
	int i, irq;

	writel(0xffffffffUL,
		sync_regs + host1x_sync_syncpt_thresh_int_disable_r());
	writel(0xffffffffUL,
		sync_regs + host1x_sync_syncpt_thresh_cpu0_int_status_r());

	for (i = 0; i < dev->info.nb_pts; i++) {
		irq = intr->host_syncpt_irq_base + i;
		irq_set_chip_and_handler(irq, &syncpt_thresh_irq,
			handle_simple_irq);
		irq_set_chip_data(irq, sync_regs);
		set_irq_flags(irq, IRQF_VALID);
	}
	irq_set_chained_handler(INT_HOST1X_MPCORE_SYNCPT,
		syncpt_thresh_cascade);
	irq_set_handler_data(INT_HOST1X_MPCORE_SYNCPT, dev);
	/* disable the ip_busy_timeout. this prevents write drops, etc.
	 * there's no real way to recover from a hung client anyway.
	 */
	writel(0, sync_regs + host1x_sync_ip_busy_timeout_r());

	/* increase the auto-ack timout to the maximum value. 2d will hang
	 * otherwise on ap20.
	 */
	writel(0xff, sync_regs + host1x_sync_ctxsw_timeout_cfg_r());
}

static void t20_intr_set_host_clocks_per_usec(struct nvhost_intr *intr, u32 cpm)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;
	/* write microsecond clock register */
	writel(cpm, sync_regs + host1x_sync_usec_clk_r());
}

static void t20_intr_set_syncpt_threshold(struct nvhost_intr *intr,
	u32 id, u32 thresh)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;
	thresh &= 0xffff;
	writel(thresh, sync_regs +
		(host1x_sync_syncpt_int_thresh_0_r() + id * REGISTER_STRIDE));
}

static void t20_intr_enable_syncpt_intr(struct nvhost_intr *intr, u32 id)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;

	writel(BIT_MASK(id), sync_regs +
			host1x_sync_syncpt_thresh_int_enable_cpu0_r() +
			BIT_WORD(id) * REGISTER_STRIDE);
}

static void t20_intr_disable_syncpt_intr(struct nvhost_intr *intr, u32 id)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;

	writel(BIT_MASK(id), sync_regs +
			host1x_sync_syncpt_thresh_int_disable_r() +
			BIT_WORD(id) * REGISTER_STRIDE);
}

static void t20_intr_disable_all_syncpt_intrs(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;
	u32 reg;

	for (reg = 0; reg <= BIT_WORD(dev->info.nb_pts) * REGISTER_STRIDE;
			reg += REGISTER_STRIDE) {
		/* disable interrupts for both cpu's */
		writel(0xffffffffu, sync_regs +
				host1x_sync_syncpt_thresh_int_disable_r() +
				reg);

		/* clear status for both cpu's */
		writel(0xffffffffu, sync_regs +
			host1x_sync_syncpt_thresh_cpu0_int_status_r() + reg);
		writel(0xffffffffu, sync_regs +
			host1x_sync_syncpt_thresh_cpu1_int_status_r() + reg);
	}
}

/**
 * Sync point threshold interrupt service function
 * Handles sync point threshold triggers, in interrupt context
 */
static irqreturn_t t20_intr_syncpt_thresh_isr(int irq, void *dev_id)
{
	struct nvhost_intr_syncpt *syncpt = dev_id;
	unsigned int id = syncpt->id;
	struct nvhost_intr *intr = intr_syncpt_to_intr(syncpt);

	void __iomem *sync_regs = intr_to_dev(intr)->sync_aperture;

	u32 reg = BIT_WORD(id) * REGISTER_STRIDE;

	writel(BIT_MASK(id), sync_regs +
		host1x_sync_syncpt_thresh_int_disable_r() + reg);
	writel(BIT_MASK(id), sync_regs +
		host1x_sync_syncpt_thresh_cpu0_int_status_r() + reg);

	return IRQ_WAKE_THREAD;
}

/**
 * Host general interrupt service function
 * Handles read / write failures
 */
static irqreturn_t t20_intr_host1x_isr(int irq, void *dev_id)
{
	struct nvhost_intr *intr = dev_id;
	void __iomem *sync_regs = intr_to_dev(intr)->sync_aperture;
	u32 stat;
	u32 ext_stat;
	u32 addr;

	stat = readl(sync_regs + host1x_sync_hintstatus_r());
	ext_stat = readl(sync_regs + host1x_sync_hintstatus_ext_r());

	if (host1x_sync_hintstatus_ext_ip_read_int_v(ext_stat)) {
		addr = readl(sync_regs + host1x_sync_ip_read_timeout_addr_r());
		pr_err("Host read timeout at address %x\n", addr);
	}

	if (host1x_sync_hintstatus_ext_ip_write_int_v(ext_stat)) {
		addr = readl(sync_regs + host1x_sync_ip_write_timeout_addr_r());
		pr_err("Host write timeout at address %x\n", addr);
	}

	writel(ext_stat, sync_regs + host1x_sync_hintstatus_ext_r());
	writel(stat, sync_regs + host1x_sync_hintstatus_r());

	return IRQ_HANDLED;
}
static int t20_intr_request_host_general_irq(struct nvhost_intr *intr)
{
	void __iomem *sync_regs = intr_to_dev(intr)->sync_aperture;
	int err;

	if (intr->host_general_irq_requested)
		return 0;

	/* master disable for general (not syncpt) host interrupts */
	writel(0, sync_regs + host1x_sync_intmask_r());

	/* clear status & extstatus */
	writel(0xfffffffful, sync_regs + host1x_sync_hintstatus_ext_r());
	writel(0xfffffffful, sync_regs + host1x_sync_hintstatus_r());

	err = request_irq(intr->host_general_irq, t20_intr_host1x_isr, 0,
			"host_status", intr);
	if (err)
		return err;

	/* enable extra interrupt sources IP_READ_INT and IP_WRITE_INT */
	writel(BIT(30) | BIT(31), sync_regs + host1x_sync_hintmask_ext_r());

	/* enable extra interrupt sources */
	writel(BIT(31), sync_regs + host1x_sync_hintmask_r());

	/* enable host module interrupt to CPU0 */
	writel(BIT(0), sync_regs + host1x_sync_intc0mask_r());

	/* master enable for general (not syncpt) host interrupts */
	writel(BIT(0), sync_regs + host1x_sync_intmask_r());

	intr->host_general_irq_requested = true;

	return err;
}

static void t20_intr_free_host_general_irq(struct nvhost_intr *intr)
{
	if (intr->host_general_irq_requested) {
		void __iomem *sync_regs = intr_to_dev(intr)->sync_aperture;

		/* master disable for general (not syncpt) host interrupts */
		writel(0, sync_regs + host1x_sync_intmask_r());

		free_irq(intr->host_general_irq, intr);
		intr->host_general_irq_requested = false;
	}
}

static int t20_request_syncpt_irq(struct nvhost_intr_syncpt *syncpt)
{
	int err;
	if (syncpt->irq_requested)
		return 0;

	err = request_threaded_irq(syncpt->irq,
				   t20_intr_syncpt_thresh_isr,
				   nvhost_syncpt_thresh_fn,
				   0, syncpt->thresh_irq_name, syncpt);
	if (err)
		return err;

	syncpt->irq_requested = 1;
	return 0;
}

static const struct nvhost_intr_ops host1x_intr_ops = {
	.init_host_sync = t20_intr_init_host_sync,
	.set_host_clocks_per_usec = t20_intr_set_host_clocks_per_usec,
	.set_syncpt_threshold = t20_intr_set_syncpt_threshold,
	.enable_syncpt_intr = t20_intr_enable_syncpt_intr,
	.disable_syncpt_intr = t20_intr_disable_syncpt_intr,
	.disable_all_syncpt_intrs = t20_intr_disable_all_syncpt_intrs,
	.request_host_general_irq = t20_intr_request_host_general_irq,
	.free_host_general_irq = t20_intr_free_host_general_irq,
	.request_syncpt_irq = t20_request_syncpt_irq,
};
