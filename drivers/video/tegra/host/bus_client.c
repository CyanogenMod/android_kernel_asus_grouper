/*
 * drivers/video/tegra/host/bus_client.c
 *
 * Tegra Graphics Host Client Module
 *
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

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/file.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>

#include <trace/events/nvhost.h>

#include <linux/io.h>
#include <linux/string.h>

#include <linux/nvhost.h>
#include <linux/nvhost_ioctl.h>

#include <mach/gpufuse.h>
#include <mach/hardware.h>
#include <mach/iomap.h>

#include "debug.h"
#include "bus_client.h"
#include "dev.h"
#include "nvhost_memmgr.h"
#include "chip_support.h"
#include "nvhost_acm.h"

#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "nvhost_hwctx.h"

static int validate_reg(struct nvhost_device *ndev, u32 offset, int count)
{
	struct resource *r = nvhost_get_resource(ndev, IORESOURCE_MEM, 0);
	int err = 0;

	if (offset + 4 * count > resource_size(r)
			|| (offset + 4 * count < offset))
		err = -EPERM;

	return err;
}

int nvhost_read_module_regs(struct nvhost_device *ndev,
			u32 offset, int count, u32 *values)
{
	void __iomem *p = ndev->aperture + offset;
	int err;

	/* verify offset */
	err = validate_reg(ndev, offset, count);
	if (err)
		return err;

	nvhost_module_busy(ndev);
	while (count--) {
		*(values++) = readl(p);
		p += 4;
	}
	rmb();
	nvhost_module_idle(ndev);

	return 0;
}

int nvhost_write_module_regs(struct nvhost_device *ndev,
			u32 offset, int count, const u32 *values)
{
	void __iomem *p = ndev->aperture + offset;
	int err;

	/* verify offset */
	err = validate_reg(ndev, offset, count);
	if (err)
		return err;

	nvhost_module_busy(ndev);
	while (count--) {
		writel(*(values++), p);
		p += 4;
	}
	wmb();
	nvhost_module_idle(ndev);

	return 0;
}

struct nvhost_channel_userctx {
	struct nvhost_channel *ch;
	struct nvhost_hwctx *hwctx;
	struct nvhost_submit_hdr_ext hdr;
	int num_relocshifts;
	struct nvhost_job *job;
	struct mem_mgr *memmgr;
	u32 timeout;
	u32 priority;
	int clientid;
};

static int nvhost_channelrelease(struct inode *inode, struct file *filp)
{
	struct nvhost_channel_userctx *priv = filp->private_data;

	trace_nvhost_channel_release(priv->ch->dev->name);

	filp->private_data = NULL;

	nvhost_module_remove_client(priv->ch->dev, priv);
	nvhost_putchannel(priv->ch, priv->hwctx);

	if (priv->hwctx)
		priv->ch->ctxhandler->put(priv->hwctx);

	if (priv->job)
		nvhost_job_put(priv->job);

	mem_op().put_mgr(priv->memmgr);
	kfree(priv);
	return 0;
}

static int nvhost_channelopen(struct inode *inode, struct file *filp)
{
	struct nvhost_channel_userctx *priv;
	struct nvhost_channel *ch;

	ch = container_of(inode->i_cdev, struct nvhost_channel, cdev);
	ch = nvhost_getchannel(ch);
	if (!ch)
		return -ENOMEM;
	trace_nvhost_channel_open(ch->dev->name);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		nvhost_putchannel(ch, NULL);
		return -ENOMEM;
	}
	filp->private_data = priv;
	priv->ch = ch;
	if(nvhost_module_add_client(ch->dev, priv))
		goto fail;

	if (ch->ctxhandler && ch->ctxhandler->alloc) {
		priv->hwctx = ch->ctxhandler->alloc(ch->ctxhandler, ch);
		if (!priv->hwctx)
			goto fail;
	}
	priv->priority = NVHOST_PRIORITY_MEDIUM;
	priv->clientid = atomic_add_return(1,
			&nvhost_get_host(ch->dev)->clientid);
	priv->timeout = CONFIG_TEGRA_GRHOST_DEFAULT_TIMEOUT;

	return 0;
fail:
	nvhost_channelrelease(inode, filp);
	return -ENOMEM;
}

static int set_submit(struct nvhost_channel_userctx *ctx)
{
	struct nvhost_device *ndev = ctx->ch->dev;
	struct nvhost_master *host = nvhost_get_host(ndev);

	/* submit should have at least 1 cmdbuf */
	if (!ctx->hdr.num_cmdbufs ||
			!nvhost_syncpt_is_valid(&host->syncpt,
				ctx->hdr.syncpt_id))
		return -EIO;

	if (!ctx->memmgr) {
		dev_err(&ndev->dev, "no nvmap context set\n");
		return -EFAULT;
	}

	ctx->job = nvhost_job_alloc(ctx->ch,
			ctx->hwctx,
			&ctx->hdr,
			ctx->memmgr,
			ctx->priority,
			ctx->clientid);
	if (!ctx->job)
		return -ENOMEM;
	ctx->job->timeout = ctx->timeout;

	if (ctx->hdr.submit_version >= NVHOST_SUBMIT_VERSION_V2)
		ctx->num_relocshifts = ctx->hdr.num_relocs;

	return 0;
}

static void reset_submit(struct nvhost_channel_userctx *ctx)
{
	ctx->hdr.num_cmdbufs = 0;
	ctx->hdr.num_relocs = 0;
	ctx->num_relocshifts = 0;
	ctx->hdr.num_waitchks = 0;

	if (ctx->job) {
		nvhost_job_put(ctx->job);
		ctx->job = NULL;
	}
}

static ssize_t nvhost_channelwrite(struct file *filp, const char __user *buf,
				size_t count, loff_t *offp)
{
	struct nvhost_channel_userctx *priv = filp->private_data;
	size_t remaining = count;
	int err = 0;
	struct nvhost_job *job = priv->job;
	struct nvhost_submit_hdr_ext *hdr = &priv->hdr;
	const char *chname = priv->ch->dev->name;

	if (!job)
		return -EIO;

	while (remaining) {
		size_t consumed;
		if (!hdr->num_relocs &&
		    !priv->num_relocshifts &&
		    !hdr->num_cmdbufs &&
		    !hdr->num_waitchks) {
			consumed = sizeof(struct nvhost_submit_hdr);
			if (remaining < consumed)
				break;
			if (copy_from_user(hdr, buf, consumed)) {
				err = -EFAULT;
				break;
			}
			hdr->submit_version = NVHOST_SUBMIT_VERSION_V0;
			err = set_submit(priv);
			if (err)
				break;
			trace_nvhost_channel_write_submit(chname,
			  count, hdr->num_cmdbufs, hdr->num_relocs,
			  hdr->syncpt_id, hdr->syncpt_incrs);
		} else if (hdr->num_cmdbufs) {
			struct nvhost_cmdbuf cmdbuf;
			consumed = sizeof(cmdbuf);
			if (remaining < consumed)
				break;
			if (copy_from_user(&cmdbuf, buf, consumed)) {
				err = -EFAULT;
				break;
			}
			trace_nvhost_channel_write_cmdbuf(chname,
				cmdbuf.mem, cmdbuf.words, cmdbuf.offset);
			nvhost_job_add_gather(job,
				cmdbuf.mem, cmdbuf.words, cmdbuf.offset);
			hdr->num_cmdbufs--;
		} else if (hdr->num_relocs) {
			int numrelocs = remaining / sizeof(struct nvhost_reloc);
			if (!numrelocs)
				break;
			numrelocs = min_t(int, numrelocs, priv->hdr.num_relocs);
			consumed = numrelocs * sizeof(struct nvhost_reloc);
			if (copy_from_user(&job->relocarray[job->num_relocs],
					buf, consumed)) {
				err = -EFAULT;
				break;
			}
			while (numrelocs) {
				struct nvhost_reloc *reloc =
					&job->relocarray[job->num_relocs];
				trace_nvhost_channel_write_reloc(chname,
					reloc->cmdbuf_mem,
					reloc->cmdbuf_offset,
					reloc->target,
					reloc->target_offset);
				job->num_relocs++;
				hdr->num_relocs--;
				numrelocs--;
			}
		} else if (hdr->num_waitchks) {
			int numwaitchks =
				(remaining / sizeof(struct nvhost_waitchk));
			if (!numwaitchks)
				break;
			numwaitchks = min_t(int,
				numwaitchks, hdr->num_waitchks);
			consumed = numwaitchks * sizeof(struct nvhost_waitchk);
			if (copy_from_user(&job->waitchk[job->num_waitchk],
					buf, consumed)) {
				err = -EFAULT;
				break;
			}
			trace_nvhost_channel_write_waitchks(
			  chname, numwaitchks,
			  hdr->waitchk_mask);
			job->num_waitchk += numwaitchks;
			hdr->num_waitchks -= numwaitchks;
		} else if (priv->num_relocshifts) {
			int next_shift =
				job->num_relocs - priv->num_relocshifts;
			int num =
				(remaining / sizeof(struct nvhost_reloc_shift));
			if (!num)
				break;
			num = min_t(int, num, priv->num_relocshifts);
			consumed = num * sizeof(struct nvhost_reloc_shift);
			if (copy_from_user(&job->relocshiftarray[next_shift],
					buf, consumed)) {
				err = -EFAULT;
				break;
			}
			priv->num_relocshifts -= num;
		} else {
			err = -EFAULT;
			break;
		}
		remaining -= consumed;
		buf += consumed;
	}

	if (err < 0) {
		dev_err(&priv->ch->dev->dev, "channel write error\n");
		reset_submit(priv);
		return err;
	}

	return count - remaining;
}

static int nvhost_ioctl_channel_flush(
	struct nvhost_channel_userctx *ctx,
	struct nvhost_get_param_args *args,
	int null_kickoff)
{
	struct nvhost_device *ndev = to_nvhost_device(&ctx->ch->dev->dev);
	int err;

	trace_nvhost_ioctl_channel_flush(ctx->ch->dev->name);

	if (!ctx->job ||
	    ctx->hdr.num_relocs ||
	    ctx->hdr.num_cmdbufs ||
	    ctx->hdr.num_waitchks) {
		reset_submit(ctx);
		dev_err(&ndev->dev, "channel submit out of sync\n");
		return -EFAULT;
	}

	err = nvhost_job_pin(ctx->job, &nvhost_get_host(ndev)->syncpt);
	if (err) {
		dev_warn(&ndev->dev, "nvhost_job_pin failed: %d\n", err);
		return err;
	}

	if (nvhost_debug_null_kickoff_pid == current->tgid)
		null_kickoff = 1;
	ctx->job->null_kickoff = null_kickoff;

	if ((nvhost_debug_force_timeout_pid == current->tgid) &&
	    (nvhost_debug_force_timeout_channel == ctx->ch->chid)) {
		ctx->timeout = nvhost_debug_force_timeout_val;
	}

	/* context switch if needed, and submit user's gathers to the channel */
	err = nvhost_channel_submit(ctx->job);
	args->value = ctx->job->syncpt_end;
	if (err)
		nvhost_job_unpin(ctx->job);

	nvhost_job_put(ctx->job);
	ctx->job = NULL;

	return err;
}

static int nvhost_ioctl_channel_read_3d_reg(struct nvhost_channel_userctx *ctx,
	struct nvhost_read_3d_reg_args *args)
{
	BUG_ON(!channel_op().read3dreg);
	return channel_op().read3dreg(ctx->ch, ctx->hwctx,
			args->offset, &args->value);
}

static long nvhost_channelctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct nvhost_channel_userctx *priv = filp->private_data;
	u8 buf[NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE];
	int err = 0;

	if ((_IOC_TYPE(cmd) != NVHOST_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_IOCTL_CHANNEL_LAST) ||
		(_IOC_SIZE(cmd) > NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE))
		return -EFAULT;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {
	case NVHOST_IOCTL_CHANNEL_FLUSH:
		err = nvhost_ioctl_channel_flush(priv, (void *)buf, 0);
		break;
	case NVHOST_IOCTL_CHANNEL_NULL_KICKOFF:
		err = nvhost_ioctl_channel_flush(priv, (void *)buf, 1);
		break;
	case NVHOST_IOCTL_CHANNEL_SUBMIT_EXT:
	{
		struct nvhost_submit_hdr_ext *hdr;

		if (priv->hdr.num_relocs ||
		    priv->num_relocshifts ||
		    priv->hdr.num_cmdbufs ||
		    priv->hdr.num_waitchks) {
			reset_submit(priv);
			dev_err(&priv->ch->dev->dev,
				"channel submit out of sync\n");
			err = -EIO;
			break;
		}

		hdr = (struct nvhost_submit_hdr_ext *)buf;
		if (hdr->submit_version > NVHOST_SUBMIT_VERSION_MAX_SUPPORTED) {
			dev_err(&priv->ch->dev->dev,
				"submit version %d > max supported %d\n",
				hdr->submit_version,
				NVHOST_SUBMIT_VERSION_MAX_SUPPORTED);
			err = -EINVAL;
			break;
		}
		memcpy(&priv->hdr, hdr, sizeof(struct nvhost_submit_hdr_ext));
		err = set_submit(priv);
		trace_nvhost_ioctl_channel_submit(priv->ch->dev->name,
			priv->hdr.submit_version,
			priv->hdr.num_cmdbufs, priv->hdr.num_relocs,
			priv->hdr.num_waitchks,
			priv->hdr.syncpt_id, priv->hdr.syncpt_incrs);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_SYNCPOINTS:
		/* host syncpt ID is used by the RM (and never be given out) */
		BUG_ON(priv->ch->dev->syncpts & (1 << NVSYNCPT_GRAPHICS_HOST));
		((struct nvhost_get_param_args *)buf)->value =
			priv->ch->dev->syncpts;
		break;
	case NVHOST_IOCTL_CHANNEL_GET_WAITBASES:
		((struct nvhost_get_param_args *)buf)->value =
			priv->ch->dev->waitbases;
		break;
	case NVHOST_IOCTL_CHANNEL_GET_MODMUTEXES:
		((struct nvhost_get_param_args *)buf)->value =
			priv->ch->dev->modulemutexes;
		break;
	case NVHOST_IOCTL_CHANNEL_SET_NVMAP_FD:
	{
		int fd = (int)((struct nvhost_set_nvmap_fd_args *)buf)->fd;
		struct mem_mgr *new_client = mem_op().get_mgr_file(fd);

		if (IS_ERR(new_client)) {
			err = PTR_ERR(new_client);
			break;
		}

		if (priv->memmgr)
			mem_op().put_mgr(priv->memmgr);

		priv->memmgr = new_client;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_READ_3D_REG:
		err = nvhost_ioctl_channel_read_3d_reg(priv, (void *)buf);
		break;
	case NVHOST_IOCTL_CHANNEL_GET_CLK_RATE:
	{
		unsigned long rate;
		struct nvhost_clk_rate_args *arg =
				(struct nvhost_clk_rate_args *)buf;

		err = nvhost_module_get_rate(priv->ch->dev, &rate, 0);
		if (err == 0)
			arg->rate = rate;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_CLK_RATE:
	{
		struct nvhost_clk_rate_args *arg =
				(struct nvhost_clk_rate_args *)buf;
		unsigned long rate = (unsigned long)arg->rate;

		err = nvhost_module_set_rate(priv->ch->dev, priv, rate, 0);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_TIMEOUT:
		priv->timeout =
			(u32)((struct nvhost_set_timeout_args *)buf)->timeout;
		dev_dbg(&priv->ch->dev->dev,
			"%s: setting buffer timeout (%d ms) for userctx 0x%p\n",
			__func__, priv->timeout, priv);
		break;
	case NVHOST_IOCTL_CHANNEL_GET_TIMEDOUT:
		((struct nvhost_get_param_args *)buf)->value =
				priv->hwctx->has_timedout;
		break;
	case NVHOST_IOCTL_CHANNEL_SET_PRIORITY:
		priv->priority =
			(u32)((struct nvhost_set_priority_args *)buf)->priority;
		break;
	default:
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static const struct file_operations nvhost_channelops = {
	.owner = THIS_MODULE,
	.release = nvhost_channelrelease,
	.open = nvhost_channelopen,
	.write = nvhost_channelwrite,
	.unlocked_ioctl = nvhost_channelctl
};

int nvhost_client_user_init(struct nvhost_device *dev)
{
	int err, devno;

	struct nvhost_channel *ch = dev->channel;
	err = alloc_chrdev_region(&devno, 0, 1, IFACE_NAME);
	if (err < 0) {
		dev_err(&dev->dev, "failed to allocate devno\n");
		goto fail;
	}

	cdev_init(&ch->cdev, &nvhost_channelops);
	ch->cdev.owner = THIS_MODULE;

	err = cdev_add(&ch->cdev, devno, 1);
	if (err < 0) {
		dev_err(&dev->dev,
			"failed to add chan %i cdev\n", dev->index);
		goto fail;
	}
	ch->node = device_create(nvhost_get_host(dev)->nvhost_class,
			NULL, devno, NULL,
			IFACE_NAME "-%s", dev->name);
	if (IS_ERR(ch->node)) {
		err = PTR_ERR(ch->node);
		dev_err(&dev->dev,
			"failed to create %s channel device\n", dev->name);
		goto fail;
	}

	return 0;
fail:
	return err;
}

int nvhost_client_device_init(struct nvhost_device *dev)
{
	int err;
	struct nvhost_master *nvhost_master = nvhost_get_host(dev);
	struct nvhost_channel *ch;

	ch = nvhost_alloc_channel(dev);
	if (ch == NULL)
		return -ENODEV;

	/* store the pointer to this device for channel */
	ch->dev = dev;

	err = nvhost_channel_init(ch, nvhost_master, dev->index);
	if (err)
		goto fail;

	err = nvhost_client_user_init(dev);
	if (err)
		goto fail;

	err = nvhost_module_init(dev);
	if (err)
		goto fail;

	dev_info(&dev->dev, "initialized\n");

	return 0;

fail:
	/* Add clean-up */
	nvhost_free_channel(ch);
	return err;
}

int nvhost_client_device_suspend(struct nvhost_device *dev)
{
	int ret = 0;

	ret = nvhost_channel_suspend(dev->channel);
	if (ret)
		return ret;

	dev_info(&dev->dev, "suspend status: %d\n", ret);

	return ret;
}

int nvhost_client_device_get_resources(struct nvhost_device *dev)
{
	struct resource *r = NULL;
	void __iomem *regs = NULL;
	struct resource *reg_mem = NULL;

	r = nvhost_get_resource(dev, IORESOURCE_MEM, 0);
	if (!r)
		goto fail;

	reg_mem = request_mem_region(r->start, resource_size(r), dev->name);
	if (!reg_mem)
		goto fail;

	regs = ioremap(r->start, resource_size(r));
	if (!regs)
		goto fail;

	dev->reg_mem = reg_mem;
	dev->aperture = regs;

	return 0;

fail:
	if (reg_mem)
		release_mem_region(r->start, resource_size(r));
	if (regs)
		iounmap(regs);

	dev_err(&dev->dev, "failed to get register memory\n");

	return -ENXIO;
}

void nvhost_client_device_put_resources(struct nvhost_device *dev)
{
	struct resource *r;

	r = nvhost_get_resource(dev, IORESOURCE_MEM, 0);
	BUG_ON(!r);

	iounmap(dev->aperture);

	release_mem_region(r->start, resource_size(r));
}
