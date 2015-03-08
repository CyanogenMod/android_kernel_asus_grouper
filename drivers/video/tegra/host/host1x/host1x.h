/*
 * drivers/video/tegra/host/host1x/host1x.h
 *
 * Tegra Graphics Host Driver Entrypoint
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

#ifndef __NVHOST_HOST1X_H
#define __NVHOST_HOST1X_H

#include <linux/cdev.h>
#include <linux/nvhost.h>

#include "nvhost_syncpt.h"
#include "nvhost_intr.h"

#define TRACE_MAX_LENGTH	128U
#define IFACE_NAME		"nvhost"

struct nvhost_channel;
struct mem_mgr;

struct host1x_device_info {
	int		nb_channels;	/* host1x: num channels supported */
	int		nb_pts; 	/* host1x: num syncpoints supported */
	int		nb_bases;	/* host1x: num syncpoints supported */
	u32		client_managed; /* host1x: client managed syncpts */
	int		nb_mlocks;	/* host1x: number of mlocks */
	const char	**syncpt_names;	/* names of sync points */
};

struct nvhost_master {
	void __iomem *aperture;
	void __iomem *sync_aperture;
	struct resource *reg_mem;
	struct class *nvhost_class;
	struct cdev cdev;
	struct device *ctrl;
	struct nvhost_syncpt syncpt;
	struct mem_mgr *memmgr;
	struct nvhost_intr intr;
	struct nvhost_device *dev;
	atomic_t clientid;

	struct host1x_device_info info;
};

extern struct nvhost_master *nvhost;

void nvhost_debug_init(struct nvhost_master *master);
void nvhost_debug_dump(struct nvhost_master *master);

struct nvhost_channel *nvhost_alloc_channel(struct nvhost_device *dev);
void nvhost_free_channel(struct nvhost_channel *ch);

extern pid_t nvhost_debug_null_kickoff_pid;

static inline struct nvhost_master *nvhost_get_host(struct nvhost_device *_dev)
{
	return (_dev->dev.parent) ? \
		((struct nvhost_master *) dev_get_drvdata(_dev->dev.parent)) : \
		((struct nvhost_master *) dev_get_drvdata(&(_dev->dev)));
}

#endif
