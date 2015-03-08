/*
 * drivers/video/tegra/host/nvhost_acm.h
 *
 * Tegra Graphics Host Automatic Clock Management
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

#ifndef __NVHOST_ACM_H
#define __NVHOST_ACM_H

#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/nvhost.h>

/* Sets clocks and powergating state for a module */
int nvhost_module_init(struct nvhost_device *ndev);
void nvhost_module_deinit(struct nvhost_device *dev);
int nvhost_module_suspend(struct nvhost_device *dev);

void nvhost_module_reset(struct nvhost_device *dev);
void nvhost_module_busy(struct nvhost_device *dev);
void nvhost_module_idle_mult(struct nvhost_device *dev, int refs);
int nvhost_module_add_client(struct nvhost_device *dev,
		void *priv);
void nvhost_module_remove_client(struct nvhost_device *dev,
		void *priv);
int nvhost_module_get_rate(struct nvhost_device *dev,
		unsigned long *rate,
		int index);
int nvhost_module_set_rate(struct nvhost_device *dev, void *priv,
		unsigned long rate, int index);

static inline bool nvhost_module_powered(struct nvhost_device *dev)
{
	return dev->powerstate == NVHOST_POWER_STATE_RUNNING;
}

static inline void nvhost_module_idle(struct nvhost_device *dev)
{
	nvhost_module_idle_mult(dev, 1);
}

#endif
