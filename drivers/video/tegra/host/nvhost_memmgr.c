/*
 * drivers/video/tegra/host/nvhost_memmgr.c
 *
 * Tegra Graphics Host Memory Management Abstraction
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/err.h>

#include "nvhost_memmgr.h"
#include "nvmap.h"

int nvhost_memmgr_init(struct nvhost_chip_support *chip)
{
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	return nvhost_init_nvmap_support(chip);
#endif
	BUG_ON(!"No memory manager selected");
	return -ENODEV;
}
