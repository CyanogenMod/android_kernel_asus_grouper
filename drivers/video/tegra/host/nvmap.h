/*
 * drivers/video/tegra/host/nvmap.h
 *
 * Tegra Graphics Host nvmap memory manager
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

#ifndef __NVHOST_NVMAP_H
#define __NVHOST_NVMAP_H

struct nvhost_chip_support;
int nvhost_init_nvmap_support(struct nvhost_chip_support *op);

#endif
