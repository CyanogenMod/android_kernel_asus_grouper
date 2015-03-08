/*
 * drivers/video/tegra/host/dev.c
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

#define CREATE_TRACE_POINTS
#include <trace/events/nvhost.h>

#include <linux/nvhost.h>
#include <mach/gpufuse.h>

MODULE_AUTHOR("NVIDIA");
MODULE_DESCRIPTION("Graphics host driver for Tegra products");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform-nvhost");
