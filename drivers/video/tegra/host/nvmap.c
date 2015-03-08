/*
 * drivers/video/tegra/host/nvmap.c
 *
 * Tegra Graphics Host Nvmap support
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

#include "chip_support.h"
#include <linux/nvmap.h>

struct mem_mgr *nvhost_nvmap_alloc_mgr(void)
{
	return (struct mem_mgr *)nvmap_create_client(nvmap_dev, "nvhost");
}

void nvhost_nvmap_put_mgr(struct mem_mgr *mgr)
{
	nvmap_client_put((struct nvmap_client *)mgr);
}

struct mem_mgr *nvhost_nvmap_get_mgr(struct mem_mgr *mgr)
{
	return (struct mem_mgr *)nvmap_client_get((struct nvmap_client *)mgr);
}

struct mem_mgr *nvhost_nvmap_get_mgr_file(int fd)
{
	return (struct mem_mgr *)nvmap_client_get_file(fd);
}

struct mem_handle *nvhost_nvmap_alloc(struct mem_mgr *mgr,
		size_t size, size_t align, int flags)
{
	return (struct mem_handle *)nvmap_alloc((struct nvmap_client *)mgr,
			size, align, flags, 0);
}

void nvhost_nvmap_put(struct mem_mgr *mgr, struct mem_handle *handle)
{
	return nvmap_free((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle);
}

phys_addr_t nvhost_nvmap_pin(struct mem_mgr *mgr, struct mem_handle *handle)
{
	return nvmap_pin((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle);
}

void nvhost_nvmap_unpin(struct mem_mgr *mgr, struct mem_handle *handle)
{
	return nvmap_unpin((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle);
}

void *nvhost_nvmap_mmap(struct mem_handle *handle)
{
	return nvmap_mmap((struct nvmap_handle_ref *)handle);
}

void nvhost_nvmap_munmap(struct mem_handle *handle, void *addr)
{
	nvmap_munmap((struct nvmap_handle_ref *)handle, addr);
}

struct mem_handle *nvhost_nvmap_get(struct mem_mgr *mgr, u32 id)
{
	return (struct mem_handle *)
		nvmap_duplicate_handle_id((struct nvmap_client *)mgr, id);
}

int nvhost_init_nvmap_support(struct nvhost_chip_support *chip)
{
	chip->mem.alloc_mgr = nvhost_nvmap_alloc_mgr;
	chip->mem.put_mgr = nvhost_nvmap_put_mgr;
	chip->mem.get_mgr = nvhost_nvmap_get_mgr;
	chip->mem.get_mgr_file = nvhost_nvmap_get_mgr_file;
	chip->mem.alloc = nvhost_nvmap_alloc;
	chip->mem.put = nvhost_nvmap_put;
	chip->mem.get = nvhost_nvmap_get;
	chip->mem.pin = nvhost_nvmap_pin;
	chip->mem.unpin = nvhost_nvmap_unpin;
	chip->mem.mmap = nvhost_nvmap_mmap;
	chip->mem.munmap = nvhost_nvmap_munmap;

	return 0;
}
