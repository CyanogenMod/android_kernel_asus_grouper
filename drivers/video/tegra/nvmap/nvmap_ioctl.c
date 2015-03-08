/*
 * drivers/video/tegra/nvmap/nvmap_ioctl.c
 *
 * User-space interface to nvmap
 *
 * Copyright (c) 2011-2012, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/nvmap.h>

#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <asm/tlbflush.h>

#include <mach/iovmm.h>
#include <trace/events/nvmap.h>

#include "nvmap_ioctl.h"
#include "nvmap.h"
#include "nvmap_common.h"

static ssize_t rw_handle(struct nvmap_client *client, struct nvmap_handle *h,
			 int is_read, unsigned long h_offs,
			 unsigned long sys_addr, unsigned long h_stride,
			 unsigned long sys_stride, unsigned long elem_size,
			 unsigned long count);

static int cache_maint(struct nvmap_client *client, struct nvmap_handle *h,
		       unsigned long start, unsigned long end, unsigned int op);


int nvmap_ioctl_pinop(struct file *filp, bool is_pin, void __user *arg)
{
	struct nvmap_pin_handle op;
	struct nvmap_handle *h;
	unsigned long on_stack[16];
	unsigned long *refs;
	unsigned long __user *output;
	unsigned int i;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.count)
		return -EINVAL;

	if (op.count > 1) {
		size_t bytes = op.count * sizeof(*refs); /* kcalloc below will catch overflow. */

		if (op.count > ARRAY_SIZE(on_stack))
			refs = kcalloc(op.count, sizeof(*refs), GFP_KERNEL);
		else
			refs = on_stack;

		if (!refs)
			return -ENOMEM;

		if (copy_from_user(refs, (void *)op.handles, bytes)) {
			err = -EFAULT;
			goto out;
		}
	} else {
		refs = on_stack;
		on_stack[0] = (unsigned long)op.handles;
	}

	trace_nvmap_ioctl_pinop(filp->private_data, is_pin, op.count, refs);
	if (is_pin)
		err = nvmap_pin_ids(filp->private_data, op.count, refs);
	else
		nvmap_unpin_ids(filp->private_data, op.count, refs);

	/* skip the output stage on unpin */
	if (err || !is_pin)
		goto out;

	/* it is guaranteed that if nvmap_pin_ids returns 0 that
	 * all of the handle_ref objects are valid, so dereferencing
	 * directly here is safe */
	if (op.count > 1)
		output = (unsigned long __user *)op.addr;
	else {
		struct nvmap_pin_handle __user *tmp = arg;
		output = (unsigned long __user *)&(tmp->addr);
	}

	if (!output)
		goto out;

	for (i = 0; i < op.count && !err; i++) {
		unsigned long addr;

		h = (struct nvmap_handle *)refs[i];

		if (h->heap_pgalloc && h->pgalloc.contig)
			addr = page_to_phys(h->pgalloc.pages[0]);
		else if (h->heap_pgalloc)
			addr = h->pgalloc.area->iovm_start;
		else
			addr = h->carveout->base;

		err = put_user(addr, &output[i]);
	}

	if (err)
		nvmap_unpin_ids(filp->private_data, op.count, refs);

out:
	if (refs != on_stack)
		kfree(refs);

	return err;
}

int nvmap_ioctl_getid(struct file *filp, void __user *arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_create_handle op;
	struct nvmap_handle *h = NULL;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.handle)
		return -EINVAL;

	h = nvmap_get_handle_id(client, op.handle);

	if (!h)
		return -EPERM;

	op.id = (__u32)h;
	if (client == h->owner)
		h->global = true;

	nvmap_handle_put(h);

	return copy_to_user(arg, &op, sizeof(op)) ? -EFAULT : 0;
}

int nvmap_ioctl_alloc(struct file *filp, void __user *arg)
{
	struct nvmap_alloc_handle op;
	struct nvmap_client *client = filp->private_data;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.handle)
		return -EINVAL;

	if (op.align & (op.align - 1))
		return -EINVAL;

	/* user-space handles are aligned to page boundaries, to prevent
	 * data leakage. */
	op.align = max_t(size_t, op.align, PAGE_SIZE);
#if defined(CONFIG_NVMAP_FORCE_ZEROED_USER_PAGES)
	op.flags |= NVMAP_HANDLE_ZEROED_PAGES;
#endif

	return nvmap_alloc_handle_id(client, op.handle, op.heap_mask,
				     op.align, op.flags);
}

int nvmap_ioctl_create(struct file *filp, unsigned int cmd, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_handle_ref *ref = NULL;
	struct nvmap_client *client = filp->private_data;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!client)
		return -ENODEV;

	if (cmd == NVMAP_IOC_CREATE) {
		ref = nvmap_create_handle(client, PAGE_ALIGN(op.size));
		if (!IS_ERR(ref))
			ref->handle->orig_size = op.size;
	} else if (cmd == NVMAP_IOC_FROM_ID) {
		ref = nvmap_duplicate_handle_id(client, op.id);
	} else {
		return -EINVAL;
	}

	if (IS_ERR(ref))
		return PTR_ERR(ref);

	op.handle = nvmap_ref_to_id(ref);
	if (copy_to_user(arg, &op, sizeof(op))) {
		err = -EFAULT;
		nvmap_free_handle_id(client, op.handle);
	}

	return err;
}

int nvmap_map_into_caller_ptr(struct file *filp, void __user *arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_map_caller op;
	struct nvmap_vma_priv *vpriv;
	struct vm_area_struct *vma;
	struct nvmap_handle *h = NULL;
	unsigned int cache_flags;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.handle)
		return -EINVAL;

	h = nvmap_get_handle_id(client, op.handle);

	if (!h)
		return -EPERM;

	if(!h->alloc) {
		nvmap_handle_put(h);
		return -EFAULT;
	}

	trace_nvmap_map_into_caller_ptr(client, h, op.offset,
					op.length, op.flags);
	down_read(&current->mm->mmap_sem);

	vma = find_vma(current->mm, op.addr);
	if (!vma || !vma->vm_private_data) {
		err = -ENOMEM;
		goto out;
	}

	if (op.offset & ~PAGE_MASK) {
		err = -EFAULT;
		goto out;
	}

	if (op.offset > h->size || (op.offset + op.length) > h->size) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	vpriv = vma->vm_private_data;
	BUG_ON(!vpriv);

	/* the VMA must exactly match the requested mapping operation, and the
	 * VMA that is targetted must have been created by this driver
	 */
	if ((vma->vm_start != op.addr) || !is_nvmap_vma(vma) ||
	    (vma->vm_end-vma->vm_start != op.length)) {
		err = -EPERM;
		goto out;
	}

	/* verify that each mmap() system call creates a unique VMA */

	if (vpriv->handle && (h == vpriv->handle)) {
		goto out;
	} else if (vpriv->handle) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	nvmap_usecount_inc(h);

	if (!h->heap_pgalloc && (h->carveout->base & ~PAGE_MASK)) {
		nvmap_usecount_dec(h);
		err = -EFAULT;
		goto out;
	}

	vpriv->handle = h;
	vpriv->offs = op.offset;

	cache_flags = op.flags & NVMAP_HANDLE_CACHE_FLAG;
	if ((cache_flags == NVMAP_HANDLE_INNER_CACHEABLE ||
	     cache_flags == NVMAP_HANDLE_CACHEABLE) &&
	    (h->flags == NVMAP_HANDLE_UNCACHEABLE ||
	     h->flags == NVMAP_HANDLE_WRITE_COMBINE)) {
		if (h->size & ~PAGE_MASK) {
			pr_err("\n%s:attempt to convert a buffer from uc/wc to"
				" wb, whose size is not a multiple of page size."
				" request ignored.\n", __func__);
		} else {
			unsigned int nr_page = h->size >> PAGE_SHIFT;
			wmb();
			/* override allocation time cache coherency attributes. */
			h->flags &= ~NVMAP_HANDLE_CACHE_FLAG;
			h->flags |= cache_flags;

			/* Update page attributes, if the memory is allocated
			 *  from system heap pages.
			 */
			if (cache_flags == NVMAP_HANDLE_INNER_CACHEABLE &&
				h->heap_pgalloc)
				set_pages_array_iwb(h->pgalloc.pages, nr_page);
			else if (h->heap_pgalloc)
				set_pages_array_wb(h->pgalloc.pages, nr_page);
		}
	}
	vma->vm_page_prot = nvmap_pgprot(h, vma->vm_page_prot);

out:
	up_read(&current->mm->mmap_sem);

	if (err)
		nvmap_handle_put(h);
	return err;
}

int nvmap_ioctl_get_param(struct file *filp, void __user* arg)
{
	struct nvmap_handle_param op;
	struct nvmap_client *client = filp->private_data;
	struct nvmap_handle *h;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	h = nvmap_get_handle_id(client, op.handle);
	if (!h)
		return -EINVAL;

	switch (op.param) {
	case NVMAP_HANDLE_PARAM_SIZE:
		op.result = h->orig_size;
		break;
	case NVMAP_HANDLE_PARAM_ALIGNMENT:
		mutex_lock(&h->lock);
		if (!h->alloc)
			op.result = 0;
		else if (h->heap_pgalloc)
			op.result = PAGE_SIZE;
		else if (h->carveout->base)
			op.result = (h->carveout->base & -h->carveout->base);
		else
			op.result = SZ_4M;
		mutex_unlock(&h->lock);
		break;
	case NVMAP_HANDLE_PARAM_BASE:
		if (WARN_ON(!h->alloc || !atomic_add_return(0, &h->pin)))
			op.result = -1ul;
		else if (!h->heap_pgalloc) {
			mutex_lock(&h->lock);
			op.result = h->carveout->base;
			mutex_unlock(&h->lock);
		} else if (h->pgalloc.contig)
			op.result = page_to_phys(h->pgalloc.pages[0]);
		else if (h->pgalloc.area)
			op.result = h->pgalloc.area->iovm_start;
		else
			op.result = -1ul;
		break;
	case NVMAP_HANDLE_PARAM_HEAP:
		if (!h->alloc)
			op.result = 0;
		else if (!h->heap_pgalloc) {
			mutex_lock(&h->lock);
			op.result = nvmap_carveout_usage(client, h->carveout);
			mutex_unlock(&h->lock);
		} else if (h->pgalloc.contig)
			op.result = NVMAP_HEAP_SYSMEM;
		else
			op.result = NVMAP_HEAP_IOVMM;
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (!err && copy_to_user(arg, &op, sizeof(op)))
		err = -EFAULT;

	nvmap_handle_put(h);
	return err;
}

int nvmap_ioctl_rw_handle(struct file *filp, int is_read, void __user* arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_rw_handle __user *uarg = arg;
	struct nvmap_rw_handle op;
	struct nvmap_handle *h;
	ssize_t copied;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.handle || !op.addr || !op.count || !op.elem_size)
		return -EINVAL;

	h = nvmap_get_handle_id(client, op.handle);
	if (!h)
		return -EPERM;

	nvmap_usecount_inc(h);

	trace_nvmap_ioctl_rw_handle(client, h, is_read, op.offset,
				    op.addr, op.hmem_stride,
				    op.user_stride, op.elem_size, op.count);
	copied = rw_handle(client, h, is_read, op.offset,
			   (unsigned long)op.addr, op.hmem_stride,
			   op.user_stride, op.elem_size, op.count);

	if (copied < 0) {
		err = copied;
		copied = 0;
	} else if (copied < (op.count * op.elem_size))
		err = -EINTR;

	__put_user(copied, &uarg->count);

	nvmap_usecount_dec(h);

	nvmap_handle_put(h);

	return err;
}

int nvmap_ioctl_cache_maint(struct file *filp, void __user *arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_cache_op op;
	struct vm_area_struct *vma;
	struct nvmap_vma_priv *vpriv;
	unsigned long start;
	unsigned long end;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.handle || !op.addr || op.op < NVMAP_CACHE_OP_WB ||
	    op.op > NVMAP_CACHE_OP_WB_INV)
		return -EINVAL;

	down_read(&current->mm->mmap_sem);

	vma = find_vma(current->active_mm, (unsigned long)op.addr);
	if (!vma || !is_nvmap_vma(vma) ||
	    (unsigned long)op.addr + op.len > vma->vm_end) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	vpriv = (struct nvmap_vma_priv *)vma->vm_private_data;

	if ((unsigned long)vpriv->handle != op.handle) {
		err = -EFAULT;
		goto out;
	}

	start = (unsigned long)op.addr - vma->vm_start;
	end = start + op.len;

	err = cache_maint(client, vpriv->handle, start, end, op.op);
out:
	up_read(&current->mm->mmap_sem);
	return err;
}

int nvmap_ioctl_free(struct file *filp, unsigned long arg)
{
	struct nvmap_client *client = filp->private_data;

	if (!arg)
		return 0;

	nvmap_free_handle_id(client, arg);
	return 0;
}

static void inner_cache_maint(unsigned int op, void *vaddr, size_t size)
{
	if (op == NVMAP_CACHE_OP_WB_INV)
		dmac_flush_range(vaddr, vaddr + size);
	else if (op == NVMAP_CACHE_OP_INV)
		dmac_map_area(vaddr, size, DMA_FROM_DEVICE);
	else
		dmac_map_area(vaddr, size, DMA_TO_DEVICE);
}

static void outer_cache_maint(unsigned int op, unsigned long paddr, size_t size)
{
	if (op == NVMAP_CACHE_OP_WB_INV)
		outer_flush_range(paddr, paddr + size);
	else if (op == NVMAP_CACHE_OP_INV)
		outer_inv_range(paddr, paddr + size);
	else
		outer_clean_range(paddr, paddr + size);
}

static void heap_page_cache_maint(struct nvmap_client *client,
	struct nvmap_handle *h, unsigned long start, unsigned long end,
	unsigned int op, bool inner, bool outer, pte_t **pte,
	unsigned long kaddr, pgprot_t prot)
{
	struct page *page;
	unsigned long paddr;
	unsigned long next;
	unsigned long off;
	size_t size;

	while (start < end) {
		page = h->pgalloc.pages[start >> PAGE_SHIFT];
		next = min(((start + PAGE_SIZE) & PAGE_MASK), end);
		off = start & ~PAGE_MASK;
		size = next - start;
		paddr = page_to_phys(page) + off;

		if (inner) {
			void *vaddr = (void *)kaddr + off;
			BUG_ON(!pte);
			BUG_ON(!kaddr);
			set_pte_at(&init_mm, kaddr, *pte,
				pfn_pte(__phys_to_pfn(paddr), prot));
			flush_tlb_kernel_page(kaddr);
			inner_cache_maint(op, vaddr, size);
		}

		if (outer)
			outer_cache_maint(op, paddr, size);
		start = next;
	}
}

#if defined(CONFIG_NVMAP_OUTER_CACHE_MAINT_BY_SET_WAYS)
static bool fast_cache_maint_outer(unsigned long start,
		unsigned long end, unsigned int op)
{
	bool result = false;

	if (end - start >= FLUSH_CLEAN_BY_SET_WAY_THRESHOLD_OUTER) {
		if (op == NVMAP_CACHE_OP_WB_INV) {
			outer_flush_all();
			result = true;
		}
		if (op == NVMAP_CACHE_OP_WB) {
			outer_clean_all();
			result = true;
		}
	}

	return result;
}
#endif

static bool fast_cache_maint(struct nvmap_client *client, struct nvmap_handle *h,
	unsigned long start, unsigned long end, unsigned int op)
{
	int ret = false;
#if defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS)
	if ((op == NVMAP_CACHE_OP_INV) ||
		((end - start) < FLUSH_CLEAN_BY_SET_WAY_THRESHOLD_INNER))
		goto out;

	if (op == NVMAP_CACHE_OP_WB_INV)
		inner_flush_cache_all();
	else if (op == NVMAP_CACHE_OP_WB)
		inner_clean_cache_all();

	/* outer maintenance */
	if (h->flags != NVMAP_HANDLE_INNER_CACHEABLE ) {
		if(!fast_cache_maint_outer(start, end, op))
		{
			if (h->heap_pgalloc) {
				heap_page_cache_maint(client, h, start,
					end, op, false, true, NULL, 0, 0);
			} else  {
				start += h->carveout->base;
				end += h->carveout->base;
				outer_cache_maint(op, start, end - start);
			}
		}
	}
	ret = true;
out:
#endif
	return ret;
}

static int cache_maint(struct nvmap_client *client, struct nvmap_handle *h,
		       unsigned long start, unsigned long end, unsigned int op)
{
	pgprot_t prot;
	pte_t **pte = NULL;
	unsigned long kaddr;
	unsigned long loop;
	int err = 0;

	h = nvmap_handle_get(h);
	if (!h)
		return -EFAULT;

	if (!h->alloc) {
		err = -EFAULT;
		goto out;
	}

	trace_cache_maint(client, h, start, end, op);
	wmb();
	if (h->flags == NVMAP_HANDLE_UNCACHEABLE ||
	    h->flags == NVMAP_HANDLE_WRITE_COMBINE || start == end)
		goto out;

	if (fast_cache_maint(client, h, start, end, op))
		goto out;

	prot = nvmap_pgprot(h, pgprot_kernel);
	pte = nvmap_alloc_pte(client->dev, (void **)&kaddr);
	if (IS_ERR(pte)) {
		err = PTR_ERR(pte);
		pte = NULL;
		goto out;
	}

	if (h->heap_pgalloc) {
		heap_page_cache_maint(client, h, start, end, op, true,
			(h->flags == NVMAP_HANDLE_INNER_CACHEABLE) ? false : true,
			pte, kaddr, prot);
		goto out;
	}

	if (start > h->size || end > h->size) {
		nvmap_warn(client, "cache maintenance outside handle\n");
		return -EINVAL;
	}

	/* lock carveout from relocation by mapcount */
	nvmap_usecount_inc(h);

	start += h->carveout->base;
	end += h->carveout->base;

	loop = start;

	while (loop < end) {
		unsigned long next = (loop + PAGE_SIZE) & PAGE_MASK;
		void *base = (void *)kaddr + (loop & ~PAGE_MASK);
		next = min(next, end);

		set_pte_at(&init_mm, kaddr, *pte,
			   pfn_pte(__phys_to_pfn(loop), prot));
		flush_tlb_kernel_page(kaddr);

		inner_cache_maint(op, base, next - loop);
		loop = next;
	}

	if (h->flags != NVMAP_HANDLE_INNER_CACHEABLE)
		outer_cache_maint(op, start, end - start);

	/* unlock carveout */
	nvmap_usecount_dec(h);

out:
	if (pte)
		nvmap_free_pte(client->dev, pte);
	nvmap_handle_put(h);
	return err;
}

static int rw_handle_page(struct nvmap_handle *h, int is_read,
			  phys_addr_t start, unsigned long rw_addr,
			  unsigned long bytes, unsigned long kaddr, pte_t *pte)
{
	pgprot_t prot = nvmap_pgprot(h, pgprot_kernel);
	unsigned long end = start + bytes;
	int err = 0;

	while (!err && start < end) {
		struct page *page = NULL;
		phys_addr_t phys;
		size_t count;
		void *src;

		if (!h->heap_pgalloc) {
			phys = h->carveout->base + start;
		} else {
			page = h->pgalloc.pages[start >> PAGE_SHIFT];
			BUG_ON(!page);
			get_page(page);
			phys = page_to_phys(page) + (start & ~PAGE_MASK);
		}

		set_pte_at(&init_mm, kaddr, pte,
			   pfn_pte(__phys_to_pfn(phys), prot));
		flush_tlb_kernel_page(kaddr);

		src = (void *)kaddr + (phys & ~PAGE_MASK);
		phys = PAGE_SIZE - (phys & ~PAGE_MASK);
		count = min_t(size_t, end - start, phys);

		if (is_read)
			err = copy_to_user((void *)rw_addr, src, count);
		else
			err = copy_from_user(src, (void *)rw_addr, count);

		if (err)
			err = -EFAULT;

		rw_addr += count;
		start += count;

		if (page)
			put_page(page);
	}

	return err;
}

static ssize_t rw_handle(struct nvmap_client *client, struct nvmap_handle *h,
			 int is_read, unsigned long h_offs,
			 unsigned long sys_addr, unsigned long h_stride,
			 unsigned long sys_stride, unsigned long elem_size,
			 unsigned long count)
{
	ssize_t copied = 0;
	pte_t **pte;
	void *addr;
	int ret = 0;

	if (!elem_size)
		return -EINVAL;

	if (!h->alloc)
		return -EFAULT;

	if (elem_size == h_stride && elem_size == sys_stride) {
		elem_size *= count;
		h_stride = elem_size;
		sys_stride = elem_size;
		count = 1;
	}

	pte = nvmap_alloc_pte(client->dev, &addr);
	if (IS_ERR(pte))
		return PTR_ERR(pte);

	while (count--) {
		if (h_offs + elem_size > h->size) {
			nvmap_warn(client, "read/write outside of handle\n");
			ret = -EFAULT;
			break;
		}
		if (is_read)
			cache_maint(client, h, h_offs,
				h_offs + elem_size, NVMAP_CACHE_OP_INV);

		ret = rw_handle_page(h, is_read, h_offs, sys_addr,
				     elem_size, (unsigned long)addr, *pte);

		if (ret)
			break;

		if (!is_read)
			cache_maint(client, h, h_offs,
				h_offs + elem_size, NVMAP_CACHE_OP_WB);

		copied += elem_size;
		sys_addr += sys_stride;
		h_offs += h_stride;
	}

	nvmap_free_pte(client->dev, pte);
	return ret ?: copied;
}
