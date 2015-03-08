/*
 * drivers/video/tegra/host/gr3d/gr3d_t30.c
 *
 * Tegra Graphics Host 3D for Tegra3
 *
 * Copyright (c) 2011-2012 NVIDIA Corporation.
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

#include "nvhost_hwctx.h"
#include "nvhost_channel.h"
#include "nvhost_cdma.h"
#include "dev.h"
#include "host1x/host1x01_hardware.h"
#include "gr3d.h"
#include "chip_support.h"
#include "nvhost_memmgr.h"

#include <mach/gpufuse.h>
#include <mach/hardware.h>
#include <linux/slab.h>

static const struct hwctx_reginfo ctxsave_regs_3d_global[] = {
	HWCTX_REGINFO(0xe00,    4, DIRECT),
	HWCTX_REGINFO(0xe05,   30, DIRECT),
	HWCTX_REGINFO(0xe25,    2, DIRECT),
	HWCTX_REGINFO(0xe28,    2, DIRECT),
	HWCTX_REGINFO(0xe30,   16, DIRECT),
	HWCTX_REGINFO(0x001,    2, DIRECT),
	HWCTX_REGINFO(0x00c,   10, DIRECT),
	HWCTX_REGINFO(0x100,   34, DIRECT),
	HWCTX_REGINFO(0x124,    2, DIRECT),
	HWCTX_REGINFO(0x200,    5, DIRECT),
	HWCTX_REGINFO(0x205, 1024, INDIRECT),
	HWCTX_REGINFO(0x207, 1024, INDIRECT),
	HWCTX_REGINFO(0x209,    1, DIRECT),
	HWCTX_REGINFO(0x300,   64, DIRECT),
	HWCTX_REGINFO(0x343,   25, DIRECT),
	HWCTX_REGINFO(0x363,    2, DIRECT),
	HWCTX_REGINFO(0x400,   16, DIRECT),
	HWCTX_REGINFO(0x411,    1, DIRECT),
	HWCTX_REGINFO(0x412,    1, DIRECT),
	HWCTX_REGINFO(0x500,    4, DIRECT),
	HWCTX_REGINFO(0x520,   32, DIRECT),
	HWCTX_REGINFO(0x540,   64, INDIRECT),
	HWCTX_REGINFO(0x600,   16, INDIRECT_4X),
	HWCTX_REGINFO(0x603,  128, INDIRECT),
	HWCTX_REGINFO(0x608,    4, DIRECT),
	HWCTX_REGINFO(0x60e,    1, DIRECT),
	HWCTX_REGINFO(0x700,   64, INDIRECT),
	HWCTX_REGINFO(0x710,   50, DIRECT),
	HWCTX_REGINFO(0x750,   16, DIRECT),
	HWCTX_REGINFO(0x800,   16, INDIRECT_4X),
	HWCTX_REGINFO(0x803,  512, INDIRECT),
	HWCTX_REGINFO(0x805,   64, INDIRECT),
	HWCTX_REGINFO(0x820,   32, DIRECT),
	HWCTX_REGINFO(0x900,   64, INDIRECT),
	HWCTX_REGINFO(0x902,    2, DIRECT),
	HWCTX_REGINFO(0x90a,    1, DIRECT),
	HWCTX_REGINFO(0xa02,   10, DIRECT),
	HWCTX_REGINFO(0xb04,    1, DIRECT),
	HWCTX_REGINFO(0xb06,   13, DIRECT),
};

static const struct hwctx_reginfo ctxsave_regs_3d_perset[] = {
	HWCTX_REGINFO(0xe04,    1, DIRECT),
	HWCTX_REGINFO(0xe2a,    1, DIRECT),
	HWCTX_REGINFO(0x413,    1, DIRECT),
	HWCTX_REGINFO(0x90b,    1, DIRECT),
	HWCTX_REGINFO(0xe41,    1, DIRECT),
};

static unsigned int restore_set1_offset;

#define SAVE_BEGIN_V1_SIZE (1 + RESTORE_BEGIN_SIZE)
#define SAVE_DIRECT_V1_SIZE (4 + RESTORE_DIRECT_SIZE)
#define SAVE_INDIRECT_V1_SIZE (6 + RESTORE_INDIRECT_SIZE)
#define SAVE_END_V1_SIZE (9 + RESTORE_END_SIZE)
#define SAVE_INCRS 3
#define SAVE_THRESH_OFFSET 0
#define RESTORE_BEGIN_SIZE 4
#define RESTORE_DIRECT_SIZE 1
#define RESTORE_INDIRECT_SIZE 2
#define RESTORE_END_SIZE 1

struct save_info {
	u32 *ptr;
	unsigned int save_count;
	unsigned int restore_count;
	unsigned int save_incrs;
	unsigned int restore_incrs;
};

/*** v1 saver ***/

static void save_push_v1(struct nvhost_hwctx *nctx, struct nvhost_cdma *cdma)
{
	struct host1x_hwctx *ctx = to_host1x_hwctx(nctx);
	struct host1x_hwctx_handler *p = host1x_hwctx_handler(ctx);

	/* wait for 3d idle */
	nvhost_cdma_push(cdma,
			nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
			nvhost_opcode_imm_incr_syncpt(
				host1x_uclass_incr_syncpt_cond_op_done_v(),
				p->syncpt));
	nvhost_cdma_push(cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					host1x_uclass_wait_syncpt_base_r(), 1),
			nvhost_class_host_wait_syncpt_base(p->syncpt,
							p->waitbase, 1));
	/* back to 3d */
	nvhost_cdma_push(cdma,
			nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
			NVHOST_OPCODE_NOOP);

	/* invalidate the FDC to prevent cache-coherency issues across GPUs
	   note that we assume FDC_CONTROL_0 is left in the reset state by all
	   contexts.  the invalidate bit will clear itself, so the register
	   should be unchanged after this */
	nvhost_cdma_push(cdma,
		nvhost_opcode_imm(AR3D_FDC_CONTROL_0,
			AR3D_FDC_CONTROL_0_RESET_VAL
				| AR3D_FDC_CONTROL_0_INVALIDATE),
		NVHOST_OPCODE_NOOP);

	/* set register set 0 and 1 register read memory output addresses,
	   and send their reads to memory */

	nvhost_cdma_push(cdma,
		nvhost_opcode_imm(AR3D_GSHIM_WRITE_MASK, 2),
		nvhost_opcode_imm(AR3D_GLOBAL_MEMORY_OUTPUT_READS, 1));
	nvhost_cdma_push(cdma,
		nvhost_opcode_nonincr(AR3D_DW_MEMORY_OUTPUT_ADDRESS, 1),
		ctx->restore_phys + restore_set1_offset * 4);

	nvhost_cdma_push(cdma,
		nvhost_opcode_imm(AR3D_GSHIM_WRITE_MASK, 1),
		nvhost_opcode_imm(AR3D_GLOBAL_MEMORY_OUTPUT_READS, 1));
	nvhost_cdma_push(cdma,
		nvhost_opcode_nonincr(AR3D_DW_MEMORY_OUTPUT_ADDRESS, 1),
		ctx->restore_phys);
	/* gather the save buffer */
	nvhost_cdma_push_gather(cdma,
			nvhost_get_host(nctx->channel->dev)->memmgr,
			p->save_buf,
			0,
			nvhost_opcode_gather(p->save_size),
			p->save_phys);
}

static void save_begin_v1(struct host1x_hwctx_handler *p, u32 *ptr)
{
	ptr[0] = nvhost_opcode_nonincr(AR3D_DW_MEMORY_OUTPUT_DATA,
			RESTORE_BEGIN_SIZE);
	nvhost_3dctx_restore_begin(p, ptr + 1);
	ptr += RESTORE_BEGIN_SIZE;
}

static void save_direct_v1(u32 *ptr, u32 start_reg, u32 count)
{
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID,
			AR3D_DW_MEMORY_OUTPUT_DATA, 1);
	nvhost_3dctx_restore_direct(ptr + 1, start_reg, count);
	ptr += RESTORE_DIRECT_SIZE;
	ptr[1] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					host1x_uclass_indoff_r(), 1);
	ptr[2] = nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_GR3D,
						start_reg, true);
	/* TODO could do this in the setclass if count < 6 */
	ptr[3] = nvhost_opcode_nonincr(host1x_uclass_inddata_r(), count);
}

static void save_indirect_v1(u32 *ptr, u32 offset_reg, u32 offset,
			u32 data_reg, u32 count)
{
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
	ptr[1] = nvhost_opcode_nonincr(AR3D_DW_MEMORY_OUTPUT_DATA,
			RESTORE_INDIRECT_SIZE);
	nvhost_3dctx_restore_indirect(ptr + 2, offset_reg, offset, data_reg,
			count);
	ptr += RESTORE_INDIRECT_SIZE;
	ptr[2] = nvhost_opcode_imm(offset_reg, offset);
	ptr[3] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					host1x_uclass_indoff_r(), 1);
	ptr[4] = nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_GR3D,
						data_reg, false);
	ptr[5] = nvhost_opcode_nonincr(host1x_uclass_inddata_r(), count);
}

static void save_end_v1(struct host1x_hwctx_handler *p, u32 *ptr)
{
	/* write end of restore buffer */
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID,
			AR3D_DW_MEMORY_OUTPUT_DATA, 1);
	nvhost_3dctx_restore_end(p, ptr + 1);
	ptr += RESTORE_END_SIZE;
	/* reset to dual reg if necessary */
	ptr[1] = nvhost_opcode_imm(AR3D_GSHIM_WRITE_MASK,
			(1 << 2) - 1);
	/* op_done syncpt incr to flush FDC */
	ptr[2] = nvhost_opcode_imm_incr_syncpt(
			host1x_uclass_incr_syncpt_cond_op_done_v(), p->syncpt);
	/* host wait for that syncpt incr, and advance the wait base */
	ptr[3] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
			host1x_uclass_wait_syncpt_base_r(),
			nvhost_mask2(
				host1x_uclass_wait_syncpt_base_r(),
				host1x_uclass_incr_syncpt_base_r()));
	ptr[4] = nvhost_class_host_wait_syncpt_base(p->syncpt,
				p->waitbase, p->save_incrs - 1);
	ptr[5] = nvhost_class_host_incr_syncpt_base(p->waitbase,
			p->save_incrs);
	/* set class back to 3d */
	ptr[6] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
	/* send reg reads back to host */
	ptr[7] = nvhost_opcode_imm(AR3D_GLOBAL_MEMORY_OUTPUT_READS, 0);
	/* final syncpt increment to release waiters */
	ptr[8] = nvhost_opcode_imm(0, p->syncpt);
}

/*** save ***/



static void setup_save_regs(struct save_info *info,
			const struct hwctx_reginfo *regs,
			unsigned int nr_regs)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;
	u32 *ptr = info->ptr;
	unsigned int save_count = info->save_count;
	unsigned int restore_count = info->restore_count;

	for ( ; regs != rend; ++regs) {
		u32 offset = regs->offset;
		u32 count = regs->count;
		u32 indoff = offset + 1;
		switch (regs->type) {
		case HWCTX_REGINFO_DIRECT:
			if (ptr) {
				save_direct_v1(ptr, offset, count);
				ptr += SAVE_DIRECT_V1_SIZE;
			}
			save_count += SAVE_DIRECT_V1_SIZE;
			restore_count += RESTORE_DIRECT_SIZE;
			break;
		case HWCTX_REGINFO_INDIRECT_4X:
			++indoff;
			/* fall through */
		case HWCTX_REGINFO_INDIRECT:
			if (ptr) {
				save_indirect_v1(ptr, offset, 0,
						indoff, count);
				ptr += SAVE_INDIRECT_V1_SIZE;
			}
			save_count += SAVE_INDIRECT_V1_SIZE;
			restore_count += RESTORE_INDIRECT_SIZE;
			break;
		}
		if (ptr) {
			/* SAVE cases only: reserve room for incoming data */
			u32 k = 0;
			/*
			 * Create a signature pattern for indirect data (which
			 * will be overwritten by true incoming data) for
			 * better deducing where we are in a long command
			 * sequence, when given only a FIFO snapshot for debug
			 * purposes.
			*/
			for (k = 0; k < count; k++)
				*(ptr + k) = 0xd000d000 | (offset << 16) | k;
			ptr += count;
		}
		save_count += count;
		restore_count += count;
	}

	info->ptr = ptr;
	info->save_count = save_count;
	info->restore_count = restore_count;
}

static void switch_gpu(struct save_info *info,
			unsigned int save_src_set,
			u32 save_dest_sets,
			u32 restore_dest_sets)
{
	if (info->ptr) {
		info->ptr[0] = nvhost_opcode_setclass(
				NV_GRAPHICS_3D_CLASS_ID,
				AR3D_DW_MEMORY_OUTPUT_DATA, 1);
		info->ptr[1] = nvhost_opcode_imm(AR3D_GSHIM_WRITE_MASK,
				restore_dest_sets);
		info->ptr[2] = nvhost_opcode_imm(AR3D_GSHIM_WRITE_MASK,
				save_dest_sets);
		info->ptr[3] = nvhost_opcode_imm(AR3D_GSHIM_READ_SELECT,
				save_src_set);
		info->ptr += 4;
	}
	info->save_count += 4;
	info->restore_count += 1;
}

static void setup_save(struct host1x_hwctx_handler *p, u32 *ptr)
{
	struct save_info info = {
		ptr,
		SAVE_BEGIN_V1_SIZE,
		RESTORE_BEGIN_SIZE,
		SAVE_INCRS,
		1
	};
	int save_end_size = SAVE_END_V1_SIZE;

	if (info.ptr) {
		save_begin_v1(p, info.ptr);
		info.ptr += SAVE_BEGIN_V1_SIZE;
	}

	/* read from set0, write cmds through set0, restore to set0 and 1 */
	switch_gpu(&info, 0, 1, 3);

	/* save regs that are common to both sets */
	setup_save_regs(&info,
			ctxsave_regs_3d_global,
			ARRAY_SIZE(ctxsave_regs_3d_global));

	/* read from set 0, write cmds through set0, restore to set0 */
	switch_gpu(&info, 0, 1, 1);

	/* save set 0 specific regs */
	setup_save_regs(&info,
			ctxsave_regs_3d_perset,
			ARRAY_SIZE(ctxsave_regs_3d_perset));


	/* read from set1, write cmds through set1, restore to set1 */
	switch_gpu(&info, 1, 2, 2);
	/* note offset at which set 1 restore starts */
	restore_set1_offset = info.restore_count;
	/* save set 1 specific regs */
	setup_save_regs(&info,
			ctxsave_regs_3d_perset,
			ARRAY_SIZE(ctxsave_regs_3d_perset));


	/* read from set0, write cmds through set1, restore to set0 and 1 */
	switch_gpu(&info, 0, 2, 3);

	if (info.ptr) {
		save_end_v1(p, info.ptr);
		info.ptr += SAVE_END_V1_SIZE;
	}

	wmb();

	p->save_size = info.save_count + save_end_size;
	p->restore_size = info.restore_count + RESTORE_END_SIZE;
	p->save_incrs = info.save_incrs;
	p->save_thresh = p->save_incrs - SAVE_THRESH_OFFSET;
	p->restore_incrs = info.restore_incrs;
}


/*** ctx3d ***/

static struct nvhost_hwctx *ctx3d_alloc_v1(struct nvhost_hwctx_handler *h,
		struct nvhost_channel *ch)
{
	struct host1x_hwctx_handler *p = to_host1x_hwctx_handler(h);
	struct host1x_hwctx *ctx = nvhost_3dctx_alloc_common(p, ch, false);

	if (ctx)
		return &ctx->hwctx;
	else
		return NULL;
}

struct nvhost_hwctx_handler *nvhost_gr3d_t30_ctxhandler_init(
		u32 syncpt, u32 waitbase,
		struct nvhost_channel *ch)
{
	struct mem_mgr *memmgr;
	u32 *save_ptr;
	struct host1x_hwctx_handler *p;

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	memmgr = nvhost_get_host(ch->dev)->memmgr;

	p->syncpt = syncpt;
	p->waitbase = waitbase;

	setup_save(p, NULL);

	p->save_buf = mem_op().alloc(memmgr, p->save_size * 4, 32,
				mem_mgr_flag_write_combine);
	if (IS_ERR_OR_NULL(p->save_buf)) {
		p->save_buf = NULL;
		return NULL;
	}

	p->save_slots = 8;

	save_ptr = mem_op().mmap(p->save_buf);
	if (!save_ptr) {
		mem_op().put(memmgr, p->save_buf);
		p->save_buf = NULL;
		return NULL;
	}

	p->save_phys = mem_op().pin(memmgr, p->save_buf);

	setup_save(p, save_ptr);

	mem_op().munmap(p->save_buf, save_ptr);

	p->h.alloc = ctx3d_alloc_v1;
	p->h.save_push = save_push_v1;
	p->h.save_service = NULL;
	p->h.get = nvhost_3dctx_get;
	p->h.put = nvhost_3dctx_put;

	return &p->h;
}
