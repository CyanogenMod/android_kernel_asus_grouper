/*
 * drivers/video/tegra/dc/window.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Copyright (c) 2010-2012, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/err.h>
#include <linux/types.h>
#include <mach/dc.h>

#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"

static int no_vsync;
static atomic_t frame_end_ref = ATOMIC_INIT(0);

module_param_named(no_vsync, no_vsync, int, S_IRUGO | S_IWUSR);

static bool tegra_dc_windows_are_clean(struct tegra_dc_win *windows[],
					     int n)
{
	int i;

	for (i = 0; i < n; i++) {
		if (windows[i]->dirty)
			return false;
	}

	return true;
}

int tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable)
{
	tegra_dc_writel(dc, FRAME_END_INT, DC_CMD_INT_STATUS);
	if (enable) {
		atomic_inc(&frame_end_ref);
		tegra_dc_unmask_interrupt(dc, FRAME_END_INT);
	} else if (!atomic_dec_return(&frame_end_ref))
		tegra_dc_mask_interrupt(dc, FRAME_END_INT);
	return 0;
}

static int get_topmost_window(u32 *depths, unsigned long *wins)
{
	int idx, best = -1;

	for_each_set_bit(idx, wins, DC_N_WINDOWS) {
		if (best == -1 || depths[idx] < depths[best])
			best = idx;
	}
	clear_bit(best, wins);
	return best;
}

static u32 blend_topwin(u32 flags)
{
	if (flags & TEGRA_WIN_FLAG_BLEND_COVERAGE)
		return BLEND(NOKEY, ALPHA, 0xff, 0xff);
	else if (flags & TEGRA_WIN_FLAG_BLEND_PREMULT)
		return BLEND(NOKEY, PREMULT, 0xff, 0xff);
	else
		return BLEND(NOKEY, FIX, 0xff, 0xff);
}

static u32 blend_2win(int idx, unsigned long behind_mask, u32* flags, int xy)
{
	int other;

	for (other = 0; other < DC_N_WINDOWS; other++) {
		if (other != idx && (xy-- == 0))
			break;
	}
	if (BIT(other) & behind_mask)
		return blend_topwin(flags[idx]);
	else if (flags[other])
		return BLEND(NOKEY, DEPENDANT, 0x00, 0x00);
	else
		return BLEND(NOKEY, FIX, 0x00, 0x00);
}

static u32 blend_3win(int idx, unsigned long behind_mask, u32* flags)
{
	unsigned long infront_mask;
	int first;

	infront_mask = ~(behind_mask | BIT(idx));
	infront_mask &= (BIT(DC_N_WINDOWS) - 1);
	first = ffs(infront_mask) - 1;

	if (!infront_mask)
		return blend_topwin(flags[idx]);
	else if (behind_mask && first != -1 && flags[first])
		return BLEND(NOKEY, DEPENDANT, 0x00, 0x00);
	else
		return BLEND(NOKEY, FIX, 0x0, 0x0);
}

static void tegra_dc_set_blending(struct tegra_dc *dc,
	struct tegra_dc_blend *blend)
{
	unsigned long mask = BIT(DC_N_WINDOWS) - 1;

	while (mask) {
		int idx = get_topmost_window(blend->z, &mask);

		tegra_dc_writel(dc, WINDOW_A_SELECT << idx,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		tegra_dc_writel(dc, BLEND(NOKEY, FIX, 0xff, 0xff),
				DC_WIN_BLEND_NOKEY);
		tegra_dc_writel(dc, BLEND(NOKEY, FIX, 0xff, 0xff),
				DC_WIN_BLEND_1WIN);
		tegra_dc_writel(dc, blend_2win(idx, mask, blend->flags, 0),
				DC_WIN_BLEND_2WIN_X);
		tegra_dc_writel(dc, blend_2win(idx, mask, blend->flags, 1),
				DC_WIN_BLEND_2WIN_Y);
		tegra_dc_writel(dc, blend_3win(idx, mask, blend->flags),
				DC_WIN_BLEND_3WIN_XY);
	}
}

/* does not support syncing windows on multiple dcs in one call */
int tegra_dc_sync_windows(struct tegra_dc_win *windows[], int n)
{
	int ret;
	if (n < 1 || n > DC_N_WINDOWS)
		return -EINVAL;

	if (!windows[0]->dc->enabled)
		return -EFAULT;

#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	/* Don't want to timeout on simulator */
	ret = wait_event_interruptible(windows[0]->dc->wq,
		tegra_dc_windows_are_clean(windows, n));
#else
	trace_printk("%s:Before wait_event_interruptible_timeout\n",
		windows[0]->dc->ndev->name);
	ret = wait_event_interruptible_timeout(windows[0]->dc->wq,
		tegra_dc_windows_are_clean(windows, n),
		HZ);
	trace_printk("%s:After wait_event_interruptible_timeout\n",
		windows[0]->dc->ndev->name);
#endif
	return ret;
}
EXPORT_SYMBOL(tegra_dc_sync_windows);

static inline u32 compute_dda_inc(fixed20_12 in, unsigned out_int,
				  bool v, unsigned Bpp)
{
	/*
	 * min(round((prescaled_size_in_pixels - 1) * 0x1000 /
	 *	     (post_scaled_size_in_pixels - 1)), MAX)
	 * Where the value of MAX is as follows:
	 * For V_DDA_INCREMENT: 15.0 (0xF000)
	 * For H_DDA_INCREMENT:  4.0 (0x4000) for 4 Bytes/pix formats.
	 *			 8.0 (0x8000) for 2 Bytes/pix formats.
	 */

	fixed20_12 out = dfixed_init(out_int);
	u32 dda_inc;
	int max;

	if (v) {
		max = 15;
	} else {
		switch (Bpp) {
		default:
			WARN_ON_ONCE(1);
			/* fallthrough */
		case 4:
			max = 4;
			break;
		case 2:
			max = 8;
			break;
		}
	}

	out.full = max_t(u32, out.full - dfixed_const(1), dfixed_const(1));
	in.full -= dfixed_const(1);

	dda_inc = dfixed_div(in, out);

	dda_inc = min_t(u32, dda_inc, dfixed_const(max));

	return dda_inc;
}

static inline u32 compute_initial_dda(fixed20_12 in)
{
	return dfixed_frac(in);
}

/* does not support updating windows on multiple dcs in one call */
int tegra_dc_update_windows(struct tegra_dc_win *windows[], int n)
{
	struct tegra_dc *dc;
	unsigned long update_mask = GENERAL_ACT_REQ;
	unsigned long val;
	bool update_blend = false;
	int i;

	dc = windows[0]->dc;

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		/* Acquire one_shot_lock to avoid race condition between
		 * cancellation of old delayed work and schedule of new
		 * delayed work. */
		mutex_lock(&dc->one_shot_lock);
		cancel_delayed_work_sync(&dc->one_shot_work);
	}
	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
			mutex_unlock(&dc->one_shot_lock);
		return -EFAULT;
	}

	tegra_dc_hold_dc_out(dc);

	if (no_vsync)
		tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
			DC_CMD_STATE_ACCESS);
	else
		tegra_dc_writel(dc, WRITE_MUX_ASSEMBLY | READ_MUX_ASSEMBLY,
			DC_CMD_STATE_ACCESS);

	for (i = 0; i < n; i++) {
		struct tegra_dc_win *win = windows[i];
		unsigned h_dda;
		unsigned v_dda;
		fixed20_12 h_offset, v_offset;
		bool invert_h = (win->flags & TEGRA_WIN_FLAG_INVERT_H) != 0;
		bool invert_v = (win->flags & TEGRA_WIN_FLAG_INVERT_V) != 0;
		bool yuv = tegra_dc_is_yuv(win->fmt);
		bool yuvp = tegra_dc_is_yuv_planar(win->fmt);
		unsigned Bpp = tegra_dc_fmt_bpp(win->fmt) / 8;
		/* Bytes per pixel of bandwidth, used for dda_inc calculation */
		unsigned Bpp_bw = Bpp * (yuvp ? 2 : 1);
		const bool filter_h = win_use_h_filter(dc, win);
		const bool filter_v = win_use_v_filter(dc, win);

#if 1
		if (win->dc->ndev->id == 0) {
			invert_h = !invert_h;
			invert_v = !invert_v;
			win->out_x = win->dc->pdata->fb->xres - (win->out_x + win->out_w);
			win->out_y = win->dc->pdata->fb->yres - (win->out_y + win->out_h);
		}
#endif

		if (win->z != dc->blend.z[win->idx]) {
			dc->blend.z[win->idx] = win->z;
			update_blend = true;
		}
		if ((win->flags & TEGRA_WIN_BLEND_FLAGS_MASK) !=
			dc->blend.flags[win->idx]) {
			dc->blend.flags[win->idx] =
				win->flags & TEGRA_WIN_BLEND_FLAGS_MASK;
			update_blend = true;
		}

		tegra_dc_writel(dc, WINDOW_A_SELECT << win->idx,
				DC_CMD_DISPLAY_WINDOW_HEADER);

		if (!no_vsync)
			update_mask |= WIN_A_ACT_REQ << win->idx;

		if (!WIN_IS_ENABLED(win)) {
			dc->windows[i].dirty = 1;
			tegra_dc_writel(dc, TEGRA_WIN_FLAG_INVERT_H|TEGRA_WIN_FLAG_INVERT_V, DC_WIN_WIN_OPTIONS);
			continue;
		}

		tegra_dc_writel(dc, win->fmt & 0x1f, DC_WIN_COLOR_DEPTH);
		tegra_dc_writel(dc, win->fmt >> 6, DC_WIN_BYTE_SWAP);

		tegra_dc_writel(dc,
			V_POSITION(win->out_y) | H_POSITION(win->out_x),
			DC_WIN_POSITION);
		tegra_dc_writel(dc,
			V_SIZE(win->out_h) | H_SIZE(win->out_w),
			DC_WIN_SIZE);

		if (tegra_dc_feature_has_scaling(dc, win->idx)) {
			tegra_dc_writel(dc,
				V_PRESCALED_SIZE(dfixed_trunc(win->h)) |
				H_PRESCALED_SIZE(dfixed_trunc(win->w) * Bpp),
				DC_WIN_PRESCALED_SIZE);

			h_dda = compute_dda_inc(win->w, win->out_w, false,
				Bpp_bw);
			v_dda = compute_dda_inc(win->h, win->out_h, true,
				Bpp_bw);
			tegra_dc_writel(dc, V_DDA_INC(v_dda) |
				H_DDA_INC(h_dda), DC_WIN_DDA_INCREMENT);
			h_dda = compute_initial_dda(win->x);
			v_dda = compute_initial_dda(win->y);
			tegra_dc_writel(dc, h_dda, DC_WIN_H_INITIAL_DDA);
			tegra_dc_writel(dc, v_dda, DC_WIN_V_INITIAL_DDA);
		}

		tegra_dc_writel(dc, 0, DC_WIN_BUF_STRIDE);
		tegra_dc_writel(dc, 0, DC_WIN_UV_BUF_STRIDE);
		tegra_dc_writel(dc, (unsigned long)win->phys_addr,
			DC_WINBUF_START_ADDR);

		if (!yuvp) {
			tegra_dc_writel(dc, win->stride, DC_WIN_LINE_STRIDE);
		} else {
			tegra_dc_writel(dc,
				(unsigned long)win->phys_addr_u,
				DC_WINBUF_START_ADDR_U);
			tegra_dc_writel(dc,
				(unsigned long)win->phys_addr_v,
				DC_WINBUF_START_ADDR_V);
			tegra_dc_writel(dc,
				LINE_STRIDE(win->stride) |
				UV_LINE_STRIDE(win->stride_uv),
				DC_WIN_LINE_STRIDE);
		}

		h_offset = win->x;
		if (invert_h) {
			h_offset.full += win->w.full - dfixed_const(1);
		}

		v_offset = win->y;
		if (invert_v) {
			v_offset.full += win->h.full - dfixed_const(1);
		}

		tegra_dc_writel(dc, dfixed_trunc(h_offset) * Bpp,
				DC_WINBUF_ADDR_H_OFFSET);
		tegra_dc_writel(dc, dfixed_trunc(v_offset),
				DC_WINBUF_ADDR_V_OFFSET);

		if (tegra_dc_feature_has_tiling(dc, win->idx)) {
			if (WIN_IS_TILED(win))
				tegra_dc_writel(dc,
					DC_WIN_BUFFER_ADDR_MODE_TILE |
					DC_WIN_BUFFER_ADDR_MODE_TILE_UV,
					DC_WIN_BUFFER_ADDR_MODE);
			else
				tegra_dc_writel(dc,
					DC_WIN_BUFFER_ADDR_MODE_LINEAR |
					DC_WIN_BUFFER_ADDR_MODE_LINEAR_UV,
					DC_WIN_BUFFER_ADDR_MODE);
		}

		val = WIN_ENABLE;
		if (yuv)
			val |= CSC_ENABLE;
		else if (tegra_dc_fmt_bpp(win->fmt) < 24)
			val |= COLOR_EXPAND;

		if (win->ppflags & TEGRA_WIN_PPFLAG_CP_ENABLE)
			val |= CP_ENABLE;

		if (filter_h)
			val |= H_FILTER_ENABLE;
		if (filter_v)
			val |= V_FILTER_ENABLE;

		if (invert_h)
			val |= H_DIRECTION_DECREMENT;
		if (invert_v)
			val |= V_DIRECTION_DECREMENT;

		tegra_dc_writel(dc, val, DC_WIN_WIN_OPTIONS);

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
		if (win->global_alpha == 255)
			tegra_dc_writel(dc, 0, DC_WIN_GLOBAL_ALPHA);
		else
			tegra_dc_writel(dc, GLOBAL_ALPHA_ENABLE |
				win->global_alpha, DC_WIN_GLOBAL_ALPHA);
#endif

		win->dirty = no_vsync ? 0 : 1;

		dev_dbg(&dc->ndev->dev, "%s():idx=%d z=%d x=%d y=%d w=%d h=%d "
			"out_x=%u out_y=%u out_w=%u out_h=%u "
			"fmt=%d yuvp=%d Bpp=%u filter_h=%d filter_v=%d",
			__func__, win->idx, win->z,
			dfixed_trunc(win->x), dfixed_trunc(win->y),
			dfixed_trunc(win->w), dfixed_trunc(win->h),
			win->out_x, win->out_y, win->out_w, win->out_h,
			win->fmt, yuvp, Bpp, filter_h, filter_v);
		trace_printk("%s:win%u in:%ux%u out:%ux%u fmt=%d\n",
			dc->ndev->name, win->idx, dfixed_trunc(win->w),
			dfixed_trunc(win->h), win->out_w, win->out_h, win->fmt);
	}

	if (update_blend) {
		tegra_dc_set_blending(dc, &dc->blend);
		for (i = 0; i < DC_N_WINDOWS; i++) {
			if (!no_vsync)
				dc->windows[i].dirty = 1;
			update_mask |= WIN_A_ACT_REQ << i;
		}
	}

	tegra_dc_set_dynamic_emc(windows, n);

	tegra_dc_writel(dc, update_mask << 8, DC_CMD_STATE_CONTROL);

	tegra_dc_writel(dc, FRAME_END_INT | V_BLANK_INT, DC_CMD_INT_STATUS);
	if (!no_vsync) {
		set_bit(V_BLANK_FLIP, &dc->vblank_ref_count);
		tegra_dc_unmask_interrupt(dc,
			FRAME_END_INT | V_BLANK_INT | ALL_UF_INT);
	} else {
		clear_bit(V_BLANK_FLIP, &dc->vblank_ref_count);
		tegra_dc_mask_interrupt(dc, V_BLANK_INT | ALL_UF_INT);
		if (!atomic_read(&frame_end_ref))
			tegra_dc_mask_interrupt(dc, FRAME_END_INT);
	}

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		schedule_delayed_work(&dc->one_shot_work,
				msecs_to_jiffies(dc->one_shot_delay_ms));

	/* update EMC clock if calculated bandwidth has changed */
	tegra_dc_program_bandwidth(dc, false);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		update_mask |= NC_HOST_TRIG;

	tegra_dc_writel(dc, update_mask, DC_CMD_STATE_CONTROL);
	trace_printk("%s:update_mask=%#lx\n", dc->ndev->name, update_mask);

	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		mutex_unlock(&dc->one_shot_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_update_windows);

void tegra_dc_trigger_windows(struct tegra_dc *dc)
{
	u32 val, i;
	u32 completed = 0;
	u32 dirty = 0;

	val = tegra_dc_readl(dc, DC_CMD_STATE_CONTROL);
	for (i = 0; i < DC_N_WINDOWS; i++) {
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
		/* FIXME: this is not needed when the simulator
		   clears WIN_x_UPDATE bits as in HW */
		dc->windows[i].dirty = 0;
		completed = 1;
#else
		if (!(val & (WIN_A_ACT_REQ << i))) {
			dc->windows[i].dirty = 0;
			completed = 1;
		} else {
			dirty = 1;
		}
#endif
	}

	if (!dirty) {
		if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
			&& !atomic_read(&frame_end_ref))
			tegra_dc_mask_interrupt(dc, FRAME_END_INT);
	}

	if (completed)
		wake_up(&dc->wq);
}

