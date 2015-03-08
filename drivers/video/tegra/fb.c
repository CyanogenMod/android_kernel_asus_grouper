/*
 * drivers/video/tegra/fb.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *         Colin Cross <ccross@android.com>
 *         Travis Geiselbrecht <travis@palm.com>
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

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <video/tegrafb.h>

#include <mach/dc.h>
#include <mach/fb.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <linux/console.h>


#include "host/dev.h"
#include "nvmap/nvmap.h"
#include "dc/dc_priv.h"

/* Pad pitch to 16-byte boundary. */
#define TEGRA_LINEAR_PITCH_ALIGNMENT 32

struct tegra_fb_info {
	struct tegra_dc_win	*win;
	struct nvhost_device	*ndev;
	struct fb_info		*info;
	bool			valid;

	struct resource		*fb_mem;

	int			xres;
	int			yres;
};

/* palette array used by the fbcon */
static u32 pseudo_palette[16];

static int tegra_fb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct tegra_dc *dc = tegra_fb->win->dc;
	struct tegra_dc_out_ops *ops = dc->out_ops;
	struct fb_videomode mode;

	if ((var->yres * var->xres * var->bits_per_pixel / 8 * 2) >
	    info->screen_size)
		return -EINVAL;

	/* Apply mode filter for HDMI only -LVDS supports only fix mode */
	if (ops && ops->mode_filter) {

		fb_var_to_videomode(&mode, var);
		if (!ops->mode_filter(dc, &mode))
			return -EINVAL;

		/* Mode filter may have modified the mode */
		fb_videomode_to_var(var, &mode);
	}

	/* Double yres_virtual to allow double buffering through pan_display */
	var->yres_virtual = var->yres * 2;

	return 0;
}

static int tegra_fb_set_par(struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct tegra_dc *dc = tegra_fb->win->dc;

	if (var->bits_per_pixel) {
		/* we only support RGB ordering for now */
		switch (var->bits_per_pixel) {
		case 32:
			var->red.offset = 0;
			var->red.length = 8;
			var->green.offset = 8;
			var->green.length = 8;
			var->blue.offset = 16;
			var->blue.length = 8;
			var->transp.offset = 24;
			var->transp.length = 8;
			tegra_fb->win->fmt = TEGRA_WIN_FMT_R8G8B8A8;
			break;
		case 16:
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			tegra_fb->win->fmt = TEGRA_WIN_FMT_B5G6R5;
			break;

		default:
			return -EINVAL;
		}
		/* if line_length unset, then pad the stride */
		info->fix.line_length = var->xres * var->bits_per_pixel / 8;
		info->fix.line_length = round_up(info->fix.line_length,
					TEGRA_LINEAR_PITCH_ALIGNMENT);
		tegra_fb->win->stride = info->fix.line_length;
		tegra_fb->win->stride_uv = 0;
		tegra_fb->win->phys_addr_u = 0;
		tegra_fb->win->phys_addr_v = 0;
	}

	if (var->pixclock) {
		bool stereo;
		unsigned old_len = 0;
		struct fb_videomode m;
		struct fb_videomode *old_mode = NULL;

		fb_var_to_videomode(&m, var);

		/* Load framebuffer info with new mode details*/
		old_mode = info->mode;
		old_len  = info->fix.line_length;

		info->mode = (struct fb_videomode *)
			fb_find_nearest_mode(&m, &info->modelist);
		if (!info->mode) {
			dev_warn(&tegra_fb->ndev->dev, "can't match video mode\n");
			info->mode = old_mode;
			return -EINVAL;
		}

		/* Update fix line_length and window stride as per new mode */
		info->fix.line_length = var->xres * var->bits_per_pixel / 8;
		info->fix.line_length = round_up(info->fix.line_length,
			TEGRA_LINEAR_PITCH_ALIGNMENT);
		tegra_fb->win->stride = info->fix.line_length;

		/*
		 * only enable stereo if the mode supports it and
		 * client requests it
		 */
		stereo = !!(var->vmode & info->mode->vmode &
#ifndef CONFIG_TEGRA_HDMI_74MHZ_LIMIT
					FB_VMODE_STEREO_FRAME_PACK);
#else
					FB_VMODE_STEREO_LEFT_RIGHT);
#endif

		/* Configure DC with new mode */
		if (tegra_dc_set_fb_mode(dc, info->mode, stereo)) {
			/* Error while configuring DC, fallback to old mode */
			dev_warn(&tegra_fb->ndev->dev, "can't configure dc with mode %ux%u\n",
				info->mode->xres, info->mode->yres);
			info->mode = old_mode;
			info->fix.line_length = old_len;
			tegra_fb->win->stride = old_len;
			return -EINVAL;
		}

		/* Reflect mode chnage on DC HW */
		if (dc->enabled)
			tegra_dc_disable(dc);
		tegra_dc_enable(dc);

		tegra_fb->win->w.full = dfixed_const(info->mode->xres);
		tegra_fb->win->h.full = dfixed_const(info->mode->yres);
		tegra_fb->win->out_w = info->mode->xres;
		tegra_fb->win->out_h = info->mode->yres;
	}
	return 0;
}

static int tegra_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transp, struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
	    info->fix.visual == FB_VISUAL_DIRECTCOLOR) {
		u32 v;

		if (regno >= 16)
			return -EINVAL;

		red = (red >> (16 - info->var.red.length));
		green = (green >> (16 - info->var.green.length));
		blue = (blue >> (16 - info->var.blue.length));

		v = (red << var->red.offset) |
			(green << var->green.offset) |
			(blue << var->blue.offset);

		((u32 *)info->pseudo_palette)[regno] = v;
	}

	return 0;
}


static int tegra_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct tegra_dc *dc = tegra_fb->win->dc;
	int i;
	u16 *red = cmap->red;
	u16 *green = cmap->green;
	u16 *blue = cmap->blue;
	int start = cmap->start;

	if (((unsigned)start > 255) || ((start + cmap->len) > 256))
		return -EINVAL;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
		info->fix.visual == FB_VISUAL_DIRECTCOLOR) {
		/*
		 * For now we are considering color schemes with
		 * cmap->len <=16 as special case of basic color
		 * scheme to support fbconsole.But for DirectColor
		 * visuals(like the one we actually have, that include
		 * a HW LUT),the way it's intended to work is that the
		 * actual LUT HW is programmed to the intended values,
		 * even for small color maps like those with 16 or fewer
		 * entries. The pseudo_palette is then programmed to the
		 * identity transform.
		 */
		if (cmap->len <= 16) {
			/* Low-color schemes like fbconsole*/
			u16 *transp = cmap->transp;
			u_int vtransp = 0xffff;

			for (i = 0; i < cmap->len; i++) {
				if (transp)
					vtransp = *transp++;
				if (tegra_fb_setcolreg(start++, *red++,
					*green++, *blue++,
					vtransp, info))
						return -EINVAL;
			}
		} else {
			/* High-color schemes*/
			for (i = 0; i < cmap->len; i++) {
				dc->fb_lut.r[start+i] = *red++ >> 8;
				dc->fb_lut.g[start+i] = *green++ >> 8;
				dc->fb_lut.b[start+i] = *blue++ >> 8;
			}
			tegra_dc_update_lut(dc, -1, -1);
		}
	}
	return 0;
}

static int tegra_fb_blank(int blank, struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		dev_dbg(&tegra_fb->ndev->dev, "unblank\n");
		tegra_fb->win->flags = TEGRA_WIN_FLAG_ENABLED;
		tegra_dc_enable(tegra_fb->win->dc);
		return 0;

	case FB_BLANK_NORMAL:
		dev_dbg(&tegra_fb->ndev->dev, "blank - normal\n");
		tegra_dc_blank(tegra_fb->win->dc);
		return 0;

	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		dev_dbg(&tegra_fb->ndev->dev, "blank - powerdown\n");
		tegra_dc_disable(tegra_fb->win->dc);
		return 0;

	default:
		return -ENOTTY;
	}
}

static int tegra_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	char __iomem *flush_start;
	char __iomem *flush_end;
	u32 addr;

	if (!tegra_fb->win->cur_handle) {
		flush_start = info->screen_base + (var->yoffset * info->fix.line_length);
		flush_end = flush_start + (var->yres * info->fix.line_length);

		info->var.xoffset = var->xoffset;
		info->var.yoffset = var->yoffset;

		addr = info->fix.smem_start + (var->yoffset * info->fix.line_length) +
			(var->xoffset * (var->bits_per_pixel/8));

		tegra_fb->win->phys_addr = addr;
		tegra_fb->win->flags = TEGRA_WIN_FLAG_ENABLED;
		tegra_fb->win->virt_addr = info->screen_base;

		tegra_dc_update_windows(&tegra_fb->win, 1);
		tegra_dc_sync_windows(&tegra_fb->win, 1);
	}

	return 0;
}

static void tegra_fb_fillrect(struct fb_info *info,
			      const struct fb_fillrect *rect)
{
	cfb_fillrect(info, rect);
}

static void tegra_fb_copyarea(struct fb_info *info,
			      const struct fb_copyarea *region)
{
	cfb_copyarea(info, region);
}

static void tegra_fb_imageblit(struct fb_info *info,
			       const struct fb_image *image)
{
	cfb_imageblit(info, image);
}

static int tegra_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct tegra_fb_info *tegra_fb = (struct tegra_fb_info *)info->par;
	struct tegra_fb_modedb modedb;
	struct fb_modelist *modelist;
	struct fb_vblank vblank = {};
	int i;

	switch (cmd) {
	case FBIO_TEGRA_GET_MODEDB:
		if (copy_from_user(&modedb, (void __user *)arg, sizeof(modedb)))
			return -EFAULT;

		i = 0;
		list_for_each_entry(modelist, &info->modelist, list) {
			struct fb_var_screeninfo var;

			if (i >= modedb.modedb_len)
				break;

			/* fb_videomode_to_var doesn't fill out all the members
			   of fb_var_screeninfo */
			memset(&var, 0x0, sizeof(var));

			fb_videomode_to_var(&var, &modelist->mode);

			if (copy_to_user((void __user *)&modedb.modedb[i],
					 &var, sizeof(var)))
				return -EFAULT;
			i++;

			if (var.vmode & FB_VMODE_STEREO_MASK) {
				if (i >= modedb.modedb_len)
					break;
				var.vmode &= ~FB_VMODE_STEREO_MASK;
				if (copy_to_user(
					(void __user *)&modedb.modedb[i],
					 &var, sizeof(var)))
					return -EFAULT;
				i++;
			}
		}
		modedb.modedb_len = i;

		if (copy_to_user((void __user *)arg, &modedb, sizeof(modedb)))
			return -EFAULT;
		break;

	case FBIOGET_VBLANK:
		tegra_dc_get_fbvblank(tegra_fb->win->dc, &vblank);

		if (copy_to_user(
			(void __user *)arg, &vblank, sizeof(vblank)))
			return -EFAULT;
		break;

	case FBIO_WAITFORVSYNC:
		return tegra_dc_wait_for_vsync(tegra_fb->win->dc);

	default:
		return -ENOTTY;
	}

	return 0;
}

int tegra_fb_get_mode(struct tegra_dc *dc)
{
	if (!dc->fb->info->mode)
		return -1;

	return dc->fb->info->mode->refresh;
}

int tegra_fb_set_mode(struct tegra_dc *dc, int fps) {
	size_t stereo;
	struct list_head *pos;
	struct fb_videomode *best_mode = NULL;
	int curr_diff = INT_MAX; /* difference of best_mode refresh rate */
	struct fb_modelist *modelist;
	struct fb_info *info = dc->fb->info;

	list_for_each(pos, &info->modelist) {
		struct fb_videomode *mode;

		modelist = list_entry(pos, struct fb_modelist, list);
		mode = &modelist->mode;
		if (fps <= mode->refresh && curr_diff > (mode->refresh - fps)) {
			curr_diff = mode->refresh - fps;
			best_mode = mode;
		}
	}
	if (best_mode) {
		info->mode = best_mode;
		stereo = !!(info->var.vmode & info->mode->vmode &
#ifndef CONFIG_TEGRA_HDMI_74MHZ_LIMIT
				FB_VMODE_STEREO_FRAME_PACK);
#else
				FB_VMODE_STEREO_LEFT_RIGHT);
#endif
		return tegra_dc_set_fb_mode(dc, best_mode, stereo);
	}
	return -EIO;
}

static struct fb_ops tegra_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = tegra_fb_check_var,
	.fb_set_par = tegra_fb_set_par,
	.fb_setcmap = tegra_fb_setcmap,
	.fb_blank = tegra_fb_blank,
	.fb_pan_display = tegra_fb_pan_display,
	.fb_fillrect = tegra_fb_fillrect,
	.fb_copyarea = tegra_fb_copyarea,
	.fb_imageblit = tegra_fb_imageblit,
	.fb_ioctl = tegra_fb_ioctl,
};

const struct fb_videomode *tegra_fb_find_best_mode(
	struct fb_var_screeninfo *var,
	struct list_head *head)
{
	struct list_head *pos;
	struct fb_modelist *modelist;
	struct fb_videomode *mode, *best = NULL;
	int diff = 0;

	list_for_each(pos, head) {
		int d;

		modelist = list_entry(pos, struct fb_modelist, list);
		mode = &modelist->mode;

		if (mode->xres >= var->xres && mode->yres >= var->yres) {
			d = (mode->xres - var->xres) +
				(mode->yres - var->yres);
			if (diff < d) {
				diff = d;
				best = mode;
			} else if (diff == d && best &&
				   mode->refresh > best->refresh)
				best = mode;
		}
	}
	return best;
}

void tegra_fb_update_monspecs(struct tegra_fb_info *fb_info,
			      struct fb_monspecs *specs,
			      bool (*mode_filter)(const struct tegra_dc *dc,
						  struct fb_videomode *mode))
{
	int i;
	bool first = false;
	struct fb_event event;
	struct fb_info *info = fb_info->info;
	struct fb_var_screeninfo var = {0,};

	mutex_lock(&fb_info->info->lock);
	fb_destroy_modedb(fb_info->info->monspecs.modedb);

	fb_destroy_modelist(&fb_info->info->modelist);

	if (specs == NULL) {
		struct tegra_dc_mode mode;
		memset(&fb_info->info->monspecs, 0x0,
		       sizeof(fb_info->info->monspecs));
		memset(&mode, 0x0, sizeof(mode));

		/*
		 * reset video mode properties to prevent garbage being displayed on 'mode' device.
		 */
		fb_info->info->mode = (struct fb_videomode*) NULL;

		tegra_dc_set_mode(fb_info->win->dc, &mode);
		mutex_unlock(&fb_info->info->lock);
		return;
	}

	memcpy(&fb_info->info->monspecs, specs,
	       sizeof(fb_info->info->monspecs));
	fb_info->info->mode = specs->modedb;

	/* Prepare a mode db */
	for (i = 0; i < specs->modedb_len; i++) {
		if (info->fbops->fb_check_var) {
			struct fb_videomode m;

			/* Call mode filter to check mode */
			fb_videomode_to_var(&var, &specs->modedb[i]);
			if (!(info->fbops->fb_check_var(&var, info))) {
				fb_var_to_videomode(&m, &var);
				fb_add_videomode(&m,
						 &fb_info->info->modelist);
				/* EDID stds recommend first detailed mode
				to be applied as default,but if first mode
				doesn't pass mode filter, we have to select
				and apply other mode. So flag on if first
				mode passes mode filter */
				if (!i)
					first = true;
			}
		} else {
			fb_add_videomode(&specs->modedb[i],
					 &fb_info->info->modelist);
		}
	}

	/* We can't apply first detailed mode, so get the best mode
	based on resolution and apply on fb */
	if (!first) {
		var.xres = 0;
		var.yres = 0;
		info->mode = (struct fb_videomode *)
			tegra_fb_find_best_mode(&var, &info->modelist);
	}

	/* Prepare fb info with new mode details */
	fb_videomode_to_var(&info->var, info->mode);
	event.info = fb_info->info;

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	/* Send a noti to change fb_display[].mode for all vc's */
	console_lock();
	fb_notifier_call_chain(FB_EVENT_MODE_CHANGE_ALL, &event);
	console_unlock();

	/* Notify framebuffer console about mode change */
	console_lock();
	fb_notifier_call_chain(FB_EVENT_NEW_MODELIST, &event);
	console_unlock();
#else
	fb_notifier_call_chain(FB_EVENT_NEW_MODELIST, &event);
#endif

	mutex_unlock(&fb_info->info->lock);
}

struct tegra_fb_info *tegra_fb_register(struct nvhost_device *ndev,
					struct tegra_dc *dc,
					struct tegra_fb_data *fb_data,
					struct resource *fb_mem)
{
	struct tegra_dc_win *win;
	struct fb_info *info;
	struct tegra_fb_info *tegra_fb;
	void __iomem *fb_base = NULL;
	unsigned long fb_size = 0;
	unsigned long fb_phys = 0;
	int ret = 0;
	unsigned stride;

	win = tegra_dc_get_window(dc, fb_data->win);
	if (!win) {
		dev_err(&ndev->dev, "dc does not have a window at index %d\n",
			fb_data->win);
		return ERR_PTR(-ENOENT);
	}

	info = framebuffer_alloc(sizeof(struct tegra_fb_info), &ndev->dev);
	if (!info) {
		ret = -ENOMEM;
		goto err;
	}

	tegra_fb = info->par;
	tegra_fb->win = win;
	tegra_fb->ndev = ndev;
	tegra_fb->fb_mem = fb_mem;
	tegra_fb->xres = fb_data->xres;
	tegra_fb->yres = fb_data->yres;

	if (fb_mem) {
		fb_size = resource_size(fb_mem);
		fb_phys = fb_mem->start;
		fb_base = ioremap_nocache(fb_phys, fb_size);
		if (!fb_base) {
			dev_err(&ndev->dev, "fb can't be mapped\n");
			ret = -EBUSY;
			goto err_free;
		}
		tegra_fb->valid = true;
	}

	stride = fb_data->xres * fb_data->bits_per_pixel / 8;
	stride = round_up(stride, TEGRA_LINEAR_PITCH_ALIGNMENT);

	info->fbops = &tegra_fb_ops;
	info->pseudo_palette = pseudo_palette;
	info->screen_base = fb_base;
	info->screen_size = fb_size;

	strlcpy(info->fix.id, "tegra_fb", sizeof(info->fix.id));
	info->fix.type		= FB_TYPE_PACKED_PIXELS;
	info->fix.visual	= FB_VISUAL_TRUECOLOR;
	info->fix.xpanstep	= 1;
	info->fix.ypanstep	= 1;
	info->fix.accel		= FB_ACCEL_NONE;
	info->fix.smem_start	= fb_phys;
	info->fix.smem_len	= fb_size;
	info->fix.line_length = stride;

	info->var.xres			= fb_data->xres;
	info->var.yres			= fb_data->yres;
	info->var.xres_virtual		= fb_data->xres;
	info->var.yres_virtual		= fb_data->yres * 2;
	info->var.bits_per_pixel	= fb_data->bits_per_pixel;
	info->var.activate		= FB_ACTIVATE_VBL;
	info->var.height		= tegra_dc_get_out_height(dc);
	info->var.width			= tegra_dc_get_out_width(dc);
	info->var.pixclock		= 0;
	info->var.left_margin		= 0;
	info->var.right_margin		= 0;
	info->var.upper_margin		= 0;
	info->var.lower_margin		= 0;
	info->var.hsync_len		= 0;
	info->var.vsync_len		= 0;
	info->var.vmode			= FB_VMODE_NONINTERLACED;

	win->x.full = dfixed_const(0);
	win->y.full = dfixed_const(0);
	win->w.full = dfixed_const(fb_data->xres);
	win->h.full = dfixed_const(fb_data->yres);
	/* TODO: set to output res dc */
	win->out_x = 0;
	win->out_y = 0;
	win->out_w = fb_data->xres;
	win->out_h = fb_data->yres;
	win->z = 0;
	win->phys_addr = fb_phys;
	win->virt_addr = fb_base;
	win->phys_addr_u = 0;
	win->phys_addr_v = 0;
	win->stride = info->fix.line_length;
	win->stride_uv = 0;
	win->flags = TEGRA_WIN_FLAG_ENABLED;

	if (fb_mem)
		tegra_fb_set_par(info);

	if (register_framebuffer(info)) {
		dev_err(&ndev->dev, "failed to register framebuffer\n");
		ret = -ENODEV;
		goto err_iounmap_fb;
	}

	tegra_fb->info = info;

	dev_info(&ndev->dev, "probed\n");

	if (fb_data->flags & TEGRA_FB_FLIP_ON_PROBE) {
		tegra_dc_update_windows(&tegra_fb->win, 1);
		tegra_dc_sync_windows(&tegra_fb->win, 1);
	}

	if (dc->mode.pclk > 1000) {
		struct tegra_dc_mode *mode = &dc->mode;
		struct fb_videomode vmode;

		if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
			info->var.pixclock = KHZ2PICOS(mode->rated_pclk / 1000);
		else
			info->var.pixclock = KHZ2PICOS(mode->pclk / 1000);
		info->var.left_margin = mode->h_back_porch;
		info->var.right_margin = mode->h_front_porch;
		info->var.upper_margin = mode->v_back_porch;
		info->var.lower_margin = mode->v_front_porch;
		info->var.hsync_len = mode->h_sync_width;
		info->var.vsync_len = mode->v_sync_width;

		/* Keep info->var consistent with info->modelist. */
		fb_var_to_videomode(&vmode, &info->var);
		fb_add_videomode(&vmode, &info->modelist);
	}

	return tegra_fb;

err_iounmap_fb:
	if (fb_base)
		iounmap(fb_base);
err_free:
	framebuffer_release(info);
err:
	return ERR_PTR(ret);
}

void tegra_fb_unregister(struct tegra_fb_info *fb_info)
{
	struct fb_info *info = fb_info->info;

	unregister_framebuffer(info);

	iounmap(info->screen_base);
	framebuffer_release(info);
}
