/*
 * drivers/video/tegra/dc/hdmi.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/device.h>

#include <mach/clk.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <linux/nvhost.h>
#include <mach/hdmi-audio.h>

#include <video/tegrafb.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "hdmi_reg.h"
#include "hdmi.h"
#include "edid.h"
#include "nvhdcp.h"

/* datasheet claims this will always be 216MHz */
#define HDMI_AUDIOCLK_FREQ		216000000

#define HDMI_REKEY_DEFAULT		56

#define HDMI_ELD_RESERVED1_INDEX		1
#define HDMI_ELD_RESERVED2_INDEX		3
#define HDMI_ELD_VER_INDEX			0
#define HDMI_ELD_BASELINE_LEN_INDEX		2
#define HDMI_ELD_CEA_VER_MNL_INDEX		4
#define HDMI_ELD_SAD_CNT_CON_TYP_SAI_HDCP_INDEX		5
#define HDMI_ELD_AUD_SYNC_DELAY_INDEX	6
#define HDMI_ELD_SPK_ALLOC_INDEX		7
#define HDMI_ELD_PORT_ID_INDEX		8
#define HDMI_ELD_MANF_NAME_INDEX		16
#define HDMI_ELD_PRODUCT_CODE_INDEX		18
#define HDMI_ELD_MONITOR_NAME_INDEX		20

/* These two values need to be cross checked in case of
     addition/removal from tegra_dc_hdmi_aspect_ratios[] */
#define TEGRA_DC_HDMI_MIN_ASPECT_RATIO_PERCENT	80
#define TEGRA_DC_HDMI_MAX_ASPECT_RATIO_PERCENT	320

/* Percentage equivalent of standard aspect ratios
    accurate upto two decimal digits */
static int tegra_dc_hdmi_aspect_ratios[] = {
	/*   3:2	*/	150,
	/*   4:3	*/	133,
	/*   4:5	*/	 80,
	/*   5:4	*/	125,
	/*   9:5	*/	180,
	/*  16:5	*/	320,
	/*  16:9	*/	178,
	/* 16:10	*/	160,
	/* 19:10	*/	190,
	/* 25:16	*/	156,
	/* 64:35	*/	183,
	/* 72:35	*/	206
};

struct tegra_dc_hdmi_data {
	struct tegra_dc			*dc;
	struct tegra_edid		*edid;
	struct tegra_edid_hdmi_eld		eld;
	struct tegra_nvhdcp		*nvhdcp;
	struct delayed_work		work;

	struct resource			*base_res;
	void __iomem			*base;
	struct clk			*clk;

	struct clk			*disp1_clk;
	struct clk			*disp2_clk;
	struct clk			*hda_clk;
	struct clk			*hda2codec_clk;
	struct clk			*hda2hdmi_clk;

#ifdef CONFIG_SWITCH
	struct switch_dev		hpd_switch;
#endif

	spinlock_t			suspend_lock;
	bool				suspended;
	bool				eld_retrieved;
	bool				clk_enabled;
	unsigned			audio_freq;
	unsigned			audio_source;
	bool				audio_inject_null;

	bool				dvi;
};

struct tegra_dc_hdmi_data *dc_hdmi;

const struct fb_videomode tegra_dc_hdmi_supported_modes[] = {
	/* 1280x720p 60hz: EIA/CEA-861-B Format 4 */
	{
		.xres =		1280,
		.yres =		720,
		.pixclock =	KHZ2PICOS(74250),
		.hsync_len =	40,	/* h_sync_width */
		.vsync_len =	5,	/* v_sync_width */
		.left_margin =	220,	/* h_back_porch */
		.upper_margin =	20,	/* v_back_porch */
		.right_margin =	110,	/* h_front_porch */
		.lower_margin =	5,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},

	/* 1280x720p 60hz: EIA/CEA-861-B Format 4 (Stereo)*/
	{
		.xres =		1280,
		.yres =		720,
		.pixclock =	KHZ2PICOS(74250),
		.hsync_len =	40,	/* h_sync_width */
		.vsync_len =	5,	/* v_sync_width */
		.left_margin =	220,	/* h_back_porch */
		.upper_margin =	20,	/* v_back_porch */
		.right_margin =	110,	/* h_front_porch */
		.lower_margin =	5,	/* v_front_porch */
		.vmode = FB_VMODE_NONINTERLACED |
#ifndef CONFIG_TEGRA_HDMI_74MHZ_LIMIT
				 FB_VMODE_STEREO_FRAME_PACK,
#else
				 FB_VMODE_STEREO_LEFT_RIGHT,
#endif
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},

	/* 720x480p 59.94hz: EIA/CEA-861-B Formats 2 & 3 */
	{
		.xres =		720,
		.yres =		480,
		.pixclock =	KHZ2PICOS(27000),
		.hsync_len =	62,	/* h_sync_width */
		.vsync_len =	6,	/* v_sync_width */
		.left_margin =	60,	/* h_back_porch */
		.upper_margin =	30,	/* v_back_porch */
		.right_margin =	16,	/* h_front_porch */
		.lower_margin =	9,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = 0,
	},

	/* 640x480p 60hz: EIA/CEA-861-B Format 1 */
	{
		.xres =		640,
		.yres =		480,
		.pixclock =	KHZ2PICOS(25200),
		.hsync_len =	96,	/* h_sync_width */
		.vsync_len =	2,	/* v_sync_width */
		.left_margin =	48,	/* h_back_porch */
		.upper_margin =	33,	/* v_back_porch */
		.right_margin =	16,	/* h_front_porch */
		.lower_margin =	10,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = 0,
	},

	/* 720x576p 50hz EIA/CEA-861-B Formats 17 & 18 */
	{
		.xres =		720,
		.yres =		576,
		.pixclock =	KHZ2PICOS(27000),
		.hsync_len =	64,	/* h_sync_width */
		.vsync_len =	5,	/* v_sync_width */
		.left_margin =	68,	/* h_back_porch */
		.upper_margin =	39,	/* v_back_porch */
		.right_margin =	12,	/* h_front_porch */
		.lower_margin =	5,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = 0,
	},

	/* 1920x1080p 23.98/24hz: EIA/CEA-861-B Format 32 (Stereo)*/
	{
		.xres =		1920,
		.yres =		1080,
		.pixclock =	KHZ2PICOS(74250),
		.hsync_len =	44,	/* h_sync_width */
		.vsync_len =	5,	/* v_sync_width */
		.left_margin =	148,	/* h_back_porch */
		.upper_margin =	36,	/* v_back_porch */
		.right_margin =	638,	/* h_front_porch */
		.lower_margin =	4,	/* v_front_porch */
		.vmode = FB_VMODE_NONINTERLACED |
#ifndef CONFIG_TEGRA_HDMI_74MHZ_LIMIT
				 FB_VMODE_STEREO_FRAME_PACK,
#else
				 FB_VMODE_STEREO_LEFT_RIGHT,
#endif
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},

	/* 1920x1080p 30Hz EIA/CEA-861-B Format 34 */
	{
		.xres =		1920,
		.yres =		1080,
		.pixclock =	KHZ2PICOS(74250),
		.hsync_len =	44,	/* h_sync_width */
		.vsync_len =	5,	/* v_sync_width */
		.left_margin =	148,	/* h_back_porch */
		.upper_margin =	36,	/* v_back_porch */
		.right_margin =	88,	/* h_front_porch */
		.lower_margin =	4,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},

	/* 1920x1080p 59.94/60hz CVT */
	{
		.xres =		1920,
		.yres =		1080,
		.pixclock =	KHZ2PICOS(138500),
		.hsync_len =	32,	/* h_sync_width */
		.vsync_len =	5,	/* v_sync_width */
		.left_margin =	80,	/* h_back_porch */
		.upper_margin =	23,	/* v_back_porch */
		.right_margin =	48,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode = FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},

	/* 1920x1080p 59.94/60hz EIA/CEA-861-B Format 16 */
	{
		.xres =		1920,
		.yres =		1080,
		.pixclock =	KHZ2PICOS(148500),
		.hsync_len =	44,	/* h_sync_width */
		.vsync_len =	5,	/* v_sync_width */
		.left_margin =	148,	/* h_back_porch */
		.upper_margin =	36,	/* v_back_porch */
		.right_margin =	88,	/* h_front_porch */
		.lower_margin =	4,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},

	/*
	* Few VGA/SVGA modes to support monitors with lower
	* resolutions or to support HDMI<->DVI connection
	*/

	/* 640x480p 75hz */
	{
		.xres =		640,
		.yres =		480,
		.pixclock =	KHZ2PICOS(31500),
		.hsync_len =	96,	/* h_sync_width */
		.vsync_len =	2,	/* v_sync_width */
		.left_margin =	48,	/* h_back_porch */
		.upper_margin =	32,	/* v_back_porch */
		.right_margin =	16,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = 0,
	},
	/* 720x400p 59hz */
	{
		.xres =		720,
		.yres =		400,
		.pixclock =	KHZ2PICOS(35500),
		.hsync_len =	72,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	108,	/* h_back_porch */
		.upper_margin =	42,	/* v_back_porch */
		.right_margin =	36,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync  = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 800x600p 60hz */
	{
		.xres =		800,
		.yres =		600,
		.pixclock =	KHZ2PICOS(40000),
		.hsync_len =	128,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	88,	/* h_back_porch */
		.upper_margin =	23,	/* v_back_porch */
		.right_margin =	40,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},
	/* 800x600p 75hz */
	{
		.xres =		800,
		.yres =		600,
		.pixclock =	KHZ2PICOS(49500),
		.hsync_len =	80,	/* h_sync_width */
		.vsync_len =	2,	/* v_sync_width */
		.left_margin =	160,	/* h_back_porch */
		.upper_margin =	21,	/* v_back_porch */
		.right_margin =	16,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1024x768p 60hz */
	{
		.xres =		1024,
		.yres =		768,
		.pixclock =	KHZ2PICOS(65000),
		.hsync_len =	136,	/* h_sync_width */
		.vsync_len =	6,	/* v_sync_width */
		.left_margin =	160,	/* h_back_porch */
		.upper_margin =	29,	/* v_back_porch */
		.right_margin =	24,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =	0,
	},
	/* 1024x768p 75hz */
	{
		.xres =		1024,
		.yres =		768,
		.pixclock =	KHZ2PICOS(78800),
		.hsync_len =	96,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	176,	/* h_back_porch */
		.upper_margin =	28,	/* v_back_porch */
		.right_margin =	16,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = 0,
	},
	/* 1152x864p 75hz */
	{
		.xres =		1152,
		.yres =		864,
		.pixclock =	KHZ2PICOS(108000),
		.hsync_len =	128,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	256,	/* h_back_porch */
		.upper_margin =	32,	/* v_back_porch */
		.right_margin =	64,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1280x800p 60hz */
	{
		.xres =		1280,
		.yres =		800,
		.pixclock =	KHZ2PICOS(83460),
		.hsync_len =	136,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	200,	/* h_back_porch */
		.upper_margin =	24,	/* v_back_porch */
		.right_margin =	64,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =		FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1280x960p 60hz */
	{
		.xres =		1280,
		.yres =		960,
		.pixclock =	KHZ2PICOS(108000),
		.hsync_len =	136,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	216,	/* h_back_porch */
		.upper_margin =	30,	/* v_back_porch */
		.right_margin =	80,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =		FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1280x1024p 60hz */
	{
		.xres =		1280,
		.yres =		1024,
		.pixclock =	KHZ2PICOS(108000),
		.hsync_len =	112,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	248,	/* h_back_porch */
		.upper_margin =	38,	/* v_back_porch */
		.right_margin =	48,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1280x1024p 75hz */
	{
		.xres =		1280,
		.yres =		1024,
		.pixclock =	KHZ2PICOS(135000),
		.hsync_len =	144,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	248,	/* h_back_porch */
		.upper_margin =	38,	/* v_back_porch */
		.right_margin =	16,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1368x768p 60hz */
	{
		.xres =		1368,
		.yres =		768,
		.pixclock =	KHZ2PICOS(85860),
		.hsync_len =	144,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	216,	/* h_back_porch */
		.upper_margin =	23,	/* v_back_porch */
		.right_margin =	72,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =		FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1440x900p 60hz */
	{
		.xres =		1440,
		.yres =		900,
		.pixclock =	KHZ2PICOS(106470),
		.hsync_len =	152,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	232,	/* h_back_porch */
		.upper_margin =	28,	/* v_back_porch */
		.right_margin =	80,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =		FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1600x1200p 60hz */
	{
		.xres =		1600,
		.yres =		1200,
		.pixclock =	KHZ2PICOS(162000),
		.hsync_len =	192,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	304,	/* h_back_porch */
		.upper_margin =	46,	/* v_back_porch */
		.right_margin =	64,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1600x1200p 75hz */
	{
		.xres =		1600,
		.yres =		1200,
		.pixclock =	KHZ2PICOS(202500),
		.hsync_len =	192,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	304,	/* h_back_porch */
		.upper_margin =	46,	/* v_back_porch */
		.right_margin =	64,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1680x1050p 59.94/60hz */
	{
		.xres =		1680,
		.yres =		1050,
		.pixclock =	KHZ2PICOS(147140),
		.hsync_len =	184,	/* h_sync_width */
		.vsync_len =	3,	/* v_sync_width */
		.left_margin =	288,	/* h_back_porch */
		.upper_margin =	33,	/* v_back_porch */
		.right_margin =	104,	/* h_front_porch */
		.lower_margin =	1,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =	FB_SYNC_VERT_HIGH_ACT,
	},
};

/* CVT timing representation of VESA modes*/
const struct fb_videomode tegra_dc_hdmi_supported_cvt_modes[] = {

	/* 640x480p 60hz */
	{
		.refresh =	60,
		.xres =		640,
		.yres =		480,
		.pixclock =	KHZ2PICOS(23750),
		.hsync_len =	64,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	80,	/* h_back_porch */
		.upper_margin =	17,	/* v_back_porch */
		.right_margin =	16,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 640x480p 75hz */
	{
		.refresh =	75,
		.xres =		640,
		.yres =		480,
		.pixclock =	KHZ2PICOS(30750),
		.hsync_len =	64,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	88,	/* h_back_porch */
		.upper_margin =	21,	/* v_back_porch */
		.right_margin =	24,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 720x400p 59hz */
	{
		.refresh =	59,
		.xres =		720,
		.yres =		400,
		.pixclock =	KHZ2PICOS(22000),
		.hsync_len =	64,	/* h_sync_width */
		.vsync_len =	10,	/* v_sync_width */
		.left_margin =	88,	/* h_back_porch */
		.upper_margin =	14,	/* v_back_porch */
		.right_margin =	24,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync  = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 800x600p 60hz */
	{
		.refresh =	60,
		.xres =		800,
		.yres =		600,
		.pixclock =	KHZ2PICOS(38250),
		.hsync_len =	80,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	112,	/* h_back_porch */
		.upper_margin =	21,	/* v_back_porch */
		.right_margin =	32,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = 	FB_SYNC_VERT_HIGH_ACT,
	},
	/* 800x600p 75hz */
	{
		.refresh =	75,
		.xres =		800,
		.yres =		600,
		.pixclock =	KHZ2PICOS(49000),
		.hsync_len =	80,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	120,	/* h_back_porch */
		.upper_margin =	26,	/* v_back_porch */
		.right_margin =	40,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1024x768p 60hz */
	{
		.refresh =	60,
		.xres =		1024,
		.yres =		768,
		.pixclock =	KHZ2PICOS(63500),
		.hsync_len =	104,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	152,	/* h_back_porch */
		.upper_margin =	27,	/* v_back_porch */
		.right_margin =	48,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =	FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1024x768p 75hz */
	{
		.refresh =	75,
		.xres =		1024,
		.yres =		768,
		.pixclock =	KHZ2PICOS(82000),
		.hsync_len =	104,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	168,	/* h_back_porch */
		.upper_margin =	34,	/* v_back_porch */
		.right_margin =	64,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1152x864p 75hz */
	{
		.refresh =	75,
		.xres =		1152,
		.yres =		864,
		.pixclock =	KHZ2PICOS(104500),
		.hsync_len =	120,	/* h_sync_width */
		.vsync_len =	10,	/* v_sync_width */
		.left_margin =	192,	/* h_back_porch */
		.upper_margin =	38,	/* v_back_porch */
		.right_margin =	72,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1280x800p 60hz */
	{
		.refresh =	60,
		.xres =		1280,
		.yres =		800,
		.pixclock =	KHZ2PICOS(83500),
		.hsync_len =	128,	/* h_sync_width */
		.vsync_len =	6,	/* v_sync_width */
		.left_margin =	200,	/* h_back_porch */
		.upper_margin =	28,	/* v_back_porch */
		.right_margin =	72,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =		FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1280x960p 60hz */
	{
		.refresh =	60,
		.xres =		1280,
		.yres =		960,
		.pixclock =	KHZ2PICOS(101250),
		.hsync_len =	128,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	208,	/* h_back_porch */
		.upper_margin =	33,	/* v_back_porch */
		.right_margin =	80,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1280x1024p 60hz */
	{
		.refresh =	60,
		.xres =		1280,
		.yres =		1024,
		.pixclock =	KHZ2PICOS(109000),
		.hsync_len =	136,	/* h_sync_width */
		.vsync_len =	7,	/* v_sync_width */
		.left_margin =	216,	/* h_back_porch */
		.upper_margin =	36,	/* v_back_porch */
		.right_margin =	80,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =	FB_SYNC_VERT_HIGH_ACT,
	},

	/* 1280x1024p 75hz */
	{
		.refresh =	75,
		.xres =		1280,
		.yres =		1024,
		.pixclock =	KHZ2PICOS(138750),
		.hsync_len =	136,	/* h_sync_width */
		.vsync_len =	7,	/* v_sync_width */
		.left_margin =	224,	/* h_back_porch */
		.upper_margin =	45,	/* v_back_porch */
		.right_margin =	88,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1368x768p 60hz */
	{
		.refresh =	60,
		.xres =		1368,
		.yres =		768,
		.pixclock =	KHZ2PICOS(85250),
		.hsync_len =	136,	/* h_sync_width */
		.vsync_len =	10,	/* v_sync_width */
		.left_margin =	208,	/* h_back_porch */
		.upper_margin =	27,	/* v_back_porch */
		.right_margin =	72,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =		FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1440x900p 60hz */
	{
		.refresh =	60,
		.xres =		1440,
		.yres =		900,
		.pixclock =	KHZ2PICOS(106500),
		.hsync_len =	152,	/* h_sync_width */
		.vsync_len =	6,	/* v_sync_width */
		.left_margin =	232,	/* h_back_porch */
		.upper_margin =	31,	/* v_back_porch */
		.right_margin =	80,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =		FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1600x1200p 60hz */
	{
		.refresh =	60,
		.xres =		1600,
		.yres =		1200,
		.pixclock =	KHZ2PICOS(161000),
		.hsync_len =	168,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	280,	/* h_back_porch */
		.upper_margin =	42,	/* v_back_porch */
		.right_margin =	112,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1600x1200p 75hz */
	{
		.refresh =	75,
		.xres =		1600,
		.yres =		1200,
		.pixclock =	KHZ2PICOS(204750),
		.hsync_len =	168,	/* h_sync_width */
		.vsync_len =	4,	/* v_sync_width */
		.left_margin =	288,	/* h_back_porch */
		.upper_margin =	52,	/* v_back_porch */
		.right_margin =	120,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync = FB_SYNC_VERT_HIGH_ACT,
	},
	/* 1680x1050p 59.94/60hz */
	{
		.refresh =	60,
		.xres =		1680,
		.yres =		1050,
		.pixclock =	KHZ2PICOS(140000),
		.hsync_len =	168,	/* h_sync_width */
		.vsync_len =	10,	/* v_sync_width */
		.left_margin =	272,	/* h_back_porch */
		.upper_margin =	36,	/* v_back_porch */
		.right_margin =	104,	/* h_front_porch */
		.lower_margin =	3,	/* v_front_porch */
		.vmode =	FB_VMODE_NONINTERLACED,
		.sync =	FB_SYNC_VERT_HIGH_ACT,
	},
};

/* table of electrical settings, must be in acending order. */
struct tdms_config {
	int pclk;
	u32 pll0;
	u32 pll1;
	u32 pe_current; /* pre-emphasis */
	u32 drive_current;
};

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
const struct tdms_config tdms_config[] = {
	{ /* 480p modes */
	.pclk = 27000000,
	.pll0 = SOR_PLL_BG_V17_S(3) | SOR_PLL_ICHPMP(1) | SOR_PLL_RESISTORSEL |
		SOR_PLL_VCOCAP(0) | SOR_PLL_TX_REG_LOAD(0),
	.pll1 = SOR_PLL_TMDS_TERM_ENABLE,
	.pe_current = PE_CURRENT0(PE_CURRENT_0_0_mA) |
		PE_CURRENT1(PE_CURRENT_0_0_mA) |
		PE_CURRENT2(PE_CURRENT_0_0_mA) |
		PE_CURRENT3(PE_CURRENT_0_0_mA),
	.drive_current = DRIVE_CURRENT_LANE0(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE1(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE2(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE3(DRIVE_CURRENT_5_250_mA),
	},
	{ /* 720p modes */
	.pclk = 74250000,
	.pll0 = SOR_PLL_BG_V17_S(3) | SOR_PLL_ICHPMP(1) | SOR_PLL_RESISTORSEL |
		SOR_PLL_VCOCAP(1) | SOR_PLL_TX_REG_LOAD(0),
	.pll1 = SOR_PLL_TMDS_TERM_ENABLE | SOR_PLL_PE_EN,
	.pe_current = PE_CURRENT0(PE_CURRENT_5_0_mA) |
		PE_CURRENT1(PE_CURRENT_5_0_mA) |
		PE_CURRENT2(PE_CURRENT_5_0_mA) |
		PE_CURRENT3(PE_CURRENT_5_0_mA),
	.drive_current = DRIVE_CURRENT_LANE0(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE1(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE2(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE3(DRIVE_CURRENT_5_250_mA),
	},
	{ /* 1080p modes */
	.pclk = INT_MAX,
	.pll0 = SOR_PLL_BG_V17_S(3) | SOR_PLL_ICHPMP(1) | SOR_PLL_RESISTORSEL |
		SOR_PLL_VCOCAP(3) | SOR_PLL_TX_REG_LOAD(0),
	.pll1 = SOR_PLL_TMDS_TERM_ENABLE | SOR_PLL_PE_EN,
	.pe_current = PE_CURRENT0(PE_CURRENT_5_0_mA) |
		PE_CURRENT1(PE_CURRENT_5_0_mA) |
		PE_CURRENT2(PE_CURRENT_5_0_mA) |
		PE_CURRENT3(PE_CURRENT_5_0_mA),
	.drive_current = DRIVE_CURRENT_LANE0(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE1(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE2(DRIVE_CURRENT_5_250_mA) |
		DRIVE_CURRENT_LANE3(DRIVE_CURRENT_5_250_mA),
	},
};
#else /*  CONFIG_ARCH_TEGRA_2x_SOC */
const struct tdms_config tdms_config[] = {
	{ /* 480p modes */
	.pclk = 27000000,
	.pll0 = SOR_PLL_BG_V17_S(3) | SOR_PLL_ICHPMP(1) | SOR_PLL_RESISTORSEL |
		SOR_PLL_VCOCAP(0) | SOR_PLL_TX_REG_LOAD(3),
	.pll1 = SOR_PLL_TMDS_TERM_ENABLE,
	.pe_current = PE_CURRENT0(PE_CURRENT_0_0_mA) |
		PE_CURRENT1(PE_CURRENT_0_0_mA) |
		PE_CURRENT2(PE_CURRENT_0_0_mA) |
		PE_CURRENT3(PE_CURRENT_0_0_mA),
	.drive_current = DRIVE_CURRENT_LANE0(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE1(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE2(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE3(DRIVE_CURRENT_7_125_mA),
	},
	{ /* 720p modes */
	.pclk = 74250000,
	.pll0 = SOR_PLL_BG_V17_S(3) | SOR_PLL_ICHPMP(1) | SOR_PLL_RESISTORSEL |
		SOR_PLL_VCOCAP(1) | SOR_PLL_TX_REG_LOAD(3),
	.pll1 = SOR_PLL_TMDS_TERM_ENABLE | SOR_PLL_PE_EN,
	.pe_current = PE_CURRENT0(PE_CURRENT_6_0_mA) |
		PE_CURRENT1(PE_CURRENT_6_0_mA) |
		PE_CURRENT2(PE_CURRENT_6_0_mA) |
		PE_CURRENT3(PE_CURRENT_6_0_mA),
	.drive_current = DRIVE_CURRENT_LANE0(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE1(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE2(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE3(DRIVE_CURRENT_7_125_mA),
	},
	{ /* 1080p modes */
	.pclk = INT_MAX,
	.pll0 = SOR_PLL_BG_V17_S(3) | SOR_PLL_ICHPMP(1) | SOR_PLL_RESISTORSEL |
		SOR_PLL_VCOCAP(1) | SOR_PLL_TX_REG_LOAD(3),
	.pll1 = SOR_PLL_TMDS_TERM_ENABLE | SOR_PLL_PE_EN,
	.pe_current = PE_CURRENT0(PE_CURRENT_6_0_mA) |
		PE_CURRENT1(PE_CURRENT_6_0_mA) |
		PE_CURRENT2(PE_CURRENT_6_0_mA) |
		PE_CURRENT3(PE_CURRENT_6_0_mA),
	.drive_current = DRIVE_CURRENT_LANE0(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE1(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE2(DRIVE_CURRENT_7_125_mA) |
		DRIVE_CURRENT_LANE3(DRIVE_CURRENT_7_125_mA),
	},
};
#endif

struct tegra_hdmi_audio_config {
	unsigned pix_clock;
	unsigned n;
	unsigned cts;
	unsigned aval;
};


const struct tegra_hdmi_audio_config tegra_hdmi_audio_32k[] = {
	{25200000,	4096,	25200,	24000},
	{27000000,	4096,	27000,	24000},
	{74250000,	4096,	74250,	24000},
	{148500000,	4096,	148500,	24000},
	{0,		0,	0},
};

const struct tegra_hdmi_audio_config tegra_hdmi_audio_44_1k[] = {
	{25200000,	5880,	26250,	25000},
	{27000000,	5880,	28125,	25000},
	{74250000,	4704,	61875,	20000},
	{148500000,	4704,	123750,	20000},
	{0,		0,	0},
};

const struct tegra_hdmi_audio_config tegra_hdmi_audio_48k[] = {
	{25200000,	6144,	25200,	24000},
	{27000000,	6144,	27000,	24000},
	{74250000,	6144,	74250,	24000},
	{148500000,	6144,	148500,	24000},
	{0,		0,	0},
};

const struct tegra_hdmi_audio_config tegra_hdmi_audio_88_2k[] = {
	{25200000,	11760,	26250,	25000},
	{27000000,	11760,	28125,	25000},
	{74250000,	9408,	61875,	20000},
	{148500000,	9408,	123750, 20000},
	{0,		0,	0},
};

const struct tegra_hdmi_audio_config tegra_hdmi_audio_96k[] = {
	{25200000,	12288,	25200,	24000},
	{27000000,	12288,	27000,	24000},
	{74250000,	12288,	74250,	24000},
	{148500000,	12288,	148500,	24000},
	{0,		0,	0},
};

const struct tegra_hdmi_audio_config tegra_hdmi_audio_176_4k[] = {
	{25200000,	23520,	26250,	25000},
	{27000000,	23520,	28125,	25000},
	{74250000,	18816,	61875,	20000},
	{148500000,	18816,	123750,	20000},
	{0,		0,	0},
};

const struct tegra_hdmi_audio_config tegra_hdmi_audio_192k[] = {
	{25200000,	24576,	25200,	24000},
	{27000000,	24576,	27000,	24000},
	{74250000,	24576,	74250,	24000},
	{148500000,	24576,	148500,	24000},
	{0,		0,	0},
};

static const struct tegra_hdmi_audio_config
*tegra_hdmi_get_audio_config(unsigned audio_freq, unsigned pix_clock)
{
	const struct tegra_hdmi_audio_config *table;

	switch (audio_freq) {
	case AUDIO_FREQ_32K:
		table = tegra_hdmi_audio_32k;
		break;
	case AUDIO_FREQ_44_1K:
		table = tegra_hdmi_audio_44_1k;
		break;
	case AUDIO_FREQ_48K:
		table = tegra_hdmi_audio_48k;
		break;
	case AUDIO_FREQ_88_2K:
		table = tegra_hdmi_audio_88_2k;
		break;
	case AUDIO_FREQ_96K:
		table = tegra_hdmi_audio_96k;
		break;
	case AUDIO_FREQ_176_4K:
		table = tegra_hdmi_audio_176_4k;
		break;
	case AUDIO_FREQ_192K:
		table = tegra_hdmi_audio_192k;
		break;
	default:
		return NULL;
	}

	while (table->pix_clock) {
		if (table->pix_clock == pix_clock)
			return table;
		table++;
	}

	return NULL;
}


unsigned long tegra_hdmi_readl(struct tegra_dc_hdmi_data *hdmi,
					     unsigned long reg)
{
	unsigned long ret;
	ret = readl(hdmi->base + reg * 4);
	trace_printk("readl %p=%#08lx\n", hdmi->base + reg * 4, ret);
	return ret;
}

void tegra_hdmi_writel(struct tegra_dc_hdmi_data *hdmi,
				     unsigned long val, unsigned long reg)
{
	trace_printk("writel %p=%#08lx\n", hdmi->base + reg * 4, val);
	writel(val, hdmi->base + reg * 4);
}

static inline void tegra_hdmi_clrsetbits(struct tegra_dc_hdmi_data *hdmi,
					 unsigned long reg, unsigned long clr,
					 unsigned long set)
{
	unsigned long val = tegra_hdmi_readl(hdmi, reg);
	val &= ~clr;
	val |= set;
	tegra_hdmi_writel(hdmi, val, reg);
}

#ifdef CONFIG_DEBUG_FS
static int dbg_hdmi_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_hdmi_data *hdmi = s->private;

#define DUMP_REG(a) do {						\
		seq_printf(s, "%-32s\t%03x\t%08lx\n",			\
		       #a, a, tegra_hdmi_readl(hdmi, a));		\
	} while (0)

	tegra_dc_io_start(hdmi->dc);
	clk_enable(hdmi->clk);

	DUMP_REG(HDMI_CTXSW);
	DUMP_REG(HDMI_NV_PDISP_SOR_STATE0);
	DUMP_REG(HDMI_NV_PDISP_SOR_STATE1);
	DUMP_REG(HDMI_NV_PDISP_SOR_STATE2);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_AN_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_AN_LSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_CN_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_CN_LSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_AKSV_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_AKSV_LSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_BKSV_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_BKSV_LSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_CKSV_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_CKSV_LSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_DKSV_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_DKSV_LSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_CTRL);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_CMODE);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_MPRIME_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_MPRIME_LSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_SPRIME_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_SPRIME_LSB2);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_SPRIME_LSB1);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_RI);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_CS_MSB);
	DUMP_REG(HDMI_NV_PDISP_RG_HDCP_CS_LSB);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_EMU0);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_EMU_RDATA0);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_EMU1);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_EMU2);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_CTRL);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_STATUS);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_HEADER);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_SUBPACK0_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_SUBPACK0_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_CTRL);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_STATUS);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_HEADER);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_SUBPACK0_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_SUBPACK0_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_SUBPACK1_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_SUBPACK1_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_CTRL);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_STATUS);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_HEADER);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_SUBPACK0_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_SUBPACK0_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_SUBPACK1_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_SUBPACK1_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_SUBPACK2_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_SUBPACK2_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_SUBPACK3_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GENERIC_SUBPACK3_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_CTRL);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0320_SUBPACK_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0320_SUBPACK_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0441_SUBPACK_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0441_SUBPACK_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0882_SUBPACK_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0882_SUBPACK_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_1764_SUBPACK_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_1764_SUBPACK_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0480_SUBPACK_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0480_SUBPACK_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0960_SUBPACK_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_0960_SUBPACK_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_1920_SUBPACK_LOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_ACR_1920_SUBPACK_HIGH);
	DUMP_REG(HDMI_NV_PDISP_HDMI_CTRL);
	DUMP_REG(HDMI_NV_PDISP_HDMI_VSYNC_KEEPOUT);
	DUMP_REG(HDMI_NV_PDISP_HDMI_VSYNC_WINDOW);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GCP_CTRL);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GCP_STATUS);
	DUMP_REG(HDMI_NV_PDISP_HDMI_GCP_SUBPACK);
	DUMP_REG(HDMI_NV_PDISP_HDMI_CHANNEL_STATUS1);
	DUMP_REG(HDMI_NV_PDISP_HDMI_CHANNEL_STATUS2);
	DUMP_REG(HDMI_NV_PDISP_HDMI_EMU0);
	DUMP_REG(HDMI_NV_PDISP_HDMI_EMU1);
	DUMP_REG(HDMI_NV_PDISP_HDMI_EMU1_RDATA);
	DUMP_REG(HDMI_NV_PDISP_HDMI_SPARE);
	DUMP_REG(HDMI_NV_PDISP_HDMI_SPDIF_CHN_STATUS1);
	DUMP_REG(HDMI_NV_PDISP_HDMI_SPDIF_CHN_STATUS2);
	DUMP_REG(HDMI_NV_PDISP_HDCPRIF_ROM_CTRL);
	DUMP_REG(HDMI_NV_PDISP_SOR_CAP);
	DUMP_REG(HDMI_NV_PDISP_SOR_PWR);
	DUMP_REG(HDMI_NV_PDISP_SOR_TEST);
	DUMP_REG(HDMI_NV_PDISP_SOR_PLL0);
	DUMP_REG(HDMI_NV_PDISP_SOR_PLL1);
	DUMP_REG(HDMI_NV_PDISP_SOR_PLL2);
	DUMP_REG(HDMI_NV_PDISP_SOR_CSTM);
	DUMP_REG(HDMI_NV_PDISP_SOR_LVDS);
	DUMP_REG(HDMI_NV_PDISP_SOR_CRCA);
	DUMP_REG(HDMI_NV_PDISP_SOR_CRCB);
	DUMP_REG(HDMI_NV_PDISP_SOR_BLANK);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_CTL);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST0);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST1);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST2);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST3);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST4);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST5);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST6);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST7);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST8);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INST9);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INSTA);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INSTB);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INSTC);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INSTD);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INSTE);
	DUMP_REG(HDMI_NV_PDISP_SOR_SEQ_INSTF);
	DUMP_REG(HDMI_NV_PDISP_SOR_VCRCA0);
	DUMP_REG(HDMI_NV_PDISP_SOR_VCRCA1);
	DUMP_REG(HDMI_NV_PDISP_SOR_CCRCA0);
	DUMP_REG(HDMI_NV_PDISP_SOR_CCRCA1);
	DUMP_REG(HDMI_NV_PDISP_SOR_EDATAA0);
	DUMP_REG(HDMI_NV_PDISP_SOR_EDATAA1);
	DUMP_REG(HDMI_NV_PDISP_SOR_COUNTA0);
	DUMP_REG(HDMI_NV_PDISP_SOR_COUNTA1);
	DUMP_REG(HDMI_NV_PDISP_SOR_DEBUGA0);
	DUMP_REG(HDMI_NV_PDISP_SOR_DEBUGA1);
	DUMP_REG(HDMI_NV_PDISP_SOR_TRIG);
	DUMP_REG(HDMI_NV_PDISP_SOR_MSCHECK);
	DUMP_REG(HDMI_NV_PDISP_SOR_LANE_DRIVE_CURRENT);
	DUMP_REG(HDMI_NV_PDISP_AUDIO_DEBUG0);
	DUMP_REG(HDMI_NV_PDISP_AUDIO_DEBUG1);
	DUMP_REG(HDMI_NV_PDISP_AUDIO_DEBUG2);
	DUMP_REG(HDMI_NV_PDISP_AUDIO_FS(0));
	DUMP_REG(HDMI_NV_PDISP_AUDIO_FS(1));
	DUMP_REG(HDMI_NV_PDISP_AUDIO_FS(2));
	DUMP_REG(HDMI_NV_PDISP_AUDIO_FS(3));
	DUMP_REG(HDMI_NV_PDISP_AUDIO_FS(4));
	DUMP_REG(HDMI_NV_PDISP_AUDIO_FS(5));
	DUMP_REG(HDMI_NV_PDISP_AUDIO_FS(6));
	DUMP_REG(HDMI_NV_PDISP_AUDIO_PULSE_WIDTH);
	DUMP_REG(HDMI_NV_PDISP_AUDIO_THRESHOLD);
	DUMP_REG(HDMI_NV_PDISP_AUDIO_CNTRL0);
	DUMP_REG(HDMI_NV_PDISP_AUDIO_N);
	DUMP_REG(HDMI_NV_PDISP_HDCPRIF_ROM_TIMING);
	DUMP_REG(HDMI_NV_PDISP_SOR_REFCLK);
	DUMP_REG(HDMI_NV_PDISP_CRC_CONTROL);
	DUMP_REG(HDMI_NV_PDISP_INPUT_CONTROL);
	DUMP_REG(HDMI_NV_PDISP_SCRATCH);
	DUMP_REG(HDMI_NV_PDISP_PE_CURRENT);
	DUMP_REG(HDMI_NV_PDISP_KEY_CTRL);
	DUMP_REG(HDMI_NV_PDISP_KEY_DEBUG0);
	DUMP_REG(HDMI_NV_PDISP_KEY_DEBUG1);
	DUMP_REG(HDMI_NV_PDISP_KEY_DEBUG2);
	DUMP_REG(HDMI_NV_PDISP_KEY_HDCP_KEY_0);
	DUMP_REG(HDMI_NV_PDISP_KEY_HDCP_KEY_1);
	DUMP_REG(HDMI_NV_PDISP_KEY_HDCP_KEY_2);
	DUMP_REG(HDMI_NV_PDISP_KEY_HDCP_KEY_3);
	DUMP_REG(HDMI_NV_PDISP_KEY_HDCP_KEY_TRIG);
	DUMP_REG(HDMI_NV_PDISP_KEY_SKEY_INDEX);
#undef DUMP_REG

	clk_disable(hdmi->clk);
	tegra_dc_io_end(hdmi->dc);

	return 0;
}

static int dbg_hdmi_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_hdmi_show, inode->i_private);
}

static const struct file_operations dbg_fops = {
	.open		= dbg_hdmi_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *hdmidir;

static void tegra_dc_hdmi_debug_create(struct tegra_dc_hdmi_data *hdmi)
{
	struct dentry *retval;

	hdmidir = debugfs_create_dir("tegra_hdmi", NULL);
	if (!hdmidir)
		return;
	retval = debugfs_create_file("regs", S_IRUGO, hdmidir, hdmi,
		&dbg_fops);
	if (!retval)
		goto free_out;
	return;
free_out:
	debugfs_remove_recursive(hdmidir);
	hdmidir = NULL;
	return;
}
#else
static inline void tegra_dc_hdmi_debug_create(struct tegra_dc_hdmi_data *hdmi)
{ }
#endif

#define PIXCLOCK_TOLERANCE	200

static int tegra_dc_calc_clock_per_frame(const struct fb_videomode *mode)
{
	return (mode->left_margin + mode->xres +
		mode->right_margin + mode->hsync_len) *
	       (mode->upper_margin + mode->yres +
		mode->lower_margin + mode->vsync_len);
}

static bool tegra_dc_hdmi_valid_pixclock(const struct tegra_dc *dc,
					const struct fb_videomode *mode)
{
	unsigned max_pixclock = tegra_dc_get_out_max_pixclock(dc);
	if (max_pixclock) {
		/* this might look counter-intuitive,
		 * but pixclock's unit is picos(not Khz)
		 */
		return mode->pixclock >= max_pixclock;
	} else {
		return true;
	}
}

static bool tegra_dc_cvt_mode_equal(const struct fb_videomode *mode1,
				const struct fb_videomode *mode2)
{
	return (mode1->xres == mode2->xres &&
		mode1->yres == mode2->yres &&
		mode1->refresh == mode2->refresh &&
		mode1->vmode == mode2->vmode);
}

static bool tegra_dc_reload_mode(struct fb_videomode *mode)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_dc_hdmi_supported_cvt_modes); i++) {
		const struct fb_videomode *cvt_mode
				= &tegra_dc_hdmi_supported_cvt_modes[i];
		if (tegra_dc_cvt_mode_equal(cvt_mode, mode)) {
			memcpy(mode, cvt_mode, sizeof(*mode));
			return true;
		}
	}
	return false;
}

static bool tegra_dc_hdmi_valid_asp_ratio(const struct tegra_dc *dc,
					struct fb_videomode *mode)
{
	int count = 0;
	int m_aspratio = 0;
	int s_aspratio = 0;

	/* To check the aspect upto two decimal digits, calculate in % */
	m_aspratio = (mode->xres*100 / mode->yres);

	if ((m_aspratio < TEGRA_DC_HDMI_MIN_ASPECT_RATIO_PERCENT) ||
			(m_aspratio > TEGRA_DC_HDMI_MAX_ASPECT_RATIO_PERCENT))
				return false;

	/* Check from the table of  supported aspect ratios, allow
	    difference of 1% for second decimal digit calibration */
	for (count = 0; count < ARRAY_SIZE(tegra_dc_hdmi_aspect_ratios);
		 count++) {
			s_aspratio =  tegra_dc_hdmi_aspect_ratios[count];
			if ((m_aspratio == s_aspratio) ||
				(abs(m_aspratio - s_aspratio) == 1))
				return true;
	}

	return false;
}


static bool tegra_dc_hdmi_mode_filter(const struct tegra_dc *dc,
					struct fb_videomode *mode)
{
	if (mode->vmode & FB_VMODE_INTERLACED)
		return false;

	/* Ignore modes with a 0 pixel clock */
	if (!mode->pixclock)
		return false;

#ifdef CONFIG_TEGRA_HDMI_74MHZ_LIMIT
		if (PICOS2KHZ(mode->pixclock) > 74250)
			return false;
#endif

	/* Check if the mode's pixel clock is more than the max rate*/
	if (!tegra_dc_hdmi_valid_pixclock(dc, mode))
		return false;

	/* Check if the mode's aspect ratio is supported */
	if (!tegra_dc_hdmi_valid_asp_ratio(dc, mode))
		return false;

	/* Check some of DC's constraints */
	if (mode->hsync_len > 1 && mode->vsync_len > 1 &&
		mode->lower_margin + mode->vsync_len + mode->upper_margin > 1 &&
		mode->xres >= 16 && mode->yres >= 16) {

		if (mode->lower_margin == 1) {
			/* This might be the case for HDMI<->DVI
			 * where std VESA representation will not
			 * pass constraint V_FRONT_PORCH >=
			 * V_REF_TO_SYNC + 1.So reload mode in
			 * CVT timing standards.
			 */
			if (!tegra_dc_reload_mode(mode))
				return false;
		}
		mode->flag = FB_MODE_IS_DETAILED;
		mode->refresh = (PICOS2KHZ(mode->pixclock) * 1000) /
				tegra_dc_calc_clock_per_frame(mode);
		return true;
	}

	return false;
}

static bool tegra_dc_hdmi_hpd(struct tegra_dc *dc)
{
	return tegra_dc_hpd(dc);
}


void tegra_dc_hdmi_detect_config(struct tegra_dc *dc,
						struct fb_monspecs *specs)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);

	/* monitors like to lie about these but they are still useful for
	 * detecting aspect ratios
	 */
	dc->out->h_size = specs->max_x * 1000;
	dc->out->v_size = specs->max_y * 1000;

	hdmi->dvi = !(specs->misc & FB_MISC_HDMI);

	tegra_fb_update_monspecs(dc->fb, specs, tegra_dc_hdmi_mode_filter);
#ifdef CONFIG_SWITCH
	hdmi->hpd_switch.state = 0;
	switch_set_state(&hdmi->hpd_switch, 1);
#endif
	dev_info(&dc->ndev->dev, "display detected\n");

	dc->connected = true;
	tegra_dc_ext_process_hotplug(dc->ndev->id);
}

/* This function is used to enable DC1 and HDMI for the purpose of testing. */
bool tegra_dc_hdmi_detect_test(struct tegra_dc *dc, unsigned char *edid_ptr)
{
	int err;
	struct fb_monspecs specs;
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);

	if (!hdmi || !edid_ptr) {
		dev_err(&dc->ndev->dev, "HDMI test failed to get arguments.\n");
		return false;
	}

	err = tegra_edid_get_monspecs_test(hdmi->edid, &specs, edid_ptr);
	if (err < 0) {
		/* Check if there's a hard-wired mode, if so, enable it */
		if (dc->out->n_modes)
			tegra_dc_enable(dc);
		else {
			dev_err(&dc->ndev->dev, "error reading edid\n");
			goto fail;
		}
#ifdef CONFIG_SWITCH
		hdmi->hpd_switch.state = 0;
		switch_set_state(&hdmi->hpd_switch, 1);
#endif
		dev_info(&dc->ndev->dev, "display detected\n");

		dc->connected = true;
		tegra_dc_ext_process_hotplug(dc->ndev->id);
	} else {
		err = tegra_edid_get_eld(hdmi->edid, &hdmi->eld);
		if (err < 0) {
			dev_err(&dc->ndev->dev, "error populating eld\n");
			goto fail;
		}
		hdmi->eld_retrieved = true;

		tegra_dc_hdmi_detect_config(dc, &specs);
	}

	return true;

fail:
	hdmi->eld_retrieved = false;
#ifdef CONFIG_SWITCH
	switch_set_state(&hdmi->hpd_switch, 0);
#endif
	tegra_nvhdcp_set_plug(hdmi->nvhdcp, 0);
	return false;
}
EXPORT_SYMBOL(tegra_dc_hdmi_detect_test);

static bool tegra_dc_hdmi_detect(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	struct fb_monspecs specs;
	int err;

	if (!tegra_dc_hdmi_hpd(dc))
		goto fail;

	err = tegra_edid_get_monspecs(hdmi->edid, &specs);
	if (err < 0) {
		if (dc->out->n_modes)
			tegra_dc_enable(dc);
		else {
			dev_err(&dc->ndev->dev, "error reading edid\n");
			goto fail;
		}
#ifdef CONFIG_SWITCH
		hdmi->hpd_switch.state = 0;
		switch_set_state(&hdmi->hpd_switch, 1);
#endif
		dev_info(&dc->ndev->dev, "display detected\n");

		dc->connected = true;
		tegra_dc_ext_process_hotplug(dc->ndev->id);
	} else {
		err = tegra_edid_get_eld(hdmi->edid, &hdmi->eld);
		if (err < 0) {
			dev_err(&dc->ndev->dev, "error populating eld\n");
			goto fail;
		}
		hdmi->eld_retrieved = true;

		tegra_dc_hdmi_detect_config(dc, &specs);
	}

	return true;

fail:
	hdmi->eld_retrieved = false;
#ifdef CONFIG_SWITCH
	switch_set_state(&hdmi->hpd_switch, 0);
#endif
	tegra_nvhdcp_set_plug(hdmi->nvhdcp, 0);
	return false;
}


static void tegra_dc_hdmi_detect_worker(struct work_struct *work)
{
	struct tegra_dc_hdmi_data *hdmi =
		container_of(to_delayed_work(work), struct tegra_dc_hdmi_data, work);
	struct tegra_dc *dc = hdmi->dc;

	tegra_dc_enable(dc);
	msleep(5);
	if (!tegra_dc_hdmi_detect(dc)) {
		tegra_dc_disable(dc);
		tegra_fb_update_monspecs(dc->fb, NULL, NULL);

		dc->connected = false;
		tegra_dc_ext_process_hotplug(dc->ndev->id);
	}
}

static irqreturn_t tegra_dc_hdmi_irq(int irq, void *ptr)
{
	struct tegra_dc *dc = ptr;
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	unsigned long flags;

	spin_lock_irqsave(&hdmi->suspend_lock, flags);
	if (!hdmi->suspended) {
		__cancel_delayed_work(&hdmi->work);
		if (tegra_dc_hdmi_hpd(dc))
			queue_delayed_work(system_nrt_wq, &hdmi->work,
					   msecs_to_jiffies(100));
		else
			queue_delayed_work(system_nrt_wq, &hdmi->work,
					   msecs_to_jiffies(30));
	}
	spin_unlock_irqrestore(&hdmi->suspend_lock, flags);

	return IRQ_HANDLED;
}

static void tegra_dc_hdmi_suspend(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	unsigned long flags;

	tegra_nvhdcp_suspend(hdmi->nvhdcp);
	spin_lock_irqsave(&hdmi->suspend_lock, flags);
	hdmi->suspended = true;
	spin_unlock_irqrestore(&hdmi->suspend_lock, flags);
}

static void tegra_dc_hdmi_resume(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	unsigned long flags;

	spin_lock_irqsave(&hdmi->suspend_lock, flags);
	hdmi->suspended = false;

	if (tegra_dc_hdmi_hpd(dc))
		queue_delayed_work(system_nrt_wq, &hdmi->work,
				   msecs_to_jiffies(100));
	else
		queue_delayed_work(system_nrt_wq, &hdmi->work,
				   msecs_to_jiffies(30));

	spin_unlock_irqrestore(&hdmi->suspend_lock, flags);
	tegra_nvhdcp_resume(hdmi->nvhdcp);
}

#ifdef CONFIG_SWITCH
static ssize_t underscan_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tegra_dc_hdmi_data *hdmi =
			container_of(dev_get_drvdata(dev), struct tegra_dc_hdmi_data, hpd_switch);

	if (hdmi->edid)
		return sprintf(buf, "%d\n", tegra_edid_underscan_supported(hdmi->edid));
	else
		return 0;
}

static DEVICE_ATTR(underscan, S_IRUGO | S_IWUSR, underscan_show, NULL);
#endif

static int tegra_dc_hdmi_init(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi;
	struct resource *res;
	struct resource *base_res;
#ifdef CONFIG_SWITCH
	int ret;
#endif
	void __iomem *base;
	struct clk *clk = NULL;
	struct clk *disp1_clk = NULL;
	struct clk *disp2_clk = NULL;
	int err;

	hdmi = kzalloc(sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	res = nvhost_get_resource_byname(dc->ndev, IORESOURCE_MEM, "hdmi_regs");
	if (!res) {
		dev_err(&dc->ndev->dev, "hdmi: no mem resource\n");
		err = -ENOENT;
		goto err_free_hdmi;
	}

	base_res = request_mem_region(res->start, resource_size(res), dc->ndev->name);
	if (!base_res) {
		dev_err(&dc->ndev->dev, "hdmi: request_mem_region failed\n");
		err = -EBUSY;
		goto err_free_hdmi;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&dc->ndev->dev, "hdmi: registers can't be mapped\n");
		err = -EBUSY;
		goto err_release_resource_reg;
	}

	clk = clk_get(&dc->ndev->dev, "hdmi");
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&dc->ndev->dev, "hdmi: can't get clock\n");
		err = -ENOENT;
		goto err_iounmap_reg;
	}

	disp1_clk = clk_get_sys("tegradc.0", NULL);
	if (IS_ERR_OR_NULL(disp1_clk)) {
		dev_err(&dc->ndev->dev, "hdmi: can't disp1 clock\n");
		err = -ENOENT;
		goto err_put_clock;
	}

	disp2_clk = clk_get_sys("tegradc.1", NULL);
	if (IS_ERR_OR_NULL(disp2_clk)) {
		dev_err(&dc->ndev->dev, "hdmi: can't disp2 clock\n");
		err = -ENOENT;
		goto err_put_clock;
	}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	hdmi->hda_clk = clk_get_sys("tegra30-hda", "hda");
	if (IS_ERR_OR_NULL(hdmi->hda_clk)) {
		dev_err(&dc->ndev->dev, "hdmi: can't get hda clock\n");
		err = -ENOENT;
		goto err_put_clock;
	}

	hdmi->hda2codec_clk = clk_get_sys("tegra30-hda", "hda2codec");
	if (IS_ERR_OR_NULL(hdmi->hda2codec_clk)) {
		dev_err(&dc->ndev->dev, "hdmi: can't get hda2codec clock\n");
		err = -ENOENT;
		goto err_put_clock;
	}

	hdmi->hda2hdmi_clk = clk_get_sys("tegra30-hda", "hda2hdmi");
	if (IS_ERR_OR_NULL(hdmi->hda2hdmi_clk)) {
		dev_err(&dc->ndev->dev, "hdmi: can't get hda2hdmi clock\n");
		err = -ENOENT;
		goto err_put_clock;
	}
#endif

	/* TODO: support non-hotplug */
	if (request_irq(gpio_to_irq(dc->out->hotplug_gpio), tegra_dc_hdmi_irq,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			dev_name(&dc->ndev->dev), dc)) {
		dev_err(&dc->ndev->dev, "hdmi: request_irq %d failed\n",
			gpio_to_irq(dc->out->hotplug_gpio));
		err = -EBUSY;
		goto err_put_clock;
	}

	hdmi->edid = tegra_edid_create(dc->out->dcc_bus);
	if (IS_ERR_OR_NULL(hdmi->edid)) {
		dev_err(&dc->ndev->dev, "hdmi: can't create edid\n");
		err = PTR_ERR(hdmi->edid);
		goto err_free_irq;
	}

#ifdef CONFIG_TEGRA_NVHDCP
	hdmi->nvhdcp = tegra_nvhdcp_create(hdmi, dc->ndev->id,
			dc->out->dcc_bus);
	if (IS_ERR_OR_NULL(hdmi->nvhdcp)) {
		dev_err(&dc->ndev->dev, "hdmi: can't create nvhdcp\n");
		err = PTR_ERR(hdmi->nvhdcp);
		goto err_edid_destroy;
	}
#else
	hdmi->nvhdcp = NULL;
#endif

	INIT_DELAYED_WORK(&hdmi->work, tegra_dc_hdmi_detect_worker);

	hdmi->dc = dc;
	hdmi->base = base;
	hdmi->base_res = base_res;
	hdmi->clk = clk;
	hdmi->disp1_clk = disp1_clk;
	hdmi->disp2_clk = disp2_clk;
	hdmi->suspended = false;
	hdmi->eld_retrieved= false;
	hdmi->clk_enabled = false;
	hdmi->audio_freq = 44100;
	hdmi->audio_source = AUTO;
	spin_lock_init(&hdmi->suspend_lock);

#ifdef CONFIG_SWITCH
	hdmi->hpd_switch.name = "hdmi";
	ret = switch_dev_register(&hdmi->hpd_switch);

	if (!ret)
		ret = device_create_file(hdmi->hpd_switch.dev,
			&dev_attr_underscan);
	BUG_ON(ret != 0);
#endif

	dc->out->depth = 24;

	tegra_dc_set_outdata(dc, hdmi);

	dc_hdmi = hdmi;
	/* boards can select default content protection policy */
	if (dc->out->flags & TEGRA_DC_OUT_NVHDCP_POLICY_ON_DEMAND)
		tegra_nvhdcp_set_policy(hdmi->nvhdcp,
			TEGRA_NVHDCP_POLICY_ON_DEMAND);
	else
		tegra_nvhdcp_set_policy(hdmi->nvhdcp,
			TEGRA_NVHDCP_POLICY_ALWAYS_ON);

	tegra_dc_hdmi_debug_create(hdmi);

	return 0;

#ifdef CONFIG_TEGRA_NVHDCP
err_edid_destroy:
	tegra_edid_destroy(hdmi->edid);
#endif
err_free_irq:
	free_irq(gpio_to_irq(dc->out->hotplug_gpio), dc);
err_put_clock:
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	if (!IS_ERR_OR_NULL(hdmi->hda2hdmi_clk))
		clk_put(hdmi->hda2hdmi_clk);
	if (!IS_ERR_OR_NULL(hdmi->hda2codec_clk))
		clk_put(hdmi->hda2codec_clk);
	if (!IS_ERR_OR_NULL(hdmi->hda_clk))
		clk_put(hdmi->hda_clk);
#endif
	if (!IS_ERR_OR_NULL(disp2_clk))
		clk_put(disp2_clk);
	if (!IS_ERR_OR_NULL(disp1_clk))
		clk_put(disp1_clk);
	if (!IS_ERR_OR_NULL(clk))
		clk_put(clk);
err_iounmap_reg:
	iounmap(base);
err_release_resource_reg:
	release_resource(base_res);
err_free_hdmi:
	kfree(hdmi);
	return err;
}

static void tegra_dc_hdmi_destroy(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);

	free_irq(gpio_to_irq(dc->out->hotplug_gpio), dc);
	cancel_delayed_work_sync(&hdmi->work);
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&hdmi->hpd_switch);
#endif
	iounmap(hdmi->base);
	release_resource(hdmi->base_res);
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	clk_put(hdmi->hda2hdmi_clk);
	clk_put(hdmi->hda2codec_clk);
	clk_put(hdmi->hda_clk);
#endif
	clk_put(hdmi->clk);
	clk_put(hdmi->disp1_clk);
	clk_put(hdmi->disp2_clk);
	tegra_edid_destroy(hdmi->edid);
	tegra_nvhdcp_destroy(hdmi->nvhdcp);

	kfree(hdmi);

}

static void tegra_dc_hdmi_setup_audio_fs_tables(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	int i;
	unsigned freqs[] = {
		32000,
		44100,
		48000,
		88200,
		96000,
		176400,
		192000,
        };

	for (i = 0; i < ARRAY_SIZE(freqs); i++) {
		unsigned f = freqs[i];
		unsigned eight_half;
		unsigned delta;;

		if (f > 96000)
			delta = 2;
		else if (f > 48000)
			delta = 6;
		else
			delta = 9;

		eight_half = (8 * HDMI_AUDIOCLK_FREQ) / (f * 128);
		tegra_hdmi_writel(hdmi, AUDIO_FS_LOW(eight_half - delta) |
				  AUDIO_FS_HIGH(eight_half + delta),
				  HDMI_NV_PDISP_AUDIO_FS(i));
	}
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
static void tegra_dc_hdmi_setup_eld_buff(struct tegra_dc *dc)
{
	int i;
	int j;
	u8 tmp;

	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);

	/* program ELD stuff */
	for (i = 0; i < HDMI_ELD_MONITOR_NAME_INDEX; i++) {
		switch (i) {
		case HDMI_ELD_VER_INDEX:
			tmp = (hdmi->eld.eld_ver << 3);
			tegra_hdmi_writel(hdmi, (i << 8) | tmp,
				  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
			break;
		case HDMI_ELD_BASELINE_LEN_INDEX:
			break;
		case HDMI_ELD_CEA_VER_MNL_INDEX:
			tmp = (hdmi->eld.cea_edid_ver << 5);
			tmp |= (hdmi->eld.mnl & 0x1f);
			tegra_hdmi_writel(hdmi, (i << 8) | tmp,
					  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
			break;
		case HDMI_ELD_SAD_CNT_CON_TYP_SAI_HDCP_INDEX:
			tmp = (hdmi->eld.sad_count << 4);
			tmp |= (hdmi->eld.conn_type & 0xC);
			tmp |= (hdmi->eld.support_ai & 0x2);
			tmp |= (hdmi->eld.support_hdcp & 0x1);
			tegra_hdmi_writel(hdmi, (i << 8) | tmp,
					  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
			break;
		case HDMI_ELD_AUD_SYNC_DELAY_INDEX:
			tegra_hdmi_writel(hdmi, (i << 8) | (hdmi->eld.aud_synch_delay),
					  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
			break;
		case HDMI_ELD_SPK_ALLOC_INDEX:
			tegra_hdmi_writel(hdmi, (i << 8) | (hdmi->eld.spk_alloc),
					  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
			break;
		case HDMI_ELD_PORT_ID_INDEX:
			for (j = 0; j < 8;j++) {
				tegra_hdmi_writel(hdmi, ((i +j) << 8) | (hdmi->eld.port_id[j]),
					  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
			}
			break;
		case HDMI_ELD_MANF_NAME_INDEX:
			for (j = 0; j < 2;j++) {
				tegra_hdmi_writel(hdmi, ((i +j) << 8) | (hdmi->eld.manufacture_id[j]),
					  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
			}
			break;
		case HDMI_ELD_PRODUCT_CODE_INDEX:
			for (j = 0; j < 2;j++) {
				tegra_hdmi_writel(hdmi, ((i +j) << 8) | (hdmi->eld.product_id[j]),
					  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
			}
			break;
		}
	}
	for (j = 0; j < hdmi->eld.mnl;j++) {
		tegra_hdmi_writel(hdmi, ((j + HDMI_ELD_MONITOR_NAME_INDEX) << 8) |
				  (hdmi->eld.monitor_name[j]),
				  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
	}
	for (j = 0; j < hdmi->eld.sad_count;j++) {
		tegra_hdmi_writel(hdmi, ((j + HDMI_ELD_MONITOR_NAME_INDEX + hdmi->eld.mnl) << 8) |
				  (hdmi->eld.sad[j]),
				  HDMI_NV_PDISP_SOR_AUDIO_HDA_ELD_BUFWR_0);
	}
		/* set presence andvalid bit  */
	tegra_hdmi_writel(hdmi, 3, HDMI_NV_PDISP_SOR_AUDIO_HDA_PRESENSE_0);
}
#endif

static int tegra_dc_hdmi_setup_audio(struct tegra_dc *dc, unsigned audio_freq,
					unsigned audio_source)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	const struct tegra_hdmi_audio_config *config;
	unsigned long audio_n;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	unsigned long reg_addr = 0;
#endif
	unsigned a_source = AUDIO_CNTRL0_SOURCE_SELECT_AUTO;

	if (HDA == audio_source)
		a_source = AUDIO_CNTRL0_SOURCE_SELECT_HDAL;
	else if (SPDIF == audio_source)
		a_source = AUDIO_CNTRL0_SOURCE_SELECT_SPDIF;

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	if (hdmi->audio_inject_null)
		a_source |= AUDIO_CNTRL0_INJECT_NULLSMPL;

	tegra_hdmi_writel(hdmi,a_source,
			  HDMI_NV_PDISP_SOR_AUDIO_CNTRL0_0);
	tegra_hdmi_writel(hdmi,
			  AUDIO_CNTRL0_ERROR_TOLERANCE(6) |
			  AUDIO_CNTRL0_FRAMES_PER_BLOCK(0xc0),
			  HDMI_NV_PDISP_AUDIO_CNTRL0);
#else
	tegra_hdmi_writel(hdmi,
			  AUDIO_CNTRL0_ERROR_TOLERANCE(6) |
			  AUDIO_CNTRL0_FRAMES_PER_BLOCK(0xc0) |
			  a_source,
			  HDMI_NV_PDISP_AUDIO_CNTRL0);
#endif
	config = tegra_hdmi_get_audio_config(audio_freq, dc->mode.pclk);
	if (!config) {
		dev_err(&dc->ndev->dev,
			"hdmi: can't set audio to %d at %d pix_clock",
			audio_freq, dc->mode.pclk);
		return -EINVAL;
	}

	tegra_hdmi_writel(hdmi, 0, HDMI_NV_PDISP_HDMI_ACR_CTRL);

	audio_n = AUDIO_N_RESETF | AUDIO_N_GENERATE_ALTERNALTE |
		AUDIO_N_VALUE(config->n - 1);
	tegra_hdmi_writel(hdmi, audio_n, HDMI_NV_PDISP_AUDIO_N);

	tegra_hdmi_writel(hdmi, ACR_SUBPACK_N(config->n) | ACR_ENABLE,
			  HDMI_NV_PDISP_HDMI_ACR_0441_SUBPACK_HIGH);

	tegra_hdmi_writel(hdmi, ACR_SUBPACK_CTS(config->cts),
			  HDMI_NV_PDISP_HDMI_ACR_0441_SUBPACK_LOW);

	tegra_hdmi_writel(hdmi, SPARE_HW_CTS | SPARE_FORCE_SW_CTS |
			  SPARE_CTS_RESET_VAL(1),
			  HDMI_NV_PDISP_HDMI_SPARE);

	audio_n &= ~AUDIO_N_RESETF;
	tegra_hdmi_writel(hdmi, audio_n, HDMI_NV_PDISP_AUDIO_N);

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	switch (audio_freq) {
	case AUDIO_FREQ_32K:
		reg_addr = HDMI_NV_PDISP_SOR_AUDIO_AVAL_0320_0;
		break;
	case AUDIO_FREQ_44_1K:
		reg_addr = HDMI_NV_PDISP_SOR_AUDIO_AVAL_0441_0;
		break;
	case AUDIO_FREQ_48K:
		reg_addr = HDMI_NV_PDISP_SOR_AUDIO_AVAL_0480_0;
		break;
	case AUDIO_FREQ_88_2K:
		reg_addr = HDMI_NV_PDISP_SOR_AUDIO_AVAL_0882_0;
		break;
	case AUDIO_FREQ_96K:
		reg_addr = HDMI_NV_PDISP_SOR_AUDIO_AVAL_0960_0;
		break;
	case AUDIO_FREQ_176_4K:
		reg_addr = HDMI_NV_PDISP_SOR_AUDIO_AVAL_1764_0;
		break;
	case AUDIO_FREQ_192K:
		reg_addr = HDMI_NV_PDISP_SOR_AUDIO_AVAL_1920_0;
		break;
	}

	tegra_hdmi_writel(hdmi, config->aval, reg_addr);
#endif
	tegra_dc_hdmi_setup_audio_fs_tables(dc);

	return 0;
}

int tegra_hdmi_setup_audio_freq_source(unsigned audio_freq, unsigned audio_source)
{
	struct tegra_dc_hdmi_data *hdmi = dc_hdmi;

	if (!hdmi)
		return -EAGAIN;

	/* check for know freq */
	if (AUDIO_FREQ_32K == audio_freq ||
		AUDIO_FREQ_44_1K== audio_freq ||
		AUDIO_FREQ_48K== audio_freq ||
		AUDIO_FREQ_88_2K== audio_freq ||
		AUDIO_FREQ_96K== audio_freq ||
		AUDIO_FREQ_176_4K== audio_freq ||
		AUDIO_FREQ_192K== audio_freq) {
		/* If we can program HDMI, then proceed */
		if (hdmi->clk_enabled)
			tegra_dc_hdmi_setup_audio(hdmi->dc, audio_freq,audio_source);

		/* Store it for using it in enable */
		hdmi->audio_freq = audio_freq;
		hdmi->audio_source = audio_source;
	}
	else
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(tegra_hdmi_setup_audio_freq_source);

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
int tegra_hdmi_audio_null_sample_inject(bool on)
{
	struct tegra_dc_hdmi_data *hdmi = dc_hdmi;
	unsigned int val = 0;

	if (!hdmi)
		return -EAGAIN;

	if (hdmi->audio_inject_null != on) {
		hdmi->audio_inject_null = on;
		if (hdmi->clk_enabled) {
			val = tegra_hdmi_readl(hdmi,
				HDMI_NV_PDISP_SOR_AUDIO_CNTRL0_0);
			val &= ~AUDIO_CNTRL0_INJECT_NULLSMPL;
			if (on)
				val |= AUDIO_CNTRL0_INJECT_NULLSMPL;
			tegra_hdmi_writel(hdmi,val,
				HDMI_NV_PDISP_SOR_AUDIO_CNTRL0_0);
		}
	}

	return 0;
}
EXPORT_SYMBOL(tegra_hdmi_audio_null_sample_inject);

int tegra_hdmi_setup_hda_presence()
{
	struct tegra_dc_hdmi_data *hdmi = dc_hdmi;

	if (!hdmi)
		return -EAGAIN;

	if (hdmi->clk_enabled && hdmi->eld_retrieved) {
		/* If HDA_PRESENCE is already set reset it */
		if (tegra_hdmi_readl(hdmi,
				     HDMI_NV_PDISP_SOR_AUDIO_HDA_PRESENSE_0))
			tegra_hdmi_writel(hdmi, 0,
				     HDMI_NV_PDISP_SOR_AUDIO_HDA_PRESENSE_0);

		tegra_dc_hdmi_setup_eld_buff(hdmi->dc);
	}
	else
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL(tegra_hdmi_setup_hda_presence);
#endif

static void tegra_dc_hdmi_write_infopack(struct tegra_dc *dc, int header_reg,
					 u8 type, u8 version, void *data, int len)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	u32 subpack[2];  /* extra byte for zero padding of subpack */
	int i;
	u8 csum;

	/* first byte of data is the checksum */
	csum = type + version + len - 1;
	for (i = 1; i < len; i++)
		csum +=((u8 *)data)[i];
	((u8 *)data)[0] = 0x100 - csum;

	tegra_hdmi_writel(hdmi, INFOFRAME_HEADER_TYPE(type) |
			  INFOFRAME_HEADER_VERSION(version) |
			  INFOFRAME_HEADER_LEN(len - 1),
			  header_reg);

	/* The audio inforame only has one set of subpack registers.  The hdmi
	 * block pads the rest of the data as per the spec so we have to fixup
	 * the length before filling in the subpacks.
	 */
	if (header_reg == HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_HEADER)
		len = 6;

	/* each subpack 7 bytes devided into:
	 *   subpack_low - bytes 0 - 3
	 *   subpack_high - bytes 4 - 6 (with byte 7 padded to 0x00)
	 */
	for (i = 0; i < len; i++) {
		int subpack_idx = i % 7;

		if (subpack_idx == 0)
			memset(subpack, 0x0, sizeof(subpack));

		((u8 *)subpack)[subpack_idx] = ((u8 *)data)[i];

		if (subpack_idx == 6 || (i + 1 == len)) {
			int reg = header_reg + 1 + (i / 7) * 2;

			tegra_hdmi_writel(hdmi, subpack[0], reg);
			tegra_hdmi_writel(hdmi, subpack[1], reg + 1);
		}
	}
}

static void tegra_dc_hdmi_setup_avi_infoframe(struct tegra_dc *dc, bool dvi)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	struct hdmi_avi_infoframe avi;

	if (dvi) {
		tegra_hdmi_writel(hdmi, 0x0,
				  HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_CTRL);
		return;
	}

	memset(&avi, 0x0, sizeof(avi));

	avi.r = HDMI_AVI_R_SAME;

	if ((dc->mode.h_active == 720) && ((dc->mode.v_active == 480) || (dc->mode.v_active == 576)))
		tegra_dc_writel(dc, 0x00101010, DC_DISP_BORDER_COLOR);
	else
		tegra_dc_writel(dc, 0x00000000, DC_DISP_BORDER_COLOR);

	if (dc->mode.v_active == 480) {
		if (dc->mode.h_active == 640) {
			avi.m = HDMI_AVI_M_4_3;
			avi.vic = 1;
		} else {
			avi.m = HDMI_AVI_M_16_9;
			avi.vic = 3;
		}
	} else if (dc->mode.v_active == 576) {
		/* CEC modes 17 and 18 differ only by the pysical size of the
		 * screen so we have to calculation the physical aspect
		 * ratio.  4 * 10 / 3  is 13
		 */
		if ((dc->out->h_size * 10) / dc->out->v_size > 14) {
			avi.m = HDMI_AVI_M_16_9;
			avi.vic = 18;
		} else {
			avi.m = HDMI_AVI_M_4_3;
			avi.vic = 17;
		}
	} else if (dc->mode.v_active == 720 ||
		(dc->mode.v_active == 1470 && dc->mode.stereo_mode)) {
		/* VIC for both 720p and 720p 3D mode */
		avi.m = HDMI_AVI_M_16_9;
		if (dc->mode.h_front_porch == 110)
			avi.vic = 4; /* 60 Hz */
		else
			avi.vic = 19; /* 50 Hz */
	} else if (dc->mode.v_active == 1080 ||
		(dc->mode.v_active == 2205 && dc->mode.stereo_mode)) {
		/* VIC for both 1080p and 1080p 3D mode */
		avi.m = HDMI_AVI_M_16_9;
		if (dc->mode.h_front_porch == 88) {
			if (dc->mode.pclk > 74250000)
				avi.vic = 16; /* 60 Hz */
			else
				avi.vic = 34; /* 30 Hz */
		} else if (dc->mode.h_front_porch == 528)
			avi.vic = 31; /* 50 Hz */
		else
			avi.vic = 32; /* 24 Hz */
	} else {
		avi.m = HDMI_AVI_M_16_9;
		avi.vic = 0;
	}


	tegra_dc_hdmi_write_infopack(dc, HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_HEADER,
				     HDMI_INFOFRAME_TYPE_AVI,
				     HDMI_AVI_VERSION,
				     &avi, sizeof(avi));

	tegra_hdmi_writel(hdmi, INFOFRAME_CTRL_ENABLE,
			  HDMI_NV_PDISP_HDMI_AVI_INFOFRAME_CTRL);
}

static void tegra_dc_hdmi_setup_stereo_infoframe(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	struct hdmi_stereo_infoframe stereo;
	u32 val;

	if (!dc->mode.stereo_mode) {
		val  = tegra_hdmi_readl(hdmi, HDMI_NV_PDISP_HDMI_GENERIC_CTRL);
		val &= ~GENERIC_CTRL_ENABLE;
		tegra_hdmi_writel(hdmi, val, HDMI_NV_PDISP_HDMI_GENERIC_CTRL);
		return;
	}

	memset(&stereo, 0x0, sizeof(stereo));

	stereo.regid0 = 0x03;
	stereo.regid1 = 0x0c;
	stereo.regid2 = 0x00;
	stereo.hdmi_video_format = 2; /* 3D_Structure present */
#ifndef CONFIG_TEGRA_HDMI_74MHZ_LIMIT
	stereo._3d_structure = 0; /* frame packing */
#else
	stereo._3d_structure = 8; /* side-by-side (half) */
	stereo._3d_ext_data = 0; /* something which fits into 00XX bit req */
#endif

	tegra_dc_hdmi_write_infopack(dc, HDMI_NV_PDISP_HDMI_GENERIC_HEADER,
					HDMI_INFOFRAME_TYPE_VENDOR,
					HDMI_VENDOR_VERSION,
					&stereo, 6);

	val  = tegra_hdmi_readl(hdmi, HDMI_NV_PDISP_HDMI_GENERIC_CTRL);
	val |= GENERIC_CTRL_ENABLE;

	tegra_hdmi_writel(hdmi, val, HDMI_NV_PDISP_HDMI_GENERIC_CTRL);
}

static void tegra_dc_hdmi_setup_audio_infoframe(struct tegra_dc *dc, bool dvi)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	struct hdmi_audio_infoframe audio;

	if (dvi) {
		tegra_hdmi_writel(hdmi, 0x0,
				  HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_CTRL);
		return;
	}

	memset(&audio, 0x0, sizeof(audio));

	audio.cc = HDMI_AUDIO_CC_2;
	tegra_dc_hdmi_write_infopack(dc, HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_HEADER,
				     HDMI_INFOFRAME_TYPE_AUDIO,
				     HDMI_AUDIO_VERSION,
				     &audio, sizeof(audio));

	tegra_hdmi_writel(hdmi, INFOFRAME_CTRL_ENABLE,
			  HDMI_NV_PDISP_HDMI_AUDIO_INFOFRAME_CTRL);
}

static void tegra_dc_hdmi_setup_tdms(struct tegra_dc_hdmi_data *hdmi,
		const struct tdms_config *tc)
{
	tegra_hdmi_writel(hdmi, tc->pll0, HDMI_NV_PDISP_SOR_PLL0);
	tegra_hdmi_writel(hdmi, tc->pll1, HDMI_NV_PDISP_SOR_PLL1);

	tegra_hdmi_writel(hdmi, tc->pe_current, HDMI_NV_PDISP_PE_CURRENT);

	tegra_hdmi_writel(hdmi,
			  tc->drive_current | DRIVE_CURRENT_FUSE_OVERRIDE,
			  HDMI_NV_PDISP_SOR_LANE_DRIVE_CURRENT);
}

static void tegra_dc_hdmi_enable(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
	int pulse_start;
	int dispclk_div_8_2;
	int retries;
	int rekey;
	int err;
	unsigned long val;
	unsigned i;
	unsigned long oldrate;

	/* enbale power, clocks, resets, etc. */

	/* The upstream DC needs to be clocked for accesses to HDMI to not
	 * hard lock the system.  Because we don't know if HDMI is conencted
	 * to disp1 or disp2 we need to enable both until we set the DC mux.
	 */
	clk_enable(hdmi->disp1_clk);
	clk_enable(hdmi->disp2_clk);

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	/* Enabling HDA clocks before asserting HDA PD and ELDV bits */
	clk_enable(hdmi->hda_clk);
	clk_enable(hdmi->hda2codec_clk);
	clk_enable(hdmi->hda2hdmi_clk);
#endif

	/* back off multiplier before attaching to parent at new rate. */
	oldrate = clk_get_rate(hdmi->clk);
	clk_set_rate(hdmi->clk, oldrate / 2);

	tegra_dc_setup_clk(dc, hdmi->clk);
	clk_set_rate(hdmi->clk, dc->mode.pclk);

	clk_enable(hdmi->clk);
	tegra_periph_reset_assert(hdmi->clk);
	mdelay(1);
	tegra_periph_reset_deassert(hdmi->clk);

	/* TODO: copy HDCP keys from KFUSE to HDMI */

	/* Program display timing registers: handled by dc */

	/* program HDMI registers and SOR sequencer */

	tegra_dc_writel(dc, VSYNC_H_POSITION(1), DC_DISP_DISP_TIMING_OPTIONS);
	tegra_dc_writel(dc, DITHER_CONTROL_DISABLE | BASE_COLOR_SIZE888,
			DC_DISP_DISP_COLOR_CONTROL);

	/* video_preamble uses h_pulse2 */
	pulse_start = dc->mode.h_ref_to_sync + dc->mode.h_sync_width +
		dc->mode.h_back_porch - 10;
	tegra_dc_writel(dc, H_PULSE_2_ENABLE, DC_DISP_DISP_SIGNAL_OPTIONS0);
	tegra_dc_writel(dc,
			PULSE_MODE_NORMAL |
			PULSE_POLARITY_HIGH |
			PULSE_QUAL_VACTIVE |
			PULSE_LAST_END_A,
			DC_DISP_H_PULSE2_CONTROL);
	tegra_dc_writel(dc, PULSE_START(pulse_start) | PULSE_END(pulse_start + 8),
		  DC_DISP_H_PULSE2_POSITION_A);

	tegra_hdmi_writel(hdmi,
			  VSYNC_WINDOW_END(0x210) |
			  VSYNC_WINDOW_START(0x200) |
			  VSYNC_WINDOW_ENABLE,
			  HDMI_NV_PDISP_HDMI_VSYNC_WINDOW);

	if ((dc->mode.h_active == 720) && ((dc->mode.v_active == 480) || (dc->mode.v_active == 576)))
		tegra_hdmi_writel(hdmi,
				  (dc->ndev->id ? HDMI_SRC_DISPLAYB : HDMI_SRC_DISPLAYA) |
				  ARM_VIDEO_RANGE_FULL,
				  HDMI_NV_PDISP_INPUT_CONTROL);
	else
		tegra_hdmi_writel(hdmi,
				  (dc->ndev->id ? HDMI_SRC_DISPLAYB : HDMI_SRC_DISPLAYA) |
				  ARM_VIDEO_RANGE_LIMITED,
				  HDMI_NV_PDISP_INPUT_CONTROL);

	clk_disable(hdmi->disp1_clk);
	clk_disable(hdmi->disp2_clk);

	dispclk_div_8_2 = clk_get_rate(hdmi->clk) / 1000000 * 4;
	tegra_hdmi_writel(hdmi,
			  SOR_REFCLK_DIV_INT(dispclk_div_8_2 >> 2) |
			  SOR_REFCLK_DIV_FRAC(dispclk_div_8_2),
			  HDMI_NV_PDISP_SOR_REFCLK);

	hdmi->clk_enabled = true;

	if (!hdmi->dvi) {
		err = tegra_dc_hdmi_setup_audio(dc, hdmi->audio_freq,
			hdmi->audio_source);

		if (err < 0)
			hdmi->dvi = true;
	}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	if (hdmi->eld_retrieved)
		tegra_dc_hdmi_setup_eld_buff(dc);
#endif

	rekey = HDMI_REKEY_DEFAULT;
	val = HDMI_CTRL_REKEY(rekey);
	val |= HDMI_CTRL_MAX_AC_PACKET((dc->mode.h_sync_width +
					dc->mode.h_back_porch +
					dc->mode.h_front_porch -
					rekey - 18) / 32);
	if (!hdmi->dvi)
		val |= HDMI_CTRL_ENABLE;
	tegra_hdmi_writel(hdmi, val, HDMI_NV_PDISP_HDMI_CTRL);

	if (hdmi->dvi)
		tegra_hdmi_writel(hdmi, 0x0,
				  HDMI_NV_PDISP_HDMI_GENERIC_CTRL);
	else
		tegra_hdmi_writel(hdmi, GENERIC_CTRL_AUDIO,
				  HDMI_NV_PDISP_HDMI_GENERIC_CTRL);

	tegra_dc_hdmi_setup_avi_infoframe(dc, hdmi->dvi);
	tegra_dc_hdmi_setup_audio_infoframe(dc, hdmi->dvi);
	tegra_dc_hdmi_setup_stereo_infoframe(dc);

	/* TMDS CONFIG */
	for (i = 0; i < ARRAY_SIZE(tdms_config); i++) {
		if (dc->mode.pclk <= tdms_config[i].pclk) {
			tegra_dc_hdmi_setup_tdms(hdmi, &tdms_config[i]);
			break;
		}
	}

	tegra_hdmi_writel(hdmi,
			  SOR_SEQ_CTL_PU_PC(0) |
			  SOR_SEQ_PU_PC_ALT(0) |
			  SOR_SEQ_PD_PC(8) |
			  SOR_SEQ_PD_PC_ALT(8),
			  HDMI_NV_PDISP_SOR_SEQ_CTL);

	val = SOR_SEQ_INST_WAIT_TIME(1) |
		SOR_SEQ_INST_WAIT_UNITS_VSYNC |
		SOR_SEQ_INST_HALT |
		SOR_SEQ_INST_PIN_A_LOW |
		SOR_SEQ_INST_PIN_B_LOW |
		SOR_SEQ_INST_DRIVE_PWM_OUT_LO;

	tegra_hdmi_writel(hdmi, val, HDMI_NV_PDISP_SOR_SEQ_INST0);
	tegra_hdmi_writel(hdmi, val, HDMI_NV_PDISP_SOR_SEQ_INST8);

	val = 0x1c800;
	val &= ~SOR_CSTM_ROTCLK(~0);
	val |= SOR_CSTM_ROTCLK(2);
	tegra_hdmi_writel(hdmi, val, HDMI_NV_PDISP_SOR_CSTM);


	tegra_dc_writel(dc, DISP_CTRL_MODE_STOP, DC_CMD_DISPLAY_COMMAND);
	tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);


	/* start SOR */
	tegra_hdmi_writel(hdmi,
			  SOR_PWR_NORMAL_STATE_PU |
			  SOR_PWR_NORMAL_START_NORMAL |
			  SOR_PWR_SAFE_STATE_PD |
			  SOR_PWR_SETTING_NEW_TRIGGER,
			  HDMI_NV_PDISP_SOR_PWR);
	tegra_hdmi_writel(hdmi,
			  SOR_PWR_NORMAL_STATE_PU |
			  SOR_PWR_NORMAL_START_NORMAL |
			  SOR_PWR_SAFE_STATE_PD |
			  SOR_PWR_SETTING_NEW_DONE,
			  HDMI_NV_PDISP_SOR_PWR);

	retries = 1000;
	do {
		BUG_ON(--retries < 0);
		val = tegra_hdmi_readl(hdmi, HDMI_NV_PDISP_SOR_PWR);
	} while (val & SOR_PWR_SETTING_NEW_PENDING);

	val = SOR_STATE_ASY_CRCMODE_COMPLETE |
		SOR_STATE_ASY_OWNER_HEAD0 |
		SOR_STATE_ASY_SUBOWNER_BOTH |
		SOR_STATE_ASY_PROTOCOL_SINGLE_TMDS_A |
		SOR_STATE_ASY_DEPOL_POS;

	if (dc->mode.flags & TEGRA_DC_MODE_FLAG_NEG_H_SYNC)
		val |= SOR_STATE_ASY_HSYNCPOL_NEG;
	else
		val |= SOR_STATE_ASY_HSYNCPOL_POS;

	if (dc->mode.flags & TEGRA_DC_MODE_FLAG_NEG_V_SYNC)
		val |= SOR_STATE_ASY_VSYNCPOL_NEG;
	else
		val |= SOR_STATE_ASY_VSYNCPOL_POS;

	tegra_hdmi_writel(hdmi, val, HDMI_NV_PDISP_SOR_STATE2);

	val = SOR_STATE_ASY_HEAD_OPMODE_AWAKE | SOR_STATE_ASY_ORMODE_NORMAL;
	tegra_hdmi_writel(hdmi, val, HDMI_NV_PDISP_SOR_STATE1);

	tegra_hdmi_writel(hdmi, 0, HDMI_NV_PDISP_SOR_STATE0);
	tegra_hdmi_writel(hdmi, SOR_STATE_UPDATE, HDMI_NV_PDISP_SOR_STATE0);
	tegra_hdmi_writel(hdmi, val | SOR_STATE_ATTACHED,
			  HDMI_NV_PDISP_SOR_STATE1);
	tegra_hdmi_writel(hdmi, 0, HDMI_NV_PDISP_SOR_STATE0);

	tegra_dc_writel(dc, HDMI_ENABLE, DC_DISP_DISP_WIN_OPTIONS);

	tegra_dc_writel(dc, PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
			PW4_ENABLE | PM0_ENABLE | PM1_ENABLE,
			DC_CMD_DISPLAY_POWER_CONTROL);

	tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY, DC_CMD_DISPLAY_COMMAND);
	tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_nvhdcp_set_plug(hdmi->nvhdcp, 1);
}

static void tegra_dc_hdmi_disable(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);

	tegra_nvhdcp_set_plug(hdmi->nvhdcp, 0);

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	tegra_hdmi_writel(hdmi, 0, HDMI_NV_PDISP_SOR_AUDIO_HDA_PRESENSE_0);
	/* sleep 1ms before disabling clocks to ensure HDA gets the interrupt */
	msleep(1);
	clk_disable(hdmi->hda2hdmi_clk);
	clk_disable(hdmi->hda2codec_clk);
	clk_disable(hdmi->hda_clk);
#endif
	tegra_periph_reset_assert(hdmi->clk);
	hdmi->clk_enabled = false;
	clk_disable(hdmi->clk);
	tegra_dvfs_set_rate(hdmi->clk, 0);
}

struct tegra_dc_out_ops tegra_dc_hdmi_ops = {
	.init = tegra_dc_hdmi_init,
	.destroy = tegra_dc_hdmi_destroy,
	.enable = tegra_dc_hdmi_enable,
	.disable = tegra_dc_hdmi_disable,
	.detect = tegra_dc_hdmi_detect,
	.suspend = tegra_dc_hdmi_suspend,
	.resume = tegra_dc_hdmi_resume,
	.mode_filter = tegra_dc_hdmi_mode_filter,
};

struct tegra_dc_edid *tegra_dc_get_edid(struct tegra_dc *dc)
{
	struct tegra_dc_hdmi_data *hdmi;

	/* TODO: Support EDID on non-HDMI devices */
	if (dc->out->type != TEGRA_DC_OUT_HDMI)
		return ERR_PTR(-ENODEV);

	hdmi = tegra_dc_get_outdata(dc);

	return tegra_edid_get_data(hdmi->edid);
}
EXPORT_SYMBOL(tegra_dc_get_edid);

void tegra_dc_put_edid(struct tegra_dc_edid *edid)
{
	tegra_edid_put_data(edid);
}
EXPORT_SYMBOL(tegra_dc_put_edid);
