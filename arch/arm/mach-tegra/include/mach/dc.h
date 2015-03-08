/*
 * arch/arm/mach-tegra/include/mach/dc.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
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

#ifndef __MACH_TEGRA_DC_H
#define __MACH_TEGRA_DC_H

#include <linux/pm.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <drm/drm_fixed.h>

#define TEGRA_MAX_DC		2
#define DC_N_WINDOWS		3


/* DSI pixel data format */
enum {
	TEGRA_DSI_PIXEL_FORMAT_16BIT_P,
	TEGRA_DSI_PIXEL_FORMAT_18BIT_P,
	TEGRA_DSI_PIXEL_FORMAT_18BIT_NP,
	TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
};

/* DSI virtual channel number */
enum {
	TEGRA_DSI_VIRTUAL_CHANNEL_0,
	TEGRA_DSI_VIRTUAL_CHANNEL_1,
	TEGRA_DSI_VIRTUAL_CHANNEL_2,
	TEGRA_DSI_VIRTUAL_CHANNEL_3,
};

/* DSI transmit method for video data */
enum {
	TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
};

/* DSI HS clock mode */
enum {
	TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
};

/* DSI burst mode setting in video mode. Each mode is assigned with a
 * fixed value. The rationale behind this is to avoid change of these
 * values, since the calculation of dsi clock depends on them. */
enum {
	TEGRA_DSI_VIDEO_NONE_BURST_MODE = 0,
	TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END = 1,
	TEGRA_DSI_VIDEO_BURST_MODE_LOWEST_SPEED = 2,
	TEGRA_DSI_VIDEO_BURST_MODE_LOW_SPEED = 3,
	TEGRA_DSI_VIDEO_BURST_MODE_MEDIUM_SPEED = 4,
	TEGRA_DSI_VIDEO_BURST_MODE_FAST_SPEED = 5,
	TEGRA_DSI_VIDEO_BURST_MODE_FASTEST_SPEED = 6,
};

enum {
	TEGRA_DSI_PACKET_CMD,
	TEGRA_DSI_DELAY_MS,
};

struct tegra_dsi_cmd {
	u8	cmd_type;
	u8	data_id;
	union {
		u16 data_len;
		u16 delay_ms;
		struct {
			u8 data0;
			u8 data1;
		} sp;
	} sp_len_dly;
	u8	*pdata;
};

#define DSI_CMD_SHORT(di, p0, p1)	{ \
					.cmd_type = TEGRA_DSI_PACKET_CMD, \
					.data_id = di, \
					.sp_len_dly.sp.data0 = p0, \
					.sp_len_dly.sp.data1 = p1, \
					}
#define DSI_DLY_MS(ms)	{ \
			.cmd_type = TEGRA_DSI_DELAY_MS, \
			.sp_len_dly.delay_ms = ms, \
			}

#define DSI_CMD_LONG(di, ptr)	{ \
				.cmd_type = TEGRA_DSI_PACKET_CMD, \
				.data_id = di, \
				.sp_len_dly.data_len = ARRAY_SIZE(ptr), \
				.pdata = ptr, \
				}

struct dsi_phy_timing_ns {
	u16		t_hsdexit_ns;
	u16		t_hstrail_ns;
	u16		t_datzero_ns;
	u16		t_hsprepare_ns;

	u16		t_clktrail_ns;
	u16		t_clkpost_ns;
	u16		t_clkzero_ns;
	u16		t_tlpx_ns;

	u16		t_clkprepare_ns;
	u16		t_clkpre_ns;
	u16		t_wakeup_ns;

	u16		t_taget_ns;
	u16		t_tasure_ns;
	u16		t_tago_ns;
};

/* Aggressiveness level of DSI suspend. The higher, the more aggressive. */
#define DSI_NO_SUSPEND			0
#define DSI_HOST_SUSPEND_LV0		1
#define DSI_HOST_SUSPEND_LV1		2
#define DSI_HOST_SUSPEND_LV2		3
#define DSI_SUSPEND_FULL		4

struct tegra_dsi_out {
	u8		n_data_lanes;			/* required */
	u8		pixel_format;			/* required */
	u8		refresh_rate;			/* required */
	u8		rated_refresh_rate;
	u8		panel_reset;			/* required */
	u8		virtual_channel;		/* required */
	u8		dsi_instance;
	u8		chip_id;
	u8		chip_rev;

	bool		panel_has_frame_buffer;	/* required*/
	bool		panel_send_dc_frames;

	struct tegra_dsi_cmd	*dsi_init_cmd;		/* required */
	u16		n_init_cmd;			/* required */

	struct tegra_dsi_cmd	*dsi_early_suspend_cmd;
	u16		n_early_suspend_cmd;

	struct tegra_dsi_cmd	*dsi_late_resume_cmd;
	u16		n_late_resume_cmd;

	struct tegra_dsi_cmd	*dsi_suspend_cmd;	/* required */
	u16		n_suspend_cmd;			/* required */

	u8		video_data_type;		/* required */
	u8		video_clock_mode;
	u8		video_burst_mode;

	u8		suspend_aggr;

	u16		panel_buffer_size_byte;
	u16		panel_reset_timeout_msec;

	bool		hs_cmd_mode_supported;
	bool		hs_cmd_mode_on_blank_supported;
	bool		enable_hs_clock_on_lp_cmd_mode;
	bool		no_pkt_seq_eot; /* 1st generation panel may not
					 * support eot. Don't set it for
					 * most panels. */
	bool		te_polarity_low;
	bool		power_saving_suspend;

	u32		max_panel_freq_khz;
	u32		lp_cmd_mode_freq_khz;
	u32		lp_read_cmd_mode_freq_khz;
	u32		hs_clk_in_lp_cmd_mode_freq_khz;
	u32		burst_mode_freq_khz;

	struct dsi_phy_timing_ns phy_timing;
};

enum {
	TEGRA_DC_STEREO_MODE_2D,
	TEGRA_DC_STEREO_MODE_3D
};

enum {
	TEGRA_DC_STEREO_LANDSCAPE,
	TEGRA_DC_STEREO_PORTRAIT
};

struct tegra_stereo_out {
	int  mode_2d_3d;
	int  orientation;

	void (*set_mode)(int mode);
	void (*set_orientation)(int orientation);
};

struct tegra_dc_mode {
	int	pclk;
	int	rated_pclk;
	int	h_ref_to_sync;
	int	v_ref_to_sync;
	int	h_sync_width;
	int	v_sync_width;
	int	h_back_porch;
	int	v_back_porch;
	int	h_active;
	int	v_active;
	int	h_front_porch;
	int	v_front_porch;
	int	stereo_mode;
	u32	flags;
};

#define TEGRA_DC_MODE_FLAG_NEG_V_SYNC	(1 << 0)
#define TEGRA_DC_MODE_FLAG_NEG_H_SYNC	(1 << 1)

enum {
	TEGRA_DC_OUT_RGB,
	TEGRA_DC_OUT_HDMI,
	TEGRA_DC_OUT_DSI,
};

struct tegra_dc_out_pin {
	int	name;
	int	pol;
};

enum {
	TEGRA_DC_OUT_PIN_DATA_ENABLE,
	TEGRA_DC_OUT_PIN_H_SYNC,
	TEGRA_DC_OUT_PIN_V_SYNC,
	TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
};

enum {
	TEGRA_DC_OUT_PIN_POL_LOW,
	TEGRA_DC_OUT_PIN_POL_HIGH,
};

enum {
	TEGRA_DC_DISABLE_DITHER = 1,
	TEGRA_DC_ORDERED_DITHER,
	TEGRA_DC_ERRDIFF_DITHER,
};

typedef u8 tegra_dc_bl_output[256];
typedef u8 *p_tegra_dc_bl_output;

struct tegra_dc_sd_blp {
	u16 time_constant;
	u8 step;
};

struct tegra_dc_sd_fc {
	u8 time_limit;
	u8 threshold;
};

struct tegra_dc_sd_rgb {
	u8 r;
	u8 g;
	u8 b;
};

struct tegra_dc_sd_agg_priorities {
	u8 pri_lvl;
	u8 agg[4];
};

struct tegra_dc_sd_settings {
	unsigned enable;
	bool use_auto_pwm;
	u8 hw_update_delay;
	u8 aggressiveness;
	short bin_width;
	u8 phase_in_settings;
	u8 phase_in_adjustments;
	u8 cmd;
	u8 final_agg;
	u16 cur_agg_step;
	u16 phase_settings_step;
	u16 phase_adj_step;
	u16 num_phase_in_steps;

	struct tegra_dc_sd_agg_priorities agg_priorities;

	bool use_vid_luma;
	struct tegra_dc_sd_rgb coeff;

	struct tegra_dc_sd_fc fc;
	struct tegra_dc_sd_blp blp;
	u8 bltf[4][4][4];
	struct tegra_dc_sd_rgb lut[4][9];

	atomic_t *sd_brightness;
	struct platform_device *bl_device;
};

enum {
	NO_CMD = 0x0,
	ENABLE = 0x1,
	DISABLE = 0x2,
	PHASE_IN = 0x4,
	AGG_CHG = 0x8,
};

enum {
	TEGRA_PIN_OUT_CONFIG_SEL_LHP0_LD21,
	TEGRA_PIN_OUT_CONFIG_SEL_LHP1_LD18,
	TEGRA_PIN_OUT_CONFIG_SEL_LHP2_LD19,
	TEGRA_PIN_OUT_CONFIG_SEL_LVP0_LVP0_Out,
	TEGRA_PIN_OUT_CONFIG_SEL_LVP1_LD20,

	TEGRA_PIN_OUT_CONFIG_SEL_LM1_M1,
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_LD21,
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_PM1,

	TEGRA_PIN_OUT_CONFIG_SEL_LDI_LD22,
	TEGRA_PIN_OUT_CONFIG_SEL_LPP_LD23,
	TEGRA_PIN_OUT_CONFIG_SEL_LDC_SDC,
	TEGRA_PIN_OUT_CONFIG_SEL_LSPI_DE,
};

struct tegra_dc_out {
	int				type;
	unsigned			flags;

	/* size in mm */
	unsigned			h_size;
	unsigned			v_size;

	int				dcc_bus;
	int				hotplug_gpio;
	const char			*parent_clk;
	const char			*parent_clk_backup;

	unsigned			max_pixclock;
	unsigned			order;
	unsigned			align;
	unsigned			depth;
	unsigned			dither;

	struct tegra_dc_mode		*modes;
	int				n_modes;

	struct tegra_dsi_out		*dsi;
	struct tegra_stereo_out		*stereo;

	unsigned			height; /* mm */
	unsigned			width; /* mm */

	struct tegra_dc_out_pin		*out_pins;
	unsigned			n_out_pins;

	struct tegra_dc_sd_settings	*sd_settings;

	u8			*out_sel_configs;
	unsigned		n_out_sel_configs;
	bool			user_needs_vblank;
	struct completion	user_vblank_comp;

	int	(*enable)(void);
	int	(*postpoweron)(void);
	int	(*prepoweroff)(void);
	int	(*disable)(void);

	int	(*hotplug_init)(void);
	int	(*postsuspend)(void);
};

/* bits for tegra_dc_out.flags */
#define TEGRA_DC_OUT_HOTPLUG_HIGH		(0 << 1)
#define TEGRA_DC_OUT_HOTPLUG_LOW		(1 << 1)
#define TEGRA_DC_OUT_HOTPLUG_MASK		(1 << 1)
#define TEGRA_DC_OUT_NVHDCP_POLICY_ALWAYS_ON	(0 << 2)
#define TEGRA_DC_OUT_NVHDCP_POLICY_ON_DEMAND	(1 << 2)
#define TEGRA_DC_OUT_NVHDCP_POLICY_MASK		(1 << 2)
#define TEGRA_DC_OUT_CONTINUOUS_MODE		(0 << 3)
#define TEGRA_DC_OUT_ONE_SHOT_MODE		(1 << 3)
#define TEGRA_DC_OUT_N_SHOT_MODE		(1 << 4)
#define TEGRA_DC_OUT_ONE_SHOT_LP_MODE		(1 << 5)

#define TEGRA_DC_ALIGN_MSB		0
#define TEGRA_DC_ALIGN_LSB		1

#define TEGRA_DC_ORDER_RED_BLUE		0
#define TEGRA_DC_ORDER_BLUE_RED		1

#define V_BLANK_FLIP		0
#define V_BLANK_NVSD		1

struct tegra_dc;
struct nvmap_handle_ref;

struct tegra_dc_csc {
	unsigned short yof;
	unsigned short kyrgb;
	unsigned short kur;
	unsigned short kvr;
	unsigned short kug;
	unsigned short kvg;
	unsigned short kub;
	unsigned short kvb;
};

/* palette lookup table */
struct tegra_dc_lut {
	u8 r[256];
	u8 g[256];
	u8 b[256];
};

struct tegra_dc_win {
	u8			idx;
	u8			fmt;
	u8			ppflags; /* see TEGRA_WIN_PPFLAG* */
	u32			flags;

	void			*virt_addr;
	dma_addr_t		phys_addr;
	dma_addr_t		phys_addr_u;
	dma_addr_t		phys_addr_v;
	unsigned		stride;
	unsigned		stride_uv;
	fixed20_12		x;
	fixed20_12		y;
	fixed20_12		w;
	fixed20_12		h;
	unsigned		out_x;
	unsigned		out_y;
	unsigned		out_w;
	unsigned		out_h;
	unsigned		z;
	u8			global_alpha;

	struct tegra_dc_csc	csc;

	int			dirty;
	int			underflows;
	struct tegra_dc		*dc;

	struct nvmap_handle_ref	*cur_handle;
	unsigned		bandwidth;
	unsigned		new_bandwidth;
	struct tegra_dc_lut	lut;
};

#define TEGRA_WIN_PPFLAG_CP_ENABLE	(1 << 0) /* enable RGB color lut */
#define TEGRA_WIN_PPFLAG_CP_FBOVERRIDE	(1 << 1) /* override fbdev color lut */

#define TEGRA_WIN_FLAG_ENABLED		(1 << 0)
#define TEGRA_WIN_FLAG_BLEND_PREMULT	(1 << 1)
#define TEGRA_WIN_FLAG_BLEND_COVERAGE	(1 << 2)
#define TEGRA_WIN_FLAG_INVERT_H		(1 << 3)
#define TEGRA_WIN_FLAG_INVERT_V		(1 << 4)
#define TEGRA_WIN_FLAG_TILED		(1 << 5)
#define TEGRA_WIN_FLAG_H_FILTER		(1 << 6)
#define TEGRA_WIN_FLAG_V_FILTER		(1 << 7)


#define TEGRA_WIN_BLEND_FLAGS_MASK \
	(TEGRA_WIN_FLAG_BLEND_PREMULT | TEGRA_WIN_FLAG_BLEND_COVERAGE)

/* Note: These are the actual values written to the DC_WIN_COLOR_DEPTH register
 * and may change in new tegra architectures.
 */
#define TEGRA_WIN_FMT_P1		0
#define TEGRA_WIN_FMT_P2		1
#define TEGRA_WIN_FMT_P4		2
#define TEGRA_WIN_FMT_P8		3
#define TEGRA_WIN_FMT_B4G4R4A4		4
#define TEGRA_WIN_FMT_B5G5R5A		5
#define TEGRA_WIN_FMT_B5G6R5		6
#define TEGRA_WIN_FMT_AB5G5R5		7
#define TEGRA_WIN_FMT_B8G8R8A8		12
#define TEGRA_WIN_FMT_R8G8B8A8		13
#define TEGRA_WIN_FMT_B6x2G6x2R6x2A8	14
#define TEGRA_WIN_FMT_R6x2G6x2B6x2A8	15
#define TEGRA_WIN_FMT_YCbCr422		16
#define TEGRA_WIN_FMT_YUV422		17
#define TEGRA_WIN_FMT_YCbCr420P		18
#define TEGRA_WIN_FMT_YUV420P		19
#define TEGRA_WIN_FMT_YCbCr422P		20
#define TEGRA_WIN_FMT_YUV422P		21
#define TEGRA_WIN_FMT_YCbCr422R		22
#define TEGRA_WIN_FMT_YUV422R		23
#define TEGRA_WIN_FMT_YCbCr422RA	24
#define TEGRA_WIN_FMT_YUV422RA		25

struct tegra_fb_data {
	int		win;

	int		xres;
	int		yres;
	int		bits_per_pixel; /* -1 means autodetect */

	unsigned long	flags;
};

#define TEGRA_FB_FLIP_ON_PROBE		(1 << 0)

struct tegra_dc_platform_data {
	unsigned long		flags;
	unsigned long		emc_clk_rate;
	struct tegra_dc_out	*default_out;
	struct tegra_fb_data	*fb;
};

#define TEGRA_DC_FLAG_ENABLED		(1 << 0)

int tegra_dc_get_stride(struct tegra_dc *dc, unsigned win);
struct tegra_dc *tegra_dc_get_dc(unsigned idx);
struct tegra_dc_win *tegra_dc_get_window(struct tegra_dc *dc, unsigned win);
bool tegra_dc_get_connected(struct tegra_dc *);
bool tegra_dc_hpd(struct tegra_dc *dc);


void tegra_dc_get_fbvblank(struct tegra_dc *dc, struct fb_vblank *vblank);
int tegra_dc_wait_for_vsync(struct tegra_dc *dc);
void tegra_dc_blank(struct tegra_dc *dc);

void tegra_dc_enable(struct tegra_dc *dc);
void tegra_dc_disable(struct tegra_dc *dc);

u32 tegra_dc_get_syncpt_id(const struct tegra_dc *dc, int i);
u32 tegra_dc_incr_syncpt_max(struct tegra_dc *dc, int i);
void tegra_dc_incr_syncpt_min(struct tegra_dc *dc, int i, u32 val);

/* tegra_dc_update_windows and tegra_dc_sync_windows do not support windows
 * with differenct dcs in one call
 */
int tegra_dc_update_windows(struct tegra_dc_win *windows[], int n);
int tegra_dc_sync_windows(struct tegra_dc_win *windows[], int n);
int tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable);
bool tegra_dc_is_within_n_vsync(struct tegra_dc *dc, s64 ts);
bool tegra_dc_does_vsync_separate(struct tegra_dc *dc, s64 new_ts, s64 old_ts);

int tegra_dc_set_mode(struct tegra_dc *dc, const struct tegra_dc_mode *mode);
struct fb_videomode;
int tegra_dc_set_fb_mode(struct tegra_dc *dc, const struct fb_videomode *fbmode,
	bool stereo_mode);

unsigned tegra_dc_get_out_height(const struct tegra_dc *dc);
unsigned tegra_dc_get_out_width(const struct tegra_dc *dc);
unsigned tegra_dc_get_out_max_pixclock(const struct tegra_dc *dc);

/* PM0 and PM1 signal control */
#define TEGRA_PWM_PM0 0
#define TEGRA_PWM_PM1 1

struct tegra_dc_pwm_params {
	int which_pwm;
	int gpio_conf_to_sfio;
	unsigned int period;
	unsigned int clk_div;
	unsigned int clk_select;
	unsigned int duty_cycle;
};

void tegra_dc_config_pwm(struct tegra_dc *dc, struct tegra_dc_pwm_params *cfg);

int tegra_dsi_send_panel_short_cmd(struct tegra_dc *dc, u8 *pdata, u8 data_len);

int tegra_dc_update_csc(struct tegra_dc *dc, int win_index);

int tegra_dc_update_lut(struct tegra_dc *dc, int win_index, int fboveride);

/*
 * In order to get a dc's current EDID, first call tegra_dc_get_edid() from an
 * interruptible context.  The returned value (if non-NULL) points to a
 * snapshot of the current state; after copying data from it, call
 * tegra_dc_put_edid() on that pointer.  Do not dereference anything through
 * that pointer after calling tegra_dc_put_edid().
 */
struct tegra_dc_edid {
	size_t		len;
	u8		buf[0];
};
struct tegra_dc_edid *tegra_dc_get_edid(struct tegra_dc *dc);
void tegra_dc_put_edid(struct tegra_dc_edid *edid);

int tegra_dc_set_flip_callback(void (*callback)(void));
int tegra_dc_unset_flip_callback(void);
int tegra_dc_get_panel_sync_rate(void);

#endif
