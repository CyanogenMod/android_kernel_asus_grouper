/*
 * drivers/video/tegra/dc/dc.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/nvhost.h>
#include <video/tegrafb.h>
#include <drm/drm_fixed.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/clk.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/mc.h>
#include <linux/nvhost.h>
#include <mach/latency_allowance.h>

#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"
#include "nvsd.h"

#define TEGRA_CRC_LATCHED_DELAY		34

#define DC_COM_PIN_OUTPUT_POLARITY1_INIT_VAL	0x01000000
#define DC_COM_PIN_OUTPUT_POLARITY3_INIT_VAL	0x0

static struct tegra_dc_mode override_disp_mode[3];

static void _tegra_dc_controller_disable(struct tegra_dc *dc);

struct tegra_dc *tegra_dcs[TEGRA_MAX_DC];

DEFINE_MUTEX(tegra_dc_lock);
DEFINE_MUTEX(shared_lock);

void tegra_dc_clk_enable(struct tegra_dc *dc)
{
	if (!tegra_is_clk_enabled(dc->clk)) {
		clk_enable(dc->clk);
		tegra_dvfs_set_rate(dc->clk, dc->mode.pclk);
	}
}

void tegra_dc_clk_disable(struct tegra_dc *dc)
{
	if (tegra_is_clk_enabled(dc->clk)) {
		clk_disable(dc->clk);
		tegra_dvfs_set_rate(dc->clk, 0);
	}
}

void tegra_dc_hold_dc_out(struct tegra_dc *dc)
{
	if (dc->out_ops->hold)
		dc->out_ops->hold(dc);
}

void tegra_dc_release_dc_out(struct tegra_dc *dc)
{
	if (dc->out_ops->release)
		dc->out_ops->release(dc);
}

#define DUMP_REG(a) do {			\
	snprintf(buff, sizeof(buff), "%-32s\t%03x\t%08lx\n", \
		 #a, a, tegra_dc_readl(dc, a));		      \
	print(data, buff);				      \
	} while (0)

#define print_underflow_info(dc) do {                 \
	trace_printk("%s:Underflow stats: underflows : %llu, "      \
			"undeflows_a : %llu, "                          \
			"underflows_b : %llu, "                         \
			"underflows_c : %llu\n",                        \
			dc->ndev->name,                                 \
			dc->stats.underflows,                           \
			dc->stats.underflows_a, dc->stats.underflows_b, \
			dc->stats.underflows_c);                        \
	} while (0)

static void _dump_regs(struct tegra_dc *dc, void *data,
		       void (* print)(void *data, const char *str))
{
	int i;
	char buff[256];

	mutex_lock(&dc->lock);
	tegra_dc_hold_dc_out(dc);
	tegra_dc_io_start(dc);

	DUMP_REG(DC_CMD_DISPLAY_COMMAND_OPTION0);
	DUMP_REG(DC_CMD_DISPLAY_COMMAND);
	DUMP_REG(DC_CMD_SIGNAL_RAISE);
	DUMP_REG(DC_CMD_INT_STATUS);
	DUMP_REG(DC_CMD_INT_MASK);
	DUMP_REG(DC_CMD_INT_ENABLE);
	DUMP_REG(DC_CMD_INT_TYPE);
	DUMP_REG(DC_CMD_INT_POLARITY);
	DUMP_REG(DC_CMD_SIGNAL_RAISE1);
	DUMP_REG(DC_CMD_SIGNAL_RAISE2);
	DUMP_REG(DC_CMD_SIGNAL_RAISE3);
	DUMP_REG(DC_CMD_STATE_ACCESS);
	DUMP_REG(DC_CMD_STATE_CONTROL);
	DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
	DUMP_REG(DC_CMD_REG_ACT_CONTROL);

	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS1);
	DUMP_REG(DC_DISP_DISP_WIN_OPTIONS);
	DUMP_REG(DC_DISP_MEM_HIGH_PRIORITY);
	DUMP_REG(DC_DISP_MEM_HIGH_PRIORITY_TIMER);
	DUMP_REG(DC_DISP_DISP_TIMING_OPTIONS);
	DUMP_REG(DC_DISP_REF_TO_SYNC);
	DUMP_REG(DC_DISP_SYNC_WIDTH);
	DUMP_REG(DC_DISP_BACK_PORCH);
	DUMP_REG(DC_DISP_DISP_ACTIVE);
	DUMP_REG(DC_DISP_FRONT_PORCH);
	DUMP_REG(DC_DISP_H_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_D);
	DUMP_REG(DC_DISP_V_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE3_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE3_POSITION_A);
	DUMP_REG(DC_DISP_M0_CONTROL);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_DISP_DI_CONTROL);
	DUMP_REG(DC_DISP_PP_CONTROL);
	DUMP_REG(DC_DISP_PP_SELECT_A);
	DUMP_REG(DC_DISP_PP_SELECT_B);
	DUMP_REG(DC_DISP_PP_SELECT_C);
	DUMP_REG(DC_DISP_PP_SELECT_D);
	DUMP_REG(DC_DISP_DISP_CLOCK_CONTROL);
	DUMP_REG(DC_DISP_DISP_INTERFACE_CONTROL);
	DUMP_REG(DC_DISP_DISP_COLOR_CONTROL);
	DUMP_REG(DC_DISP_SHIFT_CLOCK_OPTIONS);
	DUMP_REG(DC_DISP_DATA_ENABLE_OPTIONS);
	DUMP_REG(DC_DISP_SERIAL_INTERFACE_OPTIONS);
	DUMP_REG(DC_DISP_LCD_SPI_OPTIONS);
	DUMP_REG(DC_DISP_BORDER_COLOR);
	DUMP_REG(DC_DISP_COLOR_KEY0_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY0_UPPER);
	DUMP_REG(DC_DISP_COLOR_KEY1_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY1_UPPER);
	DUMP_REG(DC_DISP_CURSOR_FOREGROUND);
	DUMP_REG(DC_DISP_CURSOR_BACKGROUND);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR_NS);
	DUMP_REG(DC_DISP_CURSOR_POSITION);
	DUMP_REG(DC_DISP_CURSOR_POSITION_NS);
	DUMP_REG(DC_DISP_INIT_SEQ_CONTROL);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_A);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_B);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_C);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_D);
	DUMP_REG(DC_DISP_DC_MCCIF_FIFOCTRL);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0A_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0B_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0C_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY1B_HYST);
	DUMP_REG(DC_DISP_DAC_CRT_CTRL);
	DUMP_REG(DC_DISP_DISP_MISC_CONTROL);


	for (i = 0; i < 3; i++) {
		print(data, "\n");
		snprintf(buff, sizeof(buff), "WINDOW %c:\n", 'A' + i);
		print(data, buff);

		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
		DUMP_REG(DC_WIN_WIN_OPTIONS);
		DUMP_REG(DC_WIN_BYTE_SWAP);
		DUMP_REG(DC_WIN_BUFFER_CONTROL);
		DUMP_REG(DC_WIN_COLOR_DEPTH);
		DUMP_REG(DC_WIN_POSITION);
		DUMP_REG(DC_WIN_SIZE);
		DUMP_REG(DC_WIN_PRESCALED_SIZE);
		DUMP_REG(DC_WIN_H_INITIAL_DDA);
		DUMP_REG(DC_WIN_V_INITIAL_DDA);
		DUMP_REG(DC_WIN_DDA_INCREMENT);
		DUMP_REG(DC_WIN_LINE_STRIDE);
		DUMP_REG(DC_WIN_BUF_STRIDE);
		DUMP_REG(DC_WIN_UV_BUF_STRIDE);
		DUMP_REG(DC_WIN_BLEND_NOKEY);
		DUMP_REG(DC_WIN_BLEND_1WIN);
		DUMP_REG(DC_WIN_BLEND_2WIN_X);
		DUMP_REG(DC_WIN_BLEND_2WIN_Y);
		DUMP_REG(DC_WIN_BLEND_3WIN_XY);
		DUMP_REG(DC_WINBUF_START_ADDR);
		DUMP_REG(DC_WINBUF_START_ADDR_U);
		DUMP_REG(DC_WINBUF_START_ADDR_V);
		DUMP_REG(DC_WINBUF_ADDR_H_OFFSET);
		DUMP_REG(DC_WINBUF_ADDR_V_OFFSET);
		DUMP_REG(DC_WINBUF_UFLOW_STATUS);
		DUMP_REG(DC_WIN_CSC_YOF);
		DUMP_REG(DC_WIN_CSC_KYRGB);
		DUMP_REG(DC_WIN_CSC_KUR);
		DUMP_REG(DC_WIN_CSC_KVR);
		DUMP_REG(DC_WIN_CSC_KUG);
		DUMP_REG(DC_WIN_CSC_KVG);
		DUMP_REG(DC_WIN_CSC_KUB);
		DUMP_REG(DC_WIN_CSC_KVB);
	}

	DUMP_REG(DC_CMD_DISPLAY_POWER_CONTROL);
	DUMP_REG(DC_COM_PIN_OUTPUT_ENABLE2);
	DUMP_REG(DC_COM_PIN_OUTPUT_POLARITY2);
	DUMP_REG(DC_COM_PIN_OUTPUT_DATA2);
	DUMP_REG(DC_COM_PIN_INPUT_ENABLE2);
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT5);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_COM_PM1_CONTROL);
	DUMP_REG(DC_COM_PM1_DUTY_CYCLE);
	DUMP_REG(DC_DISP_SD_CONTROL);

	tegra_dc_io_end(dc);
	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);
}

#undef DUMP_REG

#ifdef DEBUG
static void dump_regs_print(void *data, const char *str)
{
	struct tegra_dc *dc = data;
	dev_dbg(&dc->ndev->dev, "%s", str);
}

static void dump_regs(struct tegra_dc *dc)
{
	_dump_regs(dc, dc, dump_regs_print);
}
#else /* !DEBUG */

static void dump_regs(struct tegra_dc *dc) {}

#endif /* DEBUG */

#ifdef CONFIG_DEBUG_FS

static void dbg_regs_print(void *data, const char *str)
{
	struct seq_file *s = data;

	seq_printf(s, "%s", str);
}

#undef DUMP_REG

static int dbg_dc_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	_dump_regs(dc, s, dbg_regs_print);

	return 0;
}


static int dbg_dc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open		= dbg_dc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_mode_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	struct tegra_dc_mode *m;

	mutex_lock(&dc->lock);
	m = &dc->mode;
	seq_printf(s,
		"pclk: %d\n"
		"h_ref_to_sync: %d\n"
		"v_ref_to_sync: %d\n"
		"h_sync_width: %d\n"
		"v_sync_width: %d\n"
		"h_back_porch: %d\n"
		"v_back_porch: %d\n"
		"h_active: %d\n"
		"v_active: %d\n"
		"h_front_porch: %d\n"
		"v_front_porch: %d\n"
		"stereo_mode: %d\n",
		m->pclk, m->h_ref_to_sync, m->v_ref_to_sync,
		m->h_sync_width, m->v_sync_width,
		m->h_back_porch, m->v_back_porch,
		m->h_active, m->v_active,
		m->h_front_porch, m->v_front_porch,
		m->stereo_mode);
	mutex_unlock(&dc->lock);
	return 0;
}

static int dbg_dc_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_mode_show, inode->i_private);
}

static const struct file_operations mode_fops = {
	.open		= dbg_dc_mode_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_stats_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	mutex_lock(&dc->lock);
	seq_printf(s,
		"underflows: %llu\n"
		"underflows_a: %llu\n"
		"underflows_b: %llu\n"
		"underflows_c: %llu\n",
		dc->stats.underflows,
		dc->stats.underflows_a,
		dc->stats.underflows_b,
		dc->stats.underflows_c);
	mutex_unlock(&dc->lock);

	return 0;
}

static int dbg_dc_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_stats_show, inode->i_private);
}

static const struct file_operations stats_fops = {
	.open		= dbg_dc_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void __devexit tegra_dc_remove_debugfs(struct tegra_dc *dc)
{
	if (dc->debugdir)
		debugfs_remove_recursive(dc->debugdir);
	dc->debugdir = NULL;
}

static void tegra_dc_create_debugfs(struct tegra_dc *dc)
{
	struct dentry *retval;

	dc->debugdir = debugfs_create_dir(dev_name(&dc->ndev->dev), NULL);
	if (!dc->debugdir)
		goto remove_out;

	retval = debugfs_create_file("regs", S_IRUGO, dc->debugdir, dc,
		&regs_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("mode", S_IRUGO, dc->debugdir, dc,
		&mode_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("stats", S_IRUGO, dc->debugdir, dc,
		&stats_fops);
	if (!retval)
		goto remove_out;

	return;
remove_out:
	dev_err(&dc->ndev->dev, "could not create debugfs\n");
	tegra_dc_remove_debugfs(dc);
}

#else /* !CONFIG_DEBUGFS */
static inline void tegra_dc_create_debugfs(struct tegra_dc *dc) { };
static inline void __devexit tegra_dc_remove_debugfs(struct tegra_dc *dc) { };
#endif /* CONFIG_DEBUGFS */

static int tegra_dc_set(struct tegra_dc *dc, int index)
{
	int ret = 0;

	mutex_lock(&tegra_dc_lock);
	if (index >= TEGRA_MAX_DC) {
		ret = -EINVAL;
		goto out;
	}

	if (dc != NULL && tegra_dcs[index] != NULL) {
		ret = -EBUSY;
		goto out;
	}

	tegra_dcs[index] = dc;

out:
	mutex_unlock(&tegra_dc_lock);

	return ret;
}

unsigned int tegra_dc_has_multiple_dc(void)
{
	unsigned int idx;
	unsigned int cnt = 0;
	struct tegra_dc *dc;

	mutex_lock(&tegra_dc_lock);
	for (idx = 0; idx < TEGRA_MAX_DC; idx++)
		cnt += ((dc = tegra_dcs[idx]) != NULL && dc->enabled) ? 1 : 0;
	mutex_unlock(&tegra_dc_lock);

	return (cnt > 1);
}

/* get the stride size of a window.
 * return: stride size in bytes for window win. or 0 if unavailble. */
int tegra_dc_get_stride(struct tegra_dc *dc, unsigned win)
{
	u32 stride;

	if (!dc->enabled)
		return 0;
	BUG_ON(win > DC_N_WINDOWS);
	mutex_lock(&dc->lock);
	tegra_dc_hold_dc_out(dc);
	tegra_dc_writel(dc, WINDOW_A_SELECT << win,
		DC_CMD_DISPLAY_WINDOW_HEADER);
	stride = tegra_dc_readl(dc, DC_WIN_LINE_STRIDE);
	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);
	return GET_LINE_STRIDE(stride);
}
EXPORT_SYMBOL(tegra_dc_get_stride);

struct tegra_dc *tegra_dc_get_dc(unsigned idx)
{
	if (idx < TEGRA_MAX_DC)
		return tegra_dcs[idx];
	else
		return NULL;
}
EXPORT_SYMBOL(tegra_dc_get_dc);

struct tegra_dc_win *tegra_dc_get_window(struct tegra_dc *dc, unsigned win)
{
	if (win >= dc->n_windows)
		return NULL;

	return &dc->windows[win];
}
EXPORT_SYMBOL(tegra_dc_get_window);

bool tegra_dc_get_connected(struct tegra_dc *dc)
{
	return dc->connected;
}
EXPORT_SYMBOL(tegra_dc_get_connected);

bool tegra_dc_hpd(struct tegra_dc *dc)
{
	int sense;
	int level;

	level = gpio_get_value(dc->out->hotplug_gpio);

	sense = dc->out->flags & TEGRA_DC_OUT_HOTPLUG_MASK;

	return (sense == TEGRA_DC_OUT_HOTPLUG_HIGH && level) ||
		(sense == TEGRA_DC_OUT_HOTPLUG_LOW && !level);
}
EXPORT_SYMBOL(tegra_dc_hpd);

static void tegra_dc_set_scaling_filter(struct tegra_dc *dc)
{
	unsigned i;
	unsigned v0 = 128;
	unsigned v1 = 0;
	/* linear horizontal and vertical filters */
	for (i = 0; i < 16; i++) {
		tegra_dc_writel(dc, (v1 << 16) | (v0 << 8),
				DC_WIN_H_FILTER_P(i));

		tegra_dc_writel(dc, v0,
				DC_WIN_V_FILTER_P(i));
		v0 -= 8;
		v1 += 8;
	}
}

static inline void disable_dc_irq(unsigned int irq)
{
	disable_irq(irq);
}

u32 tegra_dc_get_syncpt_id(const struct tegra_dc *dc, int i)
{
	return dc->syncpt[i].id;
}
EXPORT_SYMBOL(tegra_dc_get_syncpt_id);

u32 tegra_dc_incr_syncpt_max(struct tegra_dc *dc, int i)
{
	u32 max;

	mutex_lock(&dc->lock);
	tegra_dc_hold_dc_out(dc);
	max = nvhost_syncpt_incr_max_ext(dc->ndev,
		dc->syncpt[i].id, ((dc->enabled) ? 1 : 0));
	dc->syncpt[i].max = max;
	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);

	return max;
}

void tegra_dc_incr_syncpt_min(struct tegra_dc *dc, int i, u32 val)
{
	mutex_lock(&dc->lock);
	if (dc->enabled) {
		tegra_dc_hold_dc_out(dc);
		while (dc->syncpt[i].min < val) {
			dc->syncpt[i].min++;
			nvhost_syncpt_cpu_incr_ext(dc->ndev, dc->syncpt[i].id);
		}
		tegra_dc_release_dc_out(dc);
	}
	mutex_unlock(&dc->lock);
}

void
tegra_dc_config_pwm(struct tegra_dc *dc, struct tegra_dc_pwm_params *cfg)
{
	unsigned int ctrl;
	unsigned long out_sel;
	unsigned long cmd_state;

	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_hold_dc_out(dc);

	ctrl = ((cfg->period << PM_PERIOD_SHIFT) |
		(cfg->clk_div << PM_CLK_DIVIDER_SHIFT) |
		cfg->clk_select);

	/* The new value should be effected immediately */
	cmd_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, (cmd_state | (1 << 2)), DC_CMD_STATE_ACCESS);

	switch (cfg->which_pwm) {
	case TEGRA_PWM_PM0:
		/* Select the LM0 on PM0 */
		out_sel = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_SELECT5);
		out_sel &= ~(7 << 0);
		out_sel |= (3 << 0);
		tegra_dc_writel(dc, out_sel, DC_COM_PIN_OUTPUT_SELECT5);
		tegra_dc_writel(dc, ctrl, DC_COM_PM0_CONTROL);
		tegra_dc_writel(dc, cfg->duty_cycle, DC_COM_PM0_DUTY_CYCLE);
		break;
	case TEGRA_PWM_PM1:
		/* Select the LM1 on PM1 */
		out_sel = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_SELECT5);
		out_sel &= ~(7 << 4);
		out_sel |= (3 << 4);
		tegra_dc_writel(dc, out_sel, DC_COM_PIN_OUTPUT_SELECT5);
		tegra_dc_writel(dc, ctrl, DC_COM_PM1_CONTROL);
		tegra_dc_writel(dc, cfg->duty_cycle, DC_COM_PM1_DUTY_CYCLE);
		break;
	default:
		dev_err(&dc->ndev->dev, "Error: Need which_pwm\n");
		break;
	}
	tegra_dc_writel(dc, cmd_state, DC_CMD_STATE_ACCESS);
	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);
}
EXPORT_SYMBOL(tegra_dc_config_pwm);

void tegra_dc_set_out_pin_polars(struct tegra_dc *dc,
				const struct tegra_dc_out_pin *pins,
				const unsigned int n_pins)
{
	unsigned int i;

	int name;
	int pol;

	u32 pol1, pol3;

	u32 set1, unset1;
	u32 set3, unset3;

	set1 = set3 = unset1 = unset3 = 0;

	for (i = 0; i < n_pins; i++) {
		name = (pins + i)->name;
		pol  = (pins + i)->pol;

		/* set polarity by name */
		switch (name) {
		case TEGRA_DC_OUT_PIN_DATA_ENABLE:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set3 |= LSPI_OUTPUT_POLARITY_LOW;
			else
				unset3 |= LSPI_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_H_SYNC:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LHS_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LHS_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_V_SYNC:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LVS_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LVS_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_PIXEL_CLOCK:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LSC0_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LSC0_OUTPUT_POLARITY_LOW;
			break;
		default:
			printk("Invalid argument in function %s\n",
			       __FUNCTION__);
			break;
		}
	}

	pol1 = DC_COM_PIN_OUTPUT_POLARITY1_INIT_VAL;
	pol3 = DC_COM_PIN_OUTPUT_POLARITY3_INIT_VAL;

	pol1 |= set1;
	pol1 &= ~unset1;

	pol3 |= set3;
	pol3 &= ~unset3;

	tegra_dc_writel(dc, pol1, DC_COM_PIN_OUTPUT_POLARITY1);
	tegra_dc_writel(dc, pol3, DC_COM_PIN_OUTPUT_POLARITY3);
}

static struct tegra_dc_mode *tegra_dc_get_override_mode(struct tegra_dc *dc)
{
	if (dc->out->type == TEGRA_DC_OUT_RGB ||
		dc->out->type == TEGRA_DC_OUT_HDMI ||
		dc->out->type == TEGRA_DC_OUT_DSI)
		return override_disp_mode[dc->out->type].pclk ?
			&override_disp_mode[dc->out->type] : NULL;
	else
		return NULL;
}

static void tegra_dc_set_out(struct tegra_dc *dc, struct tegra_dc_out *out)
{
	struct tegra_dc_mode *mode;

	dc->out = out;
	mode = tegra_dc_get_override_mode(dc);

	if (mode)
		tegra_dc_set_mode(dc, mode);
	else if (out->n_modes > 0)
		tegra_dc_set_mode(dc, &dc->out->modes[0]);

	switch (out->type) {
	case TEGRA_DC_OUT_RGB:
		dc->out_ops = &tegra_dc_rgb_ops;
		break;

	case TEGRA_DC_OUT_HDMI:
		dc->out_ops = &tegra_dc_hdmi_ops;
		break;

	case TEGRA_DC_OUT_DSI:
		dc->out_ops = &tegra_dc_dsi_ops;
		break;

	default:
		dc->out_ops = NULL;
		break;
	}

	if (dc->out_ops && dc->out_ops->init)
		dc->out_ops->init(dc);

}

unsigned tegra_dc_get_out_height(const struct tegra_dc *dc)
{
	if (dc->out)
		return dc->out->height;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_height);

unsigned tegra_dc_get_out_width(const struct tegra_dc *dc)
{
	if (dc->out)
		return dc->out->width;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_width);

unsigned tegra_dc_get_out_max_pixclock(const struct tegra_dc *dc)
{
	if (dc->out && dc->out->max_pixclock)
		return dc->out->max_pixclock;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_max_pixclock);

void tegra_dc_enable_crc(struct tegra_dc *dc)
{
	u32 val;

	mutex_lock(&dc->lock);
	tegra_dc_hold_dc_out(dc);
	tegra_dc_io_start(dc);

	val = CRC_ALWAYS_ENABLE | CRC_INPUT_DATA_ACTIVE_DATA |
		CRC_ENABLE_ENABLE;
	tegra_dc_writel(dc, val, DC_COM_CRC_CONTROL);
	tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);
}

void tegra_dc_disable_crc(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	tegra_dc_hold_dc_out(dc);
	tegra_dc_writel(dc, 0x0, DC_COM_CRC_CONTROL);
	tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dc_io_end(dc);
	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);
}

u32 tegra_dc_read_checksum_latched(struct tegra_dc *dc)
{
	int crc = 0;

	if (!dc) {
		dev_err(&dc->ndev->dev, "Failed to get dc.\n");
		goto crc_error;
	}

	/* TODO: Replace mdelay with code to sync VBlANK, since
	 * DC_COM_CRC_CHECKSUM_LATCHED is available after VBLANK */
	mdelay(TEGRA_CRC_LATCHED_DELAY);

	mutex_lock(&dc->lock);
	tegra_dc_hold_dc_out(dc);
	crc = tegra_dc_readl(dc, DC_COM_CRC_CHECKSUM_LATCHED);
	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);
crc_error:
	return crc;
}

static bool tegra_dc_windows_are_dirty(struct tegra_dc *dc)
{
#ifndef CONFIG_TEGRA_SIMULATION_PLATFORM
	u32 val;

	val = tegra_dc_readl(dc, DC_CMD_STATE_CONTROL);
	if (val & (WIN_A_ACT_REQ | WIN_B_ACT_REQ | WIN_C_ACT_REQ))
	    return true;
#endif
	return false;
}

static inline void enable_dc_irq(unsigned int irq)
{
#ifndef CONFIG_TEGRA_FPGA_PLATFORM
	enable_irq(irq);
#else
	/* Always disable DC interrupts on FPGA. */
	disable_irq(irq);
#endif
}

void tegra_dc_get_fbvblank(struct tegra_dc *dc, struct fb_vblank *vblank)
{
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		vblank->flags = FB_VBLANK_HAVE_VSYNC;
}

int tegra_dc_wait_for_vsync(struct tegra_dc *dc)
{
	int ret = -ENOTTY;

	if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) || !dc->enabled)
		return ret;

	/*
	 * Logic is as follows
	 * a) Indicate we need a vblank.
	 * b) Wait for completion to be signalled from isr.
	 * c) Initialize completion for next iteration.
	 */

	tegra_dc_hold_dc_out(dc);
	dc->out->user_needs_vblank = true;

	ret = wait_for_completion_interruptible(&dc->out->user_vblank_comp);
	init_completion(&dc->out->user_vblank_comp);
	tegra_dc_release_dc_out(dc);

	return ret;
}

static void tegra_dc_vblank(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(work, struct tegra_dc, vblank_work);
	bool nvsd_updated = false;

	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_hold_dc_out(dc);
	/* use the new frame's bandwidth setting instead of max(current, new),
	 * skip this if we're using tegra_dc_one_shot_worker() */
	if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE))
		tegra_dc_program_bandwidth(dc, true);

	/* Clear the V_BLANK_FLIP bit of vblank ref-count if update is clean. */
	if (!tegra_dc_windows_are_dirty(dc))
		clear_bit(V_BLANK_FLIP, &dc->vblank_ref_count);

	/* Update the SD brightness */
	if (dc->enabled && dc->out->sd_settings) {
		nvsd_updated = nvsd_update_brightness(dc);
		/* Ref-count vblank if nvsd is on-going. Otherwise, clean the
		 * V_BLANK_NVSD bit of vblank ref-count. */
		if (nvsd_updated) {
			set_bit(V_BLANK_NVSD, &dc->vblank_ref_count);
			tegra_dc_unmask_interrupt(dc, V_BLANK_INT);
		} else {
			clear_bit(V_BLANK_NVSD, &dc->vblank_ref_count);
		}
	}

	/* Mask vblank interrupt if ref-count is zero. */
	if (!dc->vblank_ref_count)
		tegra_dc_mask_interrupt(dc, V_BLANK_INT);

	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);

	/* Do the actual brightness update outside of the mutex */
	if (nvsd_updated && dc->out->sd_settings &&
	    dc->out->sd_settings->bl_device) {

		struct platform_device *pdev = dc->out->sd_settings->bl_device;
		struct backlight_device *bl = platform_get_drvdata(pdev);
		if (bl)
			backlight_update_status(bl);
	}
}

static void tegra_dc_one_shot_worker(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(
		to_delayed_work(work), struct tegra_dc, one_shot_work);
	mutex_lock(&dc->lock);

	/* memory client has gone idle */
	tegra_dc_clear_bandwidth(dc);

	if (dc->out_ops->idle)
		dc->out_ops->idle(dc);

	mutex_unlock(&dc->lock);
}

/* return an arbitrarily large number if count overflow occurs.
 * make it a nice base-10 number to show up in stats output */
static u64 tegra_dc_underflow_count(struct tegra_dc *dc, unsigned reg)
{
	unsigned count = tegra_dc_readl(dc, reg);
	tegra_dc_writel(dc, 0, reg);
	return ((count & 0x80000000) == 0) ? count : 10000000000ll;
}

static void tegra_dc_underflow_handler(struct tegra_dc *dc)
{
	u32 val;
	int i;

	dc->stats.underflows++;
	if (dc->underflow_mask & WIN_A_UF_INT) {
		dc->stats.underflows_a += tegra_dc_underflow_count(dc,
			DC_WINBUF_AD_UFLOW_STATUS);
		trace_printk("%s:Window A Underflow\n", dc->ndev->name);
	}
	if (dc->underflow_mask & WIN_B_UF_INT) {
		dc->stats.underflows_b += tegra_dc_underflow_count(dc,
			DC_WINBUF_BD_UFLOW_STATUS);
		trace_printk("%s:Window B Underflow\n", dc->ndev->name);
	}
	if (dc->underflow_mask & WIN_C_UF_INT) {
		dc->stats.underflows_c += tegra_dc_underflow_count(dc,
			DC_WINBUF_CD_UFLOW_STATUS);
		trace_printk("%s:Window C Underflow\n", dc->ndev->name);
	}

	/* Check for any underflow reset conditions */
	for (i = 0; i < DC_N_WINDOWS; i++) {
		if (dc->underflow_mask & (WIN_A_UF_INT << i)) {
			dc->windows[i].underflows++;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
			if (dc->windows[i].underflows > 4) {
				schedule_work(&dc->reset_work);
				/* reset counter */
				dc->windows[i].underflows = 0;
				trace_printk("%s:Reset work scheduled for "
						"window %c\n",
						dc->ndev->name, (65 + i));
			}
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
			if (dc->windows[i].underflows > 4) {
				trace_printk("%s:window %c in underflow state."
					" enable UF_LINE_FLUSH to clear up\n",
					dc->ndev->name, (65 + i));
				tegra_dc_writel(dc, UF_LINE_FLUSH,
						DC_DISP_DISP_MISC_CONTROL);
				tegra_dc_writel(dc, GENERAL_UPDATE,
						DC_CMD_STATE_CONTROL);
				tegra_dc_writel(dc, GENERAL_ACT_REQ,
						DC_CMD_STATE_CONTROL);

				tegra_dc_writel(dc, 0,
						DC_DISP_DISP_MISC_CONTROL);
				tegra_dc_writel(dc, GENERAL_UPDATE,
						DC_CMD_STATE_CONTROL);
				tegra_dc_writel(dc, GENERAL_ACT_REQ,
						DC_CMD_STATE_CONTROL);
			}
#endif
		} else {
			dc->windows[i].underflows = 0;
		}
	}

	/* Clear the underflow mask now that we've checked it. */
	tegra_dc_writel(dc, dc->underflow_mask, DC_CMD_INT_STATUS);
	dc->underflow_mask = 0;
	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	tegra_dc_writel(dc, val | ALL_UF_INT, DC_CMD_INT_MASK);
	print_underflow_info(dc);
}

#ifndef CONFIG_TEGRA_FPGA_PLATFORM
static void tegra_dc_one_shot_irq(struct tegra_dc *dc, unsigned long status)
{
	/* pending user vblank, so wakeup */
	if ((status & (V_BLANK_INT | MSF_INT)) &&
	    (dc->out->user_needs_vblank)) {
		dc->out->user_needs_vblank = false;
		complete(&dc->out->user_vblank_comp);
	}

	if (status & V_BLANK_INT) {
		/* Sync up windows. */
		tegra_dc_trigger_windows(dc);

		/* Schedule any additional bottom-half vblank actvities. */
		queue_work(system_freezable_wq, &dc->vblank_work);
	}

	if (status & FRAME_END_INT) {
		/* Mark the frame_end as complete. */
		if (!completion_done(&dc->frame_end_complete))
			complete(&dc->frame_end_complete);
	}
}

static void tegra_dc_continuous_irq(struct tegra_dc *dc, unsigned long status)
{
	/* Schedule any additional bottom-half vblank actvities. */
	if (status & V_BLANK_INT)
		queue_work(system_freezable_wq, &dc->vblank_work);

	if (status & FRAME_END_INT) {
		struct timespec tm = CURRENT_TIME;
		dc->frame_end_timestamp = timespec_to_ns(&tm);
		wake_up(&dc->timestamp_wq);

		/* Mark the frame_end as complete. */
		if (!completion_done(&dc->frame_end_complete))
			complete(&dc->frame_end_complete);

		tegra_dc_trigger_windows(dc);
	}
}

/* XXX: Not sure if we limit look ahead to 1 frame */
bool tegra_dc_is_within_n_vsync(struct tegra_dc *dc, s64 ts)
{
	BUG_ON(!dc->frametime_ns);
	return ((ts - dc->frame_end_timestamp) < dc->frametime_ns);
}

bool tegra_dc_does_vsync_separate(struct tegra_dc *dc, s64 new_ts, s64 old_ts)
{
	BUG_ON(!dc->frametime_ns);
	return (((new_ts - old_ts) > dc->frametime_ns)
		|| (div_s64((new_ts - dc->frame_end_timestamp), dc->frametime_ns)
			!= div_s64((old_ts - dc->frame_end_timestamp),
				dc->frametime_ns)));
}
#endif

static irqreturn_t tegra_dc_irq(int irq, void *ptr)
{
#ifndef CONFIG_TEGRA_FPGA_PLATFORM
	struct tegra_dc *dc = ptr;
	unsigned long status;
	unsigned long underflow_mask;
	u32 val;

	if (!nvhost_module_powered_ext(nvhost_get_parent(dc->ndev))) {
		WARN(1, "IRQ when DC not powered!\n");
		tegra_dc_io_start(dc);
		status = tegra_dc_readl(dc, DC_CMD_INT_STATUS);
		tegra_dc_writel(dc, status, DC_CMD_INT_STATUS);
		tegra_dc_io_end(dc);
		return IRQ_HANDLED;
	}

	/* clear all status flags except underflow, save those for the worker */
	status = tegra_dc_readl(dc, DC_CMD_INT_STATUS);
	tegra_dc_writel(dc, status & ~ALL_UF_INT, DC_CMD_INT_STATUS);
	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	tegra_dc_writel(dc, val & ~ALL_UF_INT, DC_CMD_INT_MASK);

	/*
	 * Overlays can get thier internal state corrupted during and underflow
	 * condition.  The only way to fix this state is to reset the DC.
	 * if we get 4 consecutive frames with underflows, assume we're
	 * hosed and reset.
	 */
	underflow_mask = status & ALL_UF_INT;

	/* Check underflow */
	if (underflow_mask) {
		dc->underflow_mask |= underflow_mask;
		schedule_delayed_work(&dc->underflow_work,
			msecs_to_jiffies(1));
	}

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		tegra_dc_one_shot_irq(dc, status);
	else
		tegra_dc_continuous_irq(dc, status);

	return IRQ_HANDLED;
#else /* CONFIG_TEGRA_FPGA_PLATFORM */
	return IRQ_NONE;
#endif /* !CONFIG_TEGRA_FPGA_PLATFORM */
}

static void tegra_dc_set_color_control(struct tegra_dc *dc)
{
	u32 color_control;

	switch (dc->out->depth) {
	case 3:
		color_control = BASE_COLOR_SIZE111;
		break;

	case 6:
		color_control = BASE_COLOR_SIZE222;
		break;

	case 8:
		color_control = BASE_COLOR_SIZE332;
		break;

	case 9:
		color_control = BASE_COLOR_SIZE333;
		break;

	case 12:
		color_control = BASE_COLOR_SIZE444;
		break;

	case 15:
		color_control = BASE_COLOR_SIZE555;
		break;

	case 16:
		color_control = BASE_COLOR_SIZE565;
		break;

	case 18:
		color_control = BASE_COLOR_SIZE666;
		break;

	default:
		color_control = BASE_COLOR_SIZE888;
		break;
	}

	switch (dc->out->dither) {
	case TEGRA_DC_DISABLE_DITHER:
		color_control |= DITHER_CONTROL_DISABLE;
		break;
	case TEGRA_DC_ORDERED_DITHER:
		color_control |= DITHER_CONTROL_ORDERED;
		break;
	case TEGRA_DC_ERRDIFF_DITHER:
		/* The line buffer for error-diffusion dither is limited
		 * to 1280 pixels per line. This limits the maximum
		 * horizontal active area size to 1280 pixels when error
		 * diffusion is enabled.
		 */
		BUG_ON(dc->mode.h_active > 1280);
		color_control |= DITHER_CONTROL_ERRDIFF;
		break;
	}

	tegra_dc_writel(dc, color_control, DC_DISP_DISP_COLOR_CONTROL);
}

static u32 get_syncpt(struct tegra_dc *dc, int idx)
{
	u32 syncpt_id;

	switch (dc->ndev->id) {
	case 0:
		switch (idx) {
		case 0:
			syncpt_id = NVSYNCPT_DISP0_A;
			break;
		case 1:
			syncpt_id = NVSYNCPT_DISP0_B;
			break;
		case 2:
			syncpt_id = NVSYNCPT_DISP0_C;
			break;
		default:
			BUG();
			break;
		}
		break;
	case 1:
		switch (idx) {
		case 0:
			syncpt_id = NVSYNCPT_DISP1_A;
			break;
		case 1:
			syncpt_id = NVSYNCPT_DISP1_B;
			break;
		case 2:
			syncpt_id = NVSYNCPT_DISP1_C;
			break;
		default:
			BUG();
			break;
		}
		break;
	default:
		BUG();
		break;
	}

	return syncpt_id;
}

static int tegra_dc_init(struct tegra_dc *dc)
{
	int i;
	int int_enable;

	tegra_dc_writel(dc, 0x00000100, DC_CMD_GENERAL_INCR_SYNCPT_CNTRL);
	if (dc->ndev->id == 0) {
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0A,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0B,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0C,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY1B,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAYHC,
				      TEGRA_MC_PRIO_HIGH);
	} else if (dc->ndev->id == 1) {
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0AB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0BB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0CB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY1BB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAYHCB,
				      TEGRA_MC_PRIO_HIGH);
	}
	tegra_dc_writel(dc, 0x00000100 | dc->vblank_syncpt,
			DC_CMD_CONT_SYNCPT_VSYNC);
	tegra_dc_writel(dc, 0x00004700, DC_CMD_INT_TYPE);
	tegra_dc_writel(dc, 0x0001c700, DC_CMD_INT_POLARITY);
	tegra_dc_writel(dc, 0x00202020, DC_DISP_MEM_HIGH_PRIORITY);
	tegra_dc_writel(dc, 0x00010101, DC_DISP_MEM_HIGH_PRIORITY_TIMER);
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	tegra_dc_writel(dc, 0x00000000, DC_DISP_DISP_MISC_CONTROL);
#endif
	/* enable interrupts for vblank, frame_end and underflows */
	int_enable = (FRAME_END_INT | V_BLANK_INT | ALL_UF_INT);
	/* for panels with one-shot mode enable tearing effect interrupt */
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		int_enable |= MSF_INT;

	tegra_dc_writel(dc, int_enable, DC_CMD_INT_ENABLE);
	tegra_dc_writel(dc, ALL_UF_INT, DC_CMD_INT_MASK);

	tegra_dc_writel(dc, 0x00000000, DC_DISP_BORDER_COLOR);

	tegra_dc_set_color_control(dc);
	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *win = &dc->windows[i];
		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		tegra_dc_set_csc(dc, &win->csc);
		tegra_dc_set_lut(dc, win);
		tegra_dc_set_scaling_filter(dc);
	}


	for (i = 0; i < dc->n_windows; i++) {
		u32 syncpt = get_syncpt(dc, i);

		dc->syncpt[i].id = syncpt;

		dc->syncpt[i].min = dc->syncpt[i].max =
			nvhost_syncpt_read_ext(dc->ndev, syncpt);
	}

	print_mode_info(dc, dc->mode);

	if (dc->mode.pclk)
		if (tegra_dc_program_mode(dc, &dc->mode))
			return -EINVAL;

	/* Initialize SD AFTER the modeset.
	   nvsd_init handles the sd_settings = NULL case. */
	nvsd_init(dc, dc->out->sd_settings);

	return 0;
}

static bool _tegra_dc_controller_enable(struct tegra_dc *dc)
{
	int failed_init = 0;

	if (dc->out->enable)
		dc->out->enable();

	tegra_dc_setup_clk(dc, dc->clk);
	tegra_dc_clk_enable(dc);

	/* do not accept interrupts during initialization */
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);

	enable_dc_irq(dc->irq);

	failed_init = tegra_dc_init(dc);
	if (failed_init) {
		tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);
		disable_irq(dc->irq);
		tegra_dc_clear_bandwidth(dc);
		tegra_dc_clk_disable(dc);
		if (dc->out && dc->out->disable)
			dc->out->disable();
		return false;
	}

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	/* force a full blending update */
	dc->blend.z[0] = -1;

	tegra_dc_ext_enable(dc->ext);

	trace_printk("%s:enable\n", dc->ndev->name);

	tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	if (dc->out->postpoweron)
		dc->out->postpoweron();

	return true;
}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static bool _tegra_dc_controller_reset_enable(struct tegra_dc *dc)
{
	bool ret = true;

	if (dc->out->enable)
		dc->out->enable();

	tegra_dc_setup_clk(dc, dc->clk);
	tegra_dc_clk_enable(dc);

	if (dc->ndev->id == 0 && tegra_dcs[1] != NULL) {
		mutex_lock(&tegra_dcs[1]->lock);
		disable_irq(tegra_dcs[1]->irq);
	} else if (dc->ndev->id == 1 && tegra_dcs[0] != NULL) {
		mutex_lock(&tegra_dcs[0]->lock);
		disable_irq(tegra_dcs[0]->irq);
	}

	msleep(5);
	tegra_periph_reset_assert(dc->clk);
	msleep(2);
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
	tegra_periph_reset_deassert(dc->clk);
	msleep(1);
#endif

	if (dc->ndev->id == 0 && tegra_dcs[1] != NULL) {
		enable_dc_irq(tegra_dcs[1]->irq);
		mutex_unlock(&tegra_dcs[1]->lock);
	} else if (dc->ndev->id == 1 && tegra_dcs[0] != NULL) {
		enable_dc_irq(tegra_dcs[0]->irq);
		mutex_unlock(&tegra_dcs[0]->lock);
	}

	enable_dc_irq(dc->irq);

	if (tegra_dc_init(dc)) {
		dev_err(&dc->ndev->dev, "cannot initialize\n");
		ret = false;
	}

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	if (dc->out->postpoweron)
		dc->out->postpoweron();

	/* force a full blending update */
	dc->blend.z[0] = -1;

	tegra_dc_ext_enable(dc->ext);

	if (!ret) {
		dev_err(&dc->ndev->dev, "initialization failed,disabling");
		_tegra_dc_controller_disable(dc);
	}

	trace_printk("%s:reset enable\n", dc->ndev->name);
	return ret;
}
#endif

static bool _tegra_dc_enable(struct tegra_dc *dc)
{
	if (dc->mode.pclk == 0)
		return false;

	if (!dc->out)
		return false;

	tegra_dc_io_start(dc);

	if (!_tegra_dc_controller_enable(dc)) {
		tegra_dc_io_end(dc);
		return false;
	}
	return true;
}

void tegra_dc_enable(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);

	if (!dc->enabled)
		dc->enabled = _tegra_dc_enable(dc);

	mutex_unlock(&dc->lock);
	print_mode_info(dc, dc->mode);
}

static void _tegra_dc_controller_disable(struct tegra_dc *dc)
{
	unsigned i;

	// ensure prepoweroff called after backlight set to 0
	if ( dc->ndev->id==0 && dc->out->sd_settings && dc->out->sd_settings->bl_device) {
		struct platform_device *pdev = dc->out->sd_settings->bl_device;
		struct backlight_device *bl = platform_get_drvdata(pdev);
		int count = 0;
		while(bl->props.brightness!=0 && count<4)
		{
			count++;
			msleep(50);
		}
	}

	if (dc->out && dc->out->prepoweroff)
		dc->out->prepoweroff();

	if (dc->out_ops && dc->out_ops->disable)
		dc->out_ops->disable(dc);

	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);
	tegra_dc_writel(dc, 0, DC_CMD_INT_ENABLE);
	disable_irq(dc->irq);

	tegra_dc_clear_bandwidth(dc);
	tegra_dc_clk_disable(dc);

	if (dc->out && dc->out->disable)
		dc->out->disable();

	for (i = 0; i < dc->n_windows; i++) {
		struct tegra_dc_win *w = &dc->windows[i];

		/* reset window bandwidth */
		w->bandwidth = 0;
		w->new_bandwidth = 0;

		/* disable windows */
		w->flags &= ~TEGRA_WIN_FLAG_ENABLED;

		/* flush any pending syncpt waits */
		while (dc->syncpt[i].min < dc->syncpt[i].max) {
			trace_printk("%s:syncpt flush id=%d\n", dc->ndev->name,
				dc->syncpt[i].id);
			dc->syncpt[i].min++;
			nvhost_syncpt_cpu_incr_ext(dc->ndev, dc->syncpt[i].id);
		}
	}
	trace_printk("%s:disabled\n", dc->ndev->name);
}

void tegra_dc_stats_enable(struct tegra_dc *dc, bool enable)
{
#if 0 /* underflow interrupt is already enabled by dc reset worker */
	u32 val;
	if (dc->enabled)  {
		val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
		if (enable)
			val |= (WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT);
		else
			val &= ~(WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT);
		tegra_dc_writel(dc, val, DC_CMD_INT_ENABLE);
	}
#endif
}

bool tegra_dc_stats_get(struct tegra_dc *dc)
{
#if 0 /* right now it is always enabled */
	u32 val;
	bool res;

	if (dc->enabled)  {
		val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
		res = !!(val & (WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT));
	} else {
		res = false;
	}

	return res;
#endif
	return true;
}

/* make the screen blank by disabling all windows */
void tegra_dc_blank(struct tegra_dc *dc)
{
	struct tegra_dc_win *dcwins[DC_N_WINDOWS];
	unsigned i;

	for (i = 0; i < DC_N_WINDOWS; i++) {
		dcwins[i] = tegra_dc_get_window(dc, i);
		dcwins[i]->flags &= ~TEGRA_WIN_FLAG_ENABLED;
	}

	tegra_dc_update_windows(dcwins, DC_N_WINDOWS);
	tegra_dc_sync_windows(dcwins, DC_N_WINDOWS);
}

static void _tegra_dc_disable(struct tegra_dc *dc)
{
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		mutex_lock(&dc->one_shot_lock);
		cancel_delayed_work_sync(&dc->one_shot_work);
	}

	tegra_dc_hold_dc_out(dc);

	_tegra_dc_controller_disable(dc);
	tegra_dc_io_end(dc);

	tegra_dc_release_dc_out(dc);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		mutex_unlock(&dc->one_shot_lock);
}

void tegra_dc_disable(struct tegra_dc *dc)
{
	tegra_dc_ext_disable(dc->ext);

	/* it's important that new underflow work isn't scheduled before the
	 * lock is acquired. */
	cancel_delayed_work_sync(&dc->underflow_work);

	mutex_lock(&dc->lock);

	if (dc->enabled) {
		dc->enabled = false;

		if (!dc->suspended)
			_tegra_dc_disable(dc);
	}

#ifdef CONFIG_SWITCH
	switch_set_state(&dc->modeset_switch, 0);
#endif

	mutex_unlock(&dc->lock);
	print_mode_info(dc, dc->mode);
}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static void tegra_dc_reset_worker(struct work_struct *work)
{
	struct tegra_dc *dc =
		container_of(work, struct tegra_dc, reset_work);

	unsigned long val = 0;

	mutex_lock(&shared_lock);

	dev_warn(&dc->ndev->dev,
		"overlay stuck in underflow state.  resetting.\n");

	tegra_dc_ext_disable(dc->ext);

	mutex_lock(&dc->lock);

	if (dc->enabled == false)
		goto unlock;

	dc->enabled = false;

	/*
	 * off host read bus
	 */
	val = tegra_dc_readl(dc, DC_CMD_CONT_SYNCPT_VSYNC);
	val &= ~(0x00000100);
	tegra_dc_writel(dc, val, DC_CMD_CONT_SYNCPT_VSYNC);

	/*
	 * set DC to STOP mode
	 */
	tegra_dc_writel(dc, DISP_CTRL_MODE_STOP, DC_CMD_DISPLAY_COMMAND);

	msleep(10);

	_tegra_dc_controller_disable(dc);

	/* _tegra_dc_controller_reset_enable deasserts reset */
	_tegra_dc_controller_reset_enable(dc);

	dc->enabled = true;

	/* reopen host read bus */
	val = tegra_dc_readl(dc, DC_CMD_CONT_SYNCPT_VSYNC);
	val &= ~(0x00000100);
	val |= 0x100;
	tegra_dc_writel(dc, val, DC_CMD_CONT_SYNCPT_VSYNC);

unlock:
	mutex_unlock(&dc->lock);
	mutex_unlock(&shared_lock);
	trace_printk("%s:reset complete\n", dc->ndev->name);
}
#endif

static void tegra_dc_underflow_worker(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(
		to_delayed_work(work), struct tegra_dc, underflow_work);

	mutex_lock(&dc->lock);
	tegra_dc_hold_dc_out(dc);

	if (dc->enabled) {
		tegra_dc_underflow_handler(dc);
	}
	tegra_dc_release_dc_out(dc);
	mutex_unlock(&dc->lock);
}

#ifdef CONFIG_SWITCH
static ssize_t switch_modeset_print_mode(struct switch_dev *sdev, char *buf)
{
	struct tegra_dc *dc =
		container_of(sdev, struct tegra_dc, modeset_switch);

	if (!sdev->state)
		return sprintf(buf, "offline\n");

	return sprintf(buf, "%dx%d\n", dc->mode.h_active, dc->mode.v_active);
}
#endif

static int tegra_dc_probe(struct nvhost_device *ndev,
	struct nvhost_device_id *id_table)
{
	struct tegra_dc *dc;
	struct tegra_dc_mode *mode;
	struct clk *clk;
	struct clk *emc_clk;
	struct resource	*res;
	struct resource *base_res;
	struct resource *fb_mem = NULL;
	int ret = 0;
	void __iomem *base;
	int irq;
	int i;

	if (!ndev->dev.platform_data) {
		dev_err(&ndev->dev, "no platform data\n");
		return -ENOENT;
	}

	dc = kzalloc(sizeof(struct tegra_dc), GFP_KERNEL);
	if (!dc) {
		dev_err(&ndev->dev, "can't allocate memory for tegra_dc\n");
		return -ENOMEM;
	}

	irq = nvhost_get_irq_byname(ndev, "irq");
	if (irq <= 0) {
		dev_err(&ndev->dev, "no irq\n");
		ret = -ENOENT;
		goto err_free;
	}

	res = nvhost_get_resource_byname(ndev, IORESOURCE_MEM, "regs");
	if (!res) {
		dev_err(&ndev->dev, "no mem resource\n");
		ret = -ENOENT;
		goto err_free;
	}

	base_res = request_mem_region(res->start, resource_size(res),
		ndev->name);
	if (!base_res) {
		dev_err(&ndev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto err_free;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&ndev->dev, "registers can't be mapped\n");
		ret = -EBUSY;
		goto err_release_resource_reg;
	}

	fb_mem = nvhost_get_resource_byname(ndev, IORESOURCE_MEM, "fbmem");

	clk = clk_get(&ndev->dev, NULL);
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&ndev->dev, "can't get clock\n");
		ret = -ENOENT;
		goto err_iounmap_reg;
	}

	emc_clk = clk_get(&ndev->dev, "emc");
	if (IS_ERR_OR_NULL(emc_clk)) {
		dev_err(&ndev->dev, "can't get emc clock\n");
		ret = -ENOENT;
		goto err_put_clk;
	}

	dc->clk = clk;
	dc->emc_clk = emc_clk;
	dc->shift_clk_div = 1;
	/* Initialize one shot work delay, it will be assigned by dsi
	 * according to refresh rate later. */
	dc->one_shot_delay_ms = 40;

	dc->base_res = base_res;
	dc->base = base;
	dc->irq = irq;
	dc->ndev = ndev;
	dc->pdata = ndev->dev.platform_data;

	/*
	 * The emc is a shared clock, it will be set based on
	 * the requirements for each user on the bus.
	 */
	dc->emc_clk_rate = 0;

	mutex_init(&dc->lock);
	mutex_init(&dc->one_shot_lock);
	init_completion(&dc->frame_end_complete);
	init_waitqueue_head(&dc->wq);
	init_waitqueue_head(&dc->timestamp_wq);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	INIT_WORK(&dc->reset_work, tegra_dc_reset_worker);
#endif
	INIT_WORK(&dc->vblank_work, tegra_dc_vblank);
	dc->vblank_ref_count = 0;
	INIT_DELAYED_WORK(&dc->underflow_work, tegra_dc_underflow_worker);
	INIT_DELAYED_WORK(&dc->one_shot_work, tegra_dc_one_shot_worker);

	tegra_dc_init_lut_defaults(&dc->fb_lut);

	dc->n_windows = DC_N_WINDOWS;
	for (i = 0; i < dc->n_windows; i++) {
		struct tegra_dc_win *win = &dc->windows[i];
		win->idx = i;
		win->dc = dc;
		tegra_dc_init_csc_defaults(&win->csc);
		tegra_dc_init_lut_defaults(&win->lut);
	}

	ret = tegra_dc_set(dc, ndev->id);
	if (ret < 0) {
		dev_err(&ndev->dev, "can't add dc\n");
		goto err_free_irq;
	}

	nvhost_set_drvdata(ndev, dc);

#ifdef CONFIG_SWITCH
	dc->modeset_switch.name = dev_name(&ndev->dev);
	dc->modeset_switch.state = 0;
	dc->modeset_switch.print_state = switch_modeset_print_mode;
	switch_dev_register(&dc->modeset_switch);
#endif

	tegra_dc_feature_register(dc);

	if (dc->pdata->default_out)
		tegra_dc_set_out(dc, dc->pdata->default_out);
	else
		dev_err(&ndev->dev, "No default output specified.  Leaving output disabled.\n");

	dc->vblank_syncpt = (dc->ndev->id == 0) ?
		NVSYNCPT_VBLANK0 : NVSYNCPT_VBLANK1;

	dc->ext = tegra_dc_ext_register(ndev, dc);
	if (IS_ERR_OR_NULL(dc->ext)) {
		dev_warn(&ndev->dev, "Failed to enable Tegra DC extensions.\n");
		dc->ext = NULL;
	}

	/* interrupt handler must be registered before tegra_fb_register() */
	if (request_irq(irq, tegra_dc_irq, 0,
			dev_name(&ndev->dev), dc)) {
		dev_err(&ndev->dev, "request_irq %d failed\n", irq);
		ret = -EBUSY;
		goto err_put_emc_clk;
	}
	disable_dc_irq(irq);

	mutex_lock(&dc->lock);
	if (dc->pdata->flags & TEGRA_DC_FLAG_ENABLED)
		dc->enabled = _tegra_dc_enable(dc);
	mutex_unlock(&dc->lock);

	tegra_dc_create_debugfs(dc);

	dev_info(&ndev->dev, "probed\n");

	if (dc->pdata->fb) {
		if (dc->enabled && dc->pdata->fb->bits_per_pixel == -1) {
			unsigned long fmt;
			tegra_dc_writel(dc,
					WINDOW_A_SELECT << dc->pdata->fb->win,
					DC_CMD_DISPLAY_WINDOW_HEADER);

			fmt = tegra_dc_readl(dc, DC_WIN_COLOR_DEPTH);
			dc->pdata->fb->bits_per_pixel =
				tegra_dc_fmt_bpp(fmt);
		}

		mode = tegra_dc_get_override_mode(dc);
		if (mode) {
			dc->pdata->fb->xres = mode->h_active;
			dc->pdata->fb->yres = mode->v_active;
		}

		dc->fb = tegra_fb_register(ndev, dc, dc->pdata->fb, fb_mem);

		if (IS_ERR_OR_NULL(dc->fb))
			dc->fb = NULL;
	}

	if (dc->out && dc->out->hotplug_init)
		dc->out->hotplug_init();

	if (dc->out_ops && dc->out_ops->detect)
		dc->out_ops->detect(dc);
	else
		dc->connected = true;

	tegra_dc_create_sysfs(&dc->ndev->dev);

	return 0;

err_free_irq:
	free_irq(irq, dc);
err_put_emc_clk:
	clk_put(emc_clk);
err_put_clk:
	clk_put(clk);
err_iounmap_reg:
	iounmap(base);
	if (fb_mem)
		release_resource(fb_mem);
err_release_resource_reg:
	release_resource(base_res);
err_free:
	kfree(dc);

	return ret;
}

static int tegra_dc_remove(struct nvhost_device *ndev)
{
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);

	tegra_dc_remove_sysfs(&dc->ndev->dev);
	tegra_dc_remove_debugfs(dc);

	if (dc->fb) {
		tegra_fb_unregister(dc->fb);
		if (dc->fb_mem)
			release_resource(dc->fb_mem);
	}

	tegra_dc_ext_disable(dc->ext);

	if (dc->ext)
		tegra_dc_ext_unregister(dc->ext);

	if (dc->enabled)
		_tegra_dc_disable(dc);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&dc->modeset_switch);
#endif
	free_irq(dc->irq, dc);
	clk_put(dc->emc_clk);
	clk_put(dc->clk);
	iounmap(dc->base);
	if (dc->fb_mem)
		release_resource(dc->base_res);
	kfree(dc);
	tegra_dc_set(NULL, ndev->id);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_dc_suspend(struct nvhost_device *ndev, pm_message_t state)
{
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);

	trace_printk("%s:suspend\n", dc->ndev->name);
	dev_info(&ndev->dev, "suspend\n");

	tegra_dc_ext_disable(dc->ext);

	mutex_lock(&dc->lock);

	if (dc->out_ops && dc->out_ops->suspend)
		dc->out_ops->suspend(dc);

	if (dc->enabled) {
		_tegra_dc_disable(dc);

		dc->suspended = true;
	}

	if (dc->out && dc->out->postsuspend) {
		dc->out->postsuspend();
		if (dc->out->type && dc->out->type == TEGRA_DC_OUT_HDMI)
			/*
			 * avoid resume event due to voltage falling
			 */
			msleep(100);
	}

	mutex_unlock(&dc->lock);

	return 0;
}

static int tegra_dc_resume(struct nvhost_device *ndev)
{
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);

	trace_printk("%s:resume\n", dc->ndev->name);
	dev_info(&ndev->dev, "resume\n");

	mutex_lock(&dc->lock);
	dc->suspended = false;

	if (dc->enabled)
		_tegra_dc_enable(dc);

	if (dc->out && dc->out->hotplug_init)
		dc->out->hotplug_init();

	if (dc->out_ops && dc->out_ops->resume)
		dc->out_ops->resume(dc);
	mutex_unlock(&dc->lock);

	return 0;
}

#endif /* CONFIG_PM */

static void tegra_dc_shutdown(struct nvhost_device *ndev)
{
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);

	if (!dc || !dc->enabled)
		return;

	tegra_dc_blank(dc);
	tegra_dc_disable(dc);
}

extern int suspend_set(const char *val, struct kernel_param *kp)
{
	if (!strcmp(val, "dump"))
		dump_regs(tegra_dcs[0]);
#ifdef CONFIG_PM
	else if (!strcmp(val, "suspend"))
		tegra_dc_suspend(tegra_dcs[0]->ndev, PMSG_SUSPEND);
	else if (!strcmp(val, "resume"))
		tegra_dc_resume(tegra_dcs[0]->ndev);
#endif

	return 0;
}

extern int suspend_get(char *buffer, struct kernel_param *kp)
{
	return 0;
}

int suspend;

module_param_call(suspend, suspend_set, suspend_get, &suspend, 0644);

struct nvhost_driver tegra_dc_driver = {
	.driver = {
		.name = "tegradc",
		.owner = THIS_MODULE,
	},
	.probe = tegra_dc_probe,
	.remove = tegra_dc_remove,
#ifdef CONFIG_PM
	.suspend = tegra_dc_suspend,
	.resume = tegra_dc_resume,
#endif
	.shutdown = tegra_dc_shutdown,
};

#ifndef MODULE
static int __init parse_disp_params(char *options, struct tegra_dc_mode *mode)
{
	int i, params[11];
	char *p;

	for (i = 0; i < ARRAY_SIZE(params); i++) {
		if ((p = strsep(&options, ",")) != NULL) {
			if (*p)
				params[i] = simple_strtoul(p, &p, 10);
		} else
			return -EINVAL;
	}

	if ((mode->pclk = params[0]) == 0)
		return -EINVAL;

	mode->h_active      = params[1];
	mode->v_active      = params[2];
	mode->h_ref_to_sync = params[3];
	mode->v_ref_to_sync = params[4];
	mode->h_sync_width  = params[5];
	mode->v_sync_width  = params[6];
	mode->h_back_porch  = params[7];
	mode->v_back_porch  = params[8];
	mode->h_front_porch = params[9];
	mode->v_front_porch = params[10];

	return 0;
}

static int __init tegra_dc_mode_override(char *str)
{
	char *p = str, *options;

	if (!p || !*p)
		return -EINVAL;

	p = strstr(str, "hdmi:");
	if (p) {
		p += 5;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_HDMI]))
			return -EINVAL;
	}

	p = strstr(str, "rgb:");
	if (p) {
		p += 4;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_RGB]))
			return -EINVAL;
	}

	p = strstr(str, "dsi:");
	if (p) {
		p += 4;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_DSI]))
			return -EINVAL;
	}

	return 0;
}

__setup("disp_params=", tegra_dc_mode_override);
#endif

static int __init tegra_dc_module_init(void)
{
	int ret = tegra_dc_ext_module_init();
	if (ret)
		return ret;
	return nvhost_driver_register(&tegra_dc_driver);
}

static void __exit tegra_dc_module_exit(void)
{
	nvhost_driver_unregister(&tegra_dc_driver);
	tegra_dc_ext_module_exit();
}

module_exit(tegra_dc_module_exit);
module_init(tegra_dc_module_init);
