/*
 * drivers/video/tegra/host/gr3d/scale3d.c
 *
 * Tegra Graphics Host 3D clock scaling
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.  All rights reserved.
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

/*
 * 3d clock scaling
 *
 * module3d_notify_busy() is called upon submit, module3d_notify_idle() is
 * called when all outstanding submits are completed. Idle times are measured
 * over a fixed time period (scale3d.p_estimation_window). If the 3d module
 * idle time percentage goes over the limit (set in scale3d.p_idle_max), 3d
 * clocks are scaled down. If the percentage goes under the minimum limit (set
 * in scale3d.p_idle_min), 3d clocks are scaled up. An additional test is made
 * for clocking up quickly in response to load peaks.
 *
 * 3d.emc clock is scaled proportionately to 3d clock, with a quadratic-
 * bezier-like factor added to pull 3d.emc rate a bit lower.
 */

#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <mach/clk.h>
#include <mach/hardware.h>
#include "scale3d.h"
#include "dev.h"
#include <media/tegra_camera.h>

#define GR3D_PRINT_STATS   BIT(1)
#define GR3D_PRINT_BUSY    BIT(2)
#define GR3D_PRINT_IDLE    BIT(3)
#define GR3D_PRINT_HINT    BIT(4)
#define GR3D_PRINT_TARGET  BIT(5)

/* time frame for load and hint tracking - when events come in at a larger
 * interval, this probably indicates the current estimates are stale
 */
#define GR3D_TIMEFRAME 1000000 /* 1 sec */

/* the number of frames to use in the running average of load estimates and
 * throughput hints. Choosing 6 frames targets a window of about 100 msec.
 * Large flucutuations in frame times require a window that's large enough to
 * prevent spiky scaling behavior, which in turn exacerbates frame rate
 * instability.
 */
#define GR3D_FRAME_SPAN 6

static int scale3d_is_enabled(void);
static void scale3d_enable(int enable);

#define POW2(x) ((x) * (x))

/*
 * 3D clock scaling should be treated differently when camera is on in AP37.
 * 3D in AP37 requires 1.3V and combining it with MPE reaches to EDP limit.
 * 3D clock really needs to be set to lower frequency which requires 1.0V.
 * The same thing applies to 3D EMC clock.
 */
#define CAMERA_3D_CLK 300000000
#define CAMERA_3D_EMC_CLK 437000000

/*
 * debugfs parameters to control 3d clock scaling test
 *
 * estimation_window  - time period for clock rate evaluation
 * idle_min           - if less than [idle_min / 10] percent idle over
 *                      [estimation_window] microseconds, clock up.
 * idle_max      - if over [idle_max] percent idle over [estimation_window]
 *                 microseconds, clock down.
 * max_scale     - limits rate changes to no less than (100 - max_scale)% or
 *                 (100 + 2 * max_scale)% of current clock rate
 * verbosity     - bit flag to control debug printouts:
 *                 1 - stats
 *                 2 - busy
 *                 3 - idle
 *                 4 - hints
 *                 5 - target frequencies
 */

struct scale3d_info_rec {
	struct mutex lock; /* lock for timestamps etc */
	int enable;
	int init;
	ktime_t last_scale;
	int is_idle;
	ktime_t last_adjust;
	int fast_up_count;
	int slow_down_count;
	int is_scaled;
	long emc_slope;
	long emc_offset;
	long emc_dip_slope;
	long emc_dip_offset;
	long emc_xmid;
	unsigned long max_rate_3d;
	unsigned long min_rate_3d;
	ktime_t last_throughput_hint;

	struct work_struct work;
	struct delayed_work idle_timer;

	ktime_t last_estimation_window;
	long last_total_idle;
	long total_idle;
	ktime_t estimation_window;
	ktime_t last_notification;
	long idle_estimate;

	unsigned int scale;
	unsigned int p_busy_cutoff;
	unsigned int p_estimation_window;
	unsigned int p_use_throughput_hint;
	unsigned int p_throughput_lo_limit;
	unsigned int p_throughput_lower_limit;
	unsigned int p_throughput_hi_limit;
	unsigned int p_scale_step;
	unsigned int p_idle_min;
	unsigned int idle_min;
	unsigned int p_idle_max;
	unsigned int idle_max;
	unsigned int p_adjust;
	unsigned int p_scale_emc;
	unsigned int p_emc_dip;
	unsigned int p_verbosity;
	struct clk *clk_3d;
	struct clk *clk_3d2;
	struct clk *clk_3d_emc;
	int *freqlist;
	int freq_count;
};

static struct scale3d_info_rec scale3d;

static void scale_to_freq(unsigned long hz)
{
	unsigned long curr;

	if (!tegra_is_clk_enabled(scale3d.clk_3d))
		return;

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3)
		if (!tegra_is_clk_enabled(scale3d.clk_3d2))
			return;

	curr = clk_get_rate(scale3d.clk_3d);
	if (hz == curr)
		return;

	if (!(hz >= scale3d.max_rate_3d && curr == scale3d.max_rate_3d)) {
		if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3)
			clk_set_rate(scale3d.clk_3d2, 0);
		clk_set_rate(scale3d.clk_3d, hz);

		if (scale3d.p_scale_emc) {
			long after = (long) clk_get_rate(scale3d.clk_3d);
			hz = after * scale3d.emc_slope + scale3d.emc_offset;
			if (scale3d.p_emc_dip)
				hz -=
					(scale3d.emc_dip_slope *
					POW2(after / 1000 - scale3d.emc_xmid) +
					scale3d.emc_dip_offset);
			clk_set_rate(scale3d.clk_3d_emc, hz);
		}
	}
}

static void scale3d_clocks(unsigned long percent)
{
	unsigned long hz, curr;

	curr = clk_get_rate(scale3d.clk_3d);
	hz = percent * (curr / 100);

	scale_to_freq(hz);
}

static void scale3d_clocks_handler(struct work_struct *work)
{
	unsigned int scale;

	mutex_lock(&scale3d.lock);
	scale = scale3d.scale;
	mutex_unlock(&scale3d.lock);

	if (scale != 0)
		scale3d_clocks(scale);
}

void nvhost_scale3d_suspend(struct nvhost_device *dev)
{
	if (!scale3d.enable)
		return;

	cancel_work_sync(&scale3d.work);
	cancel_delayed_work(&scale3d.idle_timer);
}

/* set 3d clocks to max */
static void reset_3d_clocks(void)
{
	if (clk_get_rate(scale3d.clk_3d) != scale3d.max_rate_3d) {
		if (is_tegra_camera_on())
			clk_set_rate(scale3d.clk_3d, CAMERA_3D_CLK);
		else
			clk_set_rate(scale3d.clk_3d, scale3d.max_rate_3d);
		if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3) {
			if (is_tegra_camera_on())
				clk_set_rate(scale3d.clk_3d2, CAMERA_3D_CLK);
			else
				clk_set_rate(scale3d.clk_3d2,
							scale3d.max_rate_3d);
		}
		if (scale3d.p_scale_emc) {
			if (is_tegra_camera_on())
				clk_set_rate(scale3d.clk_3d_emc,
					CAMERA_3D_EMC_CLK);
			else
				clk_set_rate(scale3d.clk_3d_emc,
					clk_round_rate(scale3d.clk_3d_emc,
								UINT_MAX));
		}
	}
}

static int scale3d_is_enabled(void)
{
	int enable;

	if (!scale3d.enable)
		return 0;

	mutex_lock(&scale3d.lock);
	enable = scale3d.enable;
	mutex_unlock(&scale3d.lock);

	return enable;
}

static void scale3d_enable(int enable)
{
	int disable = 0;

	mutex_lock(&scale3d.lock);

	if (enable) {
		if (scale3d.max_rate_3d != scale3d.min_rate_3d)
			scale3d.enable = 1;
	} else {
		scale3d.enable = 0;
		disable = 1;
	}

	mutex_unlock(&scale3d.lock);

	if (disable)
		reset_3d_clocks();
}

/* scaling_adjust - use scale up / scale down hint counts to adjust scaling
 * parameters.
 *
 * hint_ratio is 100 x the ratio of scale up to scale down hints. Three cases
 * are distinguished:
 *
 * hint_ratio < HINT_RATIO_MIN - set parameters to maximize scaling effect
 * hint_ratio > HINT_RATIO_MAX - set parameters to minimize scaling effect
 * hint_ratio between limits - scale parameters linearly
 *
 * the parameters adjusted are
 *
 * * idle_min percentage
 * * idle_max percentage
 */
#define SCALING_ADJUST_PERIOD 1000000
#define HINT_RATIO_MAX 400
#define HINT_RATIO_MIN 100
#define HINT_RATIO_MID ((HINT_RATIO_MAX + HINT_RATIO_MIN) / 2)
#define HINT_RATIO_DIFF (HINT_RATIO_MAX - HINT_RATIO_MIN)

static void scaling_adjust(ktime_t time)
{
	long hint_ratio;
	int idle_min_adjustment;
	int idle_max_adjustment;
	unsigned long dt;

	dt = (unsigned long) ktime_us_delta(time, scale3d.last_adjust);
	if (dt < SCALING_ADJUST_PERIOD)
		return;

	hint_ratio = (100 * (scale3d.fast_up_count + 1)) /
				 (scale3d.slow_down_count + 1);

	if (hint_ratio > HINT_RATIO_MAX) {
		idle_min_adjustment = scale3d.p_idle_min;
		idle_max_adjustment = scale3d.p_idle_max;
	} else if (hint_ratio < HINT_RATIO_MIN) {
		idle_min_adjustment = -((int) scale3d.p_idle_min) / 2;
		idle_max_adjustment = -((int) scale3d.p_idle_max) / 2;
	} else {
		int diff;
		int factor;

		diff = HINT_RATIO_MID - hint_ratio;
		if (diff < 0)
			factor = -diff * 2;
		else {
			factor = -diff;
			diff *= 2;
		}

		idle_min_adjustment =
			(factor * (int) scale3d.p_idle_min) / HINT_RATIO_DIFF;
		idle_max_adjustment =
			(factor * (int) scale3d.p_idle_max) / HINT_RATIO_DIFF;
	}

	scale3d.idle_min = scale3d.p_idle_min + idle_min_adjustment;
	scale3d.idle_max = scale3d.p_idle_max + idle_max_adjustment;

	if (scale3d.p_verbosity & GR3D_PRINT_STATS)
		pr_info("scale3d stats: + %d - %d min %u max %u\n",
			scale3d.fast_up_count, scale3d.slow_down_count,
			scale3d.idle_min, scale3d.idle_max);

	scale3d.fast_up_count = 0;
	scale3d.slow_down_count = 0;
	scale3d.last_adjust = time;
}

#undef SCALING_ADJUST_PERIOD
#undef HINT_RATIO_MAX
#undef HINT_RATIO_MIN
#undef HINT_RATIO_MID
#undef HINT_RATIO_DIFF

static void scaling_state_check(ktime_t time)
{
	unsigned long dt;

	/* adjustment: set scale parameters (idle_min, idle_max) +/- 25%
	 * based on ratio of scale up to scale down hints
	 */
	if (scale3d.p_adjust)
		scaling_adjust(time);
	else {
		scale3d.idle_min = scale3d.p_idle_min;
		scale3d.idle_max = scale3d.p_idle_max;
	}

	dt = (unsigned long) ktime_us_delta(time, scale3d.last_scale);
	if (dt < scale3d.p_estimation_window)
		return;

	scale3d.last_scale = time;

	/* if too busy, scale up */
	if (scale3d.idle_estimate < scale3d.idle_min) {
		scale3d.is_scaled = 0;
		scale3d.fast_up_count++;
		if (scale3d.p_verbosity & GR3D_PRINT_BUSY)
			pr_info("scale3d: %ld/1000 busy\n",
				1000 - scale3d.idle_estimate);

		reset_3d_clocks();
		return;
	}

	if (scale3d.p_verbosity & GR3D_PRINT_IDLE)
		pr_info("scale3d: idle %lu/1000\n",
			scale3d.idle_estimate);

	if (scale3d.idle_estimate > scale3d.idle_max) {
		if (!scale3d.is_scaled)
			scale3d.is_scaled = 1;

		scale3d.slow_down_count++;
		/* if idle time is high, clock down */
		scale3d.scale =
			100 - (scale3d.idle_estimate - scale3d.idle_min) / 10;
		schedule_work(&scale3d.work);
	}
}

/* the idle estimate is done by keeping 2 time stamps, initially set to the
 * same time. Once the estimation_window time has been exceeded, one time
 * stamp is moved up to the current time. The idle estimate is calculated
 * based on the idle time percentage from the earlier estimate. The next time
 * an estimation_window time is exceeded, the previous idle time and estimates
 * are moved up - this is intended to prevent abrupt changes to the idle
 * estimate.
 */
static void update_load_estimate(int idle)
{
	unsigned long window;
	unsigned long t;

	ktime_t now = ktime_get();
	t = ktime_us_delta(now, scale3d.last_notification);

	/* if the last event was over GR3D_TIMEFRAME usec ago (1 sec), the
	 * current load tracking data is probably stale
	 */
	if (t > GR3D_TIMEFRAME) {
		scale3d.is_idle = idle;
		scale3d.last_notification = now;
		scale3d.estimation_window = now;
		scale3d.last_estimation_window = now;
		scale3d.total_idle = 0;
		scale3d.last_total_idle = 0;
		scale3d.idle_estimate = idle ? 1000 : 0;
		return;
	}

	if (scale3d.is_idle) {
		scale3d.total_idle += t;
		scale3d.last_total_idle += t;
	}

	scale3d.is_idle = idle;
	scale3d.last_notification = now;

	window = ktime_us_delta(now, scale3d.last_estimation_window);
	/* prevent division by 0 if events come in less than 1 usec apart */
	if (window > 0)
		scale3d.idle_estimate =
			(1000 * scale3d.last_total_idle) / window;

	/* move up to the last estimation window */
	if (ktime_us_delta(now, scale3d.estimation_window) >
		scale3d.p_estimation_window) {
		scale3d.last_estimation_window = scale3d.estimation_window;
		scale3d.last_total_idle = scale3d.total_idle;
		scale3d.total_idle = 0;
		scale3d.estimation_window = now;
	}
}

void nvhost_scale3d_notify_idle(struct nvhost_device *dev)
{
	ktime_t t;
	unsigned long dt;
	int delay;

	if (!scale3d.enable)
		return;

	update_load_estimate(1);

	t = ktime_get();

	/* if throughput hint enabled, and last hint is recent enough, return */
	if (scale3d.p_use_throughput_hint) {
		dt = ktime_us_delta(t, scale3d.last_throughput_hint);
		if (dt < GR3D_TIMEFRAME)
			return;
	}

	mutex_lock(&scale3d.lock);

	scaling_state_check(t);

	/* delay idle_max % of 2 * estimation_window (given in microseconds) */
	delay = (scale3d.idle_max * scale3d.p_estimation_window) / 500000;
	schedule_delayed_work(&scale3d.idle_timer, msecs_to_jiffies(delay));

	mutex_unlock(&scale3d.lock);
}

void nvhost_scale3d_notify_busy(struct nvhost_device *dev)
{
	ktime_t t;

	if (!scale3d.enable)
		return;

	update_load_estimate(0);

	t = ktime_get();

	/* if throughput hint enabled, and last hint is recent enough, return */
	if (scale3d.p_use_throughput_hint) {
		unsigned long dt;
		dt = ktime_us_delta(t, scale3d.last_throughput_hint);
		if (dt < GR3D_TIMEFRAME)
			return;
	}

	mutex_lock(&scale3d.lock);

	cancel_delayed_work(&scale3d.idle_timer);
	scaling_state_check(t);

	mutex_unlock(&scale3d.lock);
}

struct score {
	int size;		/* number of elements */
	int pos;		/* position in ring buffer */
	int count;		/* actual item count */
	unsigned int sum;	/* running sum */
	unsigned int prev;	/* previous score after 'reset' operation */
	unsigned int list[];	/* ring buffer */
};

static struct score *score_init(int capacity)
{
	struct score *s;

	s = kzalloc(sizeof(struct score) + capacity * sizeof(int), GFP_KERNEL);
	if (s == NULL)
		return NULL;

	s->size = capacity;

	return s;
}

static void score_delete(struct score *s)
{
	kfree(s);
}

#define score_get_average(s) ((s)->count ? (s)->sum / (s)->count : 0)

static void score_add(struct score *s, unsigned int reading)
{
	if (s->count < s->size) {
		s->sum += reading;
		s->count++;
	} else
		s->sum = s->sum - s->list[s->pos] + reading;

	s->list[s->pos] = reading;
	s->pos = (s->pos + 1) % s->size;
}


static unsigned int score_reset(struct score *s)
{
	s->prev = s->sum;

	s->count = 0;
	s->pos = 0;
	s->sum = 0;

	return s->prev;
}

int freqlist_up(long target, int steps)
{
	int i, pos;

	for (i = 0; i < scale3d.freq_count; i++)
		if (scale3d.freqlist[i] >= target)
			break;

	pos = min(scale3d.freq_count - 1, i + steps);
	return scale3d.freqlist[pos];
}

int freqlist_down(long target, int steps)
{
	int i, pos;

	for (i = scale3d.freq_count - 1; i >= 0; i--)
		if (scale3d.freqlist[i] <= target)
			break;

	pos = max(0, i - steps);
	return scale3d.freqlist[pos];
}

static struct score *busy_history;
static struct score *hint_history;

/* When a throughput hint is given, perform scaling based on the hint and on
 * the current idle estimation. This is done as follows:
 *
 * 1. On moderate loads force min frequency if the throughput hint is not too
 *    low.
 * 2. Otherwise, calculate target-rate = max-rate * load-percentage
 * 3. Unless the current or average throughput hint is below the minimum
 *    limit, in which case, choose a higher rate
 * 4. Or the average throughput hint is above the maximum limit, in which case,
 *    choose a lower rate.
 */
void nvhost_scale3d_set_throughput_hint(int hint)
{
	ktime_t now;
	long busy;
	long curr;
	long target;
	long dt;
	int avg_busy, avg_hint;

	if (!scale3d.enable)
		return;

	if (!scale3d.p_use_throughput_hint)
		return;

	if (scale3d.p_verbosity & GR3D_PRINT_HINT)
		pr_info("3fds: idle %ld, hint %d\n",
			scale3d.idle_estimate, hint);

	now = ktime_get();
	dt = ktime_us_delta(now, scale3d.last_throughput_hint);
	if (dt > GR3D_TIMEFRAME) {
		score_reset(busy_history);
		score_reset(hint_history);
	}

	scale3d.last_throughput_hint = now;

	busy = 1000 - scale3d.idle_estimate;
	curr = clk_get_rate(scale3d.clk_3d);
	target = scale3d.min_rate_3d;

	score_add(busy_history, busy);
	score_add(hint_history, hint);

	avg_busy = score_get_average(busy_history);
	avg_hint = score_get_average(hint_history);

	if (busy > 0)
		target = (curr / 1000) * busy;

	/* In practice, running the gpu at min frequency is typically
	 * sufficient to keep up performance at loads up to 70% on cases,
	 * but the average hint value is tested to keep performance up if
	 * needed.
	 */
	if (avg_busy <= scale3d.p_busy_cutoff &&
	    avg_hint >= scale3d.p_throughput_lower_limit)
		target = scale3d.min_rate_3d;
	else {
		target = (scale3d.max_rate_3d / 1000) * avg_busy;

		/* Scale up if either the current hint or the running average
		 * are below the target to prevent performance drop.
		 */
		if (hint <= scale3d.p_throughput_lo_limit ||
		    avg_hint <= scale3d.p_throughput_lo_limit) {
			if (target < curr)
				target = curr;
			target = freqlist_up(target, scale3d.p_scale_step);
		} else if (avg_hint >= scale3d.p_throughput_hi_limit) {
			if (target > curr)
				target = curr;
			target = freqlist_down(target, scale3d.p_scale_step);
		}
	}

	scale_to_freq(target);

	if (scale3d.p_verbosity & GR3D_PRINT_TARGET)
		pr_info("3dfs: busy %ld <%d>, curr %ld, t %ld, hint %d <%d>\n",
			busy, avg_busy, curr / 1000000, target, hint, avg_hint);
}
EXPORT_SYMBOL(nvhost_scale3d_set_throughput_hint);

static void scale3d_idle_handler(struct work_struct *work)
{
	int notify_idle = 0;

	if (!scale3d.enable)
		return;

	mutex_lock(&scale3d.lock);

	if (scale3d.is_idle && tegra_is_clk_enabled(scale3d.clk_3d)) {
		unsigned long curr = clk_get_rate(scale3d.clk_3d);
		if (curr > scale3d.min_rate_3d)
			notify_idle = 1;
	}

	mutex_unlock(&scale3d.lock);

	if (notify_idle)
		nvhost_scale3d_notify_idle(NULL);
}

/*
 * debugfs parameters to control 3d clock scaling
 */

void nvhost_scale3d_debug_init(struct dentry *de)
{
	struct dentry *d, *f;

	d = debugfs_create_dir("scaling", de);
	if (!d) {
		pr_err("scale3d: can\'t create debugfs directory\n");
		return;
	}

#define CREATE_SCALE3D_FILE(fname) \
	do {\
		f = debugfs_create_u32(#fname, S_IRUGO | S_IWUSR, d,\
			&scale3d.p_##fname);\
		if (NULL == f) {\
			pr_err("scale3d: can\'t create file " #fname "\n");\
			return;\
		} \
	} while (0)

	CREATE_SCALE3D_FILE(estimation_window);
	CREATE_SCALE3D_FILE(idle_min);
	CREATE_SCALE3D_FILE(idle_max);
	CREATE_SCALE3D_FILE(adjust);
	CREATE_SCALE3D_FILE(scale_emc);
	CREATE_SCALE3D_FILE(emc_dip);
	CREATE_SCALE3D_FILE(use_throughput_hint);
	CREATE_SCALE3D_FILE(throughput_hi_limit);
	CREATE_SCALE3D_FILE(throughput_lo_limit);
	CREATE_SCALE3D_FILE(throughput_lower_limit);
	CREATE_SCALE3D_FILE(scale_step);
	CREATE_SCALE3D_FILE(verbosity);
#undef CREATE_SCALE3D_FILE
}

static ssize_t enable_3d_scaling_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n", scale3d_is_enabled());

	return res;
}

static ssize_t enable_3d_scaling_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = 0;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	scale3d_enable(val);

	return count;
}

static DEVICE_ATTR(enable_3d_scaling, S_IRUGO | S_IWUSR,
	enable_3d_scaling_show, enable_3d_scaling_store);

#define MAX_FREQ_COUNT 0x40 /* 64 frequencies should be enough for anyone */

void nvhost_scale3d_init(struct nvhost_device *d)
{
	if (!scale3d.init) {
		int error;
		unsigned long max_emc, min_emc;
		long correction;
		long rate;
		int freqs[MAX_FREQ_COUNT];

		mutex_init(&scale3d.lock);

		INIT_WORK(&scale3d.work, scale3d_clocks_handler);
		INIT_DELAYED_WORK(&scale3d.idle_timer, scale3d_idle_handler);

		scale3d.clk_3d = d->clk[0];
		if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3) {
			scale3d.clk_3d2 = d->clk[1];
			scale3d.clk_3d_emc = d->clk[2];
		} else
			scale3d.clk_3d_emc = d->clk[1];

		scale3d.max_rate_3d = clk_round_rate(scale3d.clk_3d, UINT_MAX);
		scale3d.min_rate_3d = clk_round_rate(scale3d.clk_3d, 0);

		if (scale3d.max_rate_3d == scale3d.min_rate_3d) {
			pr_warn("scale3d: 3d max rate = min rate (%lu), "
				"disabling\n", scale3d.max_rate_3d);
			scale3d.enable = 0;
			return;
		}

		/* emc scaling:
		 *
		 * Remc = S * R3d + O - (Sd * (R3d - Rm)^2 + Od)
		 *
		 * Remc - 3d.emc rate
		 * R3d  - 3d.cbus rate
		 * Rm   - 3d.cbus 'middle' rate = (max + min)/2
		 * S    - emc_slope
		 * O    - emc_offset
		 * Sd   - emc_dip_slope
		 * Od   - emc_dip_offset
		 *
		 * this superposes a quadratic dip centered around the middle 3d
		 * frequency over a linear correlation of 3d.emc to 3d clock
		 * rates.
		 *
		 * S, O are chosen so that the maximum 3d rate produces the
		 * maximum 3d.emc rate exactly, and the minimum 3d rate produces
		 * at least the minimum 3d.emc rate.
		 *
		 * Sd and Od are chosen to produce the largest dip that will
		 * keep 3d.emc frequencies monotonously decreasing with 3d
		 * frequencies. To achieve this, the first derivative of Remc
		 * with respect to R3d should be zero for the minimal 3d rate:
		 *
		 *   R'emc = S - 2 * Sd * (R3d - Rm)
		 *   R'emc(R3d-min) = 0
		 *   S = 2 * Sd * (R3d-min - Rm)
		 *     = 2 * Sd * (R3d-min - R3d-max) / 2
		 *   Sd = S / (R3d-min - R3d-max)
		 *
		 *   +---------------------------------------------------+
		 *   | Sd = -(emc-max - emc-min) / (R3d-min - R3d-max)^2 |
		 *   +---------------------------------------------------+
		 *
		 *   dip = Sd * (R3d - Rm)^2 + Od
		 *
		 * requiring dip(R3d-min) = 0 and dip(R3d-max) = 0 gives
		 *
		 *   Sd * (R3d-min - Rm)^2 + Od = 0
		 *   Od = -Sd * ((R3d-min - R3d-max) / 2)^2
		 *      = -Sd * ((R3d-min - R3d-max)^2) / 4
		 *
		 *   +------------------------------+
		 *   | Od = (emc-max - emc-min) / 4 |
		 *   +------------------------------+
		 */

		max_emc = clk_round_rate(scale3d.clk_3d_emc, UINT_MAX);
		min_emc = clk_round_rate(scale3d.clk_3d_emc, 0);

		scale3d.emc_slope = (max_emc - min_emc) /
			 (scale3d.max_rate_3d - scale3d.min_rate_3d);
		scale3d.emc_offset = max_emc -
			scale3d.emc_slope * scale3d.max_rate_3d;
		/* guarantee max 3d rate maps to max emc rate */
		scale3d.emc_offset += max_emc -
			(scale3d.emc_slope * scale3d.max_rate_3d +
			scale3d.emc_offset);

		scale3d.emc_dip_offset = (max_emc - min_emc) / 4;
		scale3d.emc_dip_slope =
			-4 * (scale3d.emc_dip_offset /
			(POW2(scale3d.max_rate_3d - scale3d.min_rate_3d)));
		scale3d.emc_xmid =
			(scale3d.max_rate_3d + scale3d.min_rate_3d) / 2;
		correction =
			scale3d.emc_dip_offset +
				scale3d.emc_dip_slope *
				POW2(scale3d.max_rate_3d - scale3d.emc_xmid);
		scale3d.emc_dip_offset -= correction;

		scale3d.is_idle = 1;

		/* set scaling parameter defaults */
		scale3d.enable = 1;
		scale3d.idle_min = scale3d.p_idle_min = 100;
		scale3d.idle_max = scale3d.p_idle_max = 150;
		scale3d.p_scale_emc = 1;
		scale3d.p_emc_dip = 1;
		scale3d.p_verbosity = 0;
		scale3d.p_adjust = 1;
		scale3d.p_use_throughput_hint = 1;
		scale3d.p_throughput_lower_limit = 940;
		scale3d.p_throughput_lo_limit = 990;
		scale3d.p_throughput_hi_limit = 1010;
		scale3d.p_scale_step = 1;
		scale3d.p_estimation_window = 8000;
		scale3d.p_busy_cutoff = 750;

		error = device_create_file(&d->dev,
				&dev_attr_enable_3d_scaling);
		if (error)
			dev_err(&d->dev, "failed to create sysfs attributes");

		rate = 0;
		scale3d.freq_count = 0;
		while (rate <= scale3d.max_rate_3d) {
			long rounded_rate;
			if (unlikely(scale3d.freq_count == MAX_FREQ_COUNT)) {
				pr_err("%s: too many frequencies\n", __func__);
				break;
			}
			rounded_rate =
				clk_round_rate(scale3d.clk_3d, rate);
			freqs[scale3d.freq_count++] = rounded_rate;
			rate = rounded_rate + 2000;
		}
		scale3d.freqlist =
			kmalloc(scale3d.freq_count * sizeof(int), GFP_KERNEL);
		if (scale3d.freqlist == NULL)
			pr_err("%s: can\'t allocate freq table\n", __func__);

		memcpy(scale3d.freqlist, freqs,
			scale3d.freq_count * sizeof(int));

		busy_history = score_init(GR3D_FRAME_SPAN);
		if (busy_history == NULL)
			pr_err("%s: can\'t init load tracking array\n",
			       __func__);

		hint_history = score_init(GR3D_FRAME_SPAN);
		if (hint_history == NULL)
			pr_err("%s: can\'t init throughput tracking array\n",
			       __func__);

		scale3d.init = 1;
	}
}

void nvhost_scale3d_deinit(struct nvhost_device *dev)
{
	device_remove_file(&dev->dev, &dev_attr_enable_3d_scaling);
	scale3d.init = 0;
	if (scale3d.freqlist != NULL) {
		kfree(scale3d.freqlist);
		scale3d.freq_count = 0;
		scale3d.freqlist = NULL;
	}

	score_delete(busy_history);
	score_delete(hint_history);
}
