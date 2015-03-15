/*
 * Copyright (C) 2014-2015, Sultanxda <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "CPU-boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/slab.h>

enum {
	UNBOOST,
	BOOST,
};

struct boost_policy {
	unsigned int boost_freq;
	unsigned int boost_ms;
	unsigned int cpu;
	unsigned int cpu_boost;
	struct work_struct boost_work;
	struct delayed_work restore_work;
};

static DEFINE_PER_CPU(struct boost_policy, boost_info);
static struct workqueue_struct *boost_wq;

static bool suspended;

static u64 last_input_time;
#define MIN_INPUT_INTERVAL (150 * USEC_PER_MSEC)

static unsigned int input_boost_freq;
module_param(input_boost_freq, uint, 0644);

static unsigned int input_boost_ms;
module_param(input_boost_ms, uint, 0644);

static void set_boost(struct boost_policy *b, unsigned int boost)
{
	b->cpu_boost = boost;
	get_online_cpus();
	if (cpu_online(b->cpu))
		cpufreq_update_policy(b->cpu);
	put_online_cpus();
}

static void boost_all_cpus(unsigned int freq, unsigned int ms)
{
	struct boost_policy *b;
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		b->boost_freq = freq;
		b->boost_ms = ms;
		queue_work(boost_wq, &b->boost_work);
	}
}

static void __cpuinit cpu_boost_main(struct work_struct *work)
{
	struct boost_policy *b = container_of(work, struct boost_policy,
						boost_work);

	cancel_delayed_work_sync(&b->restore_work);

	/* Boost must be set from within work context to prevent deadlock. */
	set_boost(b, BOOST);

	queue_delayed_work(boost_wq, &b->restore_work,
				msecs_to_jiffies(b->boost_ms));
}

static void __cpuinit cpu_restore_main(struct work_struct *work)
{
	struct boost_policy *b = container_of(work, struct boost_policy,
						restore_work.work);

	set_boost(b, UNBOOST);
}

static int cpu_do_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct boost_policy *b = &per_cpu(boost_info, policy->cpu);

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	switch (b->cpu_boost) {
	case UNBOOST:
		policy->min = policy->cpuinfo.min_freq;
		break;
	case BOOST:
		if (b->boost_freq > policy->max)
			b->boost_freq = policy->max;
		policy->min = b->boost_freq;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu_do_boost_nb = {
	.notifier_call = cpu_do_boost,
};

static void cpu_boost_early_suspend(struct early_suspend *handler)
{
	struct boost_policy *b;
	unsigned int cpu;

	suspended = true;

	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		if (cancel_delayed_work_sync(&b->restore_work))
			set_boost(b, UNBOOST);
	}
}

static void __cpuinit cpu_boost_late_resume(struct early_suspend *handler)
{
	suspended = false;
}

static struct early_suspend __refdata cpu_boost_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = cpu_boost_early_suspend,
	.resume = cpu_boost_late_resume,
};

static void cpu_boost_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	struct boost_policy *b;
	unsigned int cpu;
	u64 now;

	if (suspended || !input_boost_freq || !input_boost_ms)
		return;

	now = ktime_to_us(ktime_get());
	if (now - last_input_time < MIN_INPUT_INTERVAL)
		return;

	/* exit if boost is pending for any cpu */
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		if (work_pending(&b->boost_work))
			return;
	}

	boost_all_cpus(input_boost_freq, input_boost_ms);
	last_input_time = ktime_to_us(ktime_get());
}

static int cpu_boost_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_input_boost";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void cpu_boost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_boost_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	/* keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler cpu_boost_input_handler = {
	.event		= cpu_boost_input_event,
	.connect	= cpu_boost_input_connect,
	.disconnect	= cpu_boost_input_disconnect,
	.name		= "cpu_input_boost",
	.id_table	= cpu_boost_ids,
};

static int __init cpu_boost_init(void)
{
	struct boost_policy *b;
	unsigned int cpu;
	int ret;

	boost_wq = alloc_workqueue("cpu_input_boost_wq", WQ_HIGHPRI, 0);
	if (!boost_wq) {
		pr_err("Failed to allocate workqueue\n");
		ret = -EFAULT;
		goto fail;
	}

	cpufreq_register_notifier(&cpu_do_boost_nb, CPUFREQ_POLICY_NOTIFIER);

	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		b->cpu = cpu;
		INIT_WORK(&b->boost_work, cpu_boost_main);
		INIT_DELAYED_WORK(&b->restore_work, cpu_restore_main);
	}

	ret = input_register_handler(&cpu_boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto fail;
	}

	register_early_suspend(&cpu_boost_early_suspend_handler);
fail:
	return ret;
}
late_initcall(cpu_boost_init);

MODULE_AUTHOR("Sultanxda <sultanxda@gmail.com>");
MODULE_DESCRIPTION("CPU Input Boost");
MODULE_LICENSE("GPLv2");
