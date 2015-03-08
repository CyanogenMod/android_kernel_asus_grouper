/*
 * drivers/video/tegra/host/debug.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/io.h>

#include "bus.h"
#include "dev.h"
#include "debug.h"
#include "nvhost_acm.h"
#include "nvhost_channel.h"
#include "chip_support.h"

pid_t nvhost_debug_null_kickoff_pid;
unsigned int nvhost_debug_trace_cmdbuf;

pid_t nvhost_debug_force_timeout_pid;
u32 nvhost_debug_force_timeout_val;
u32 nvhost_debug_force_timeout_channel;

void nvhost_debug_output(struct output *o, const char* fmt, ...)
{
	va_list args;
	int len;

	va_start(args, fmt);
	len = vsnprintf(o->buf, sizeof(o->buf), fmt, args);
	va_end(args);
	o->fn(o->ctx, o->buf, len);
}

static int show_channels(struct device *dev, void *data)
{
	struct nvhost_channel *ch;
	struct nvhost_device *nvdev = to_nvhost_device(dev);
	struct output *o = data;
	struct nvhost_master *m;

	if (nvdev == NULL)
		return 0;

	m = nvhost_get_host(nvdev);
	ch = nvdev->channel;
	if (ch) {
		mutex_lock(&ch->reflock);
		if (ch->refcount) {
			mutex_lock(&ch->cdma.lock);
			nvhost_get_chip_ops()->debug.show_channel_fifo(m, ch, o, nvdev->index);
			nvhost_get_chip_ops()->debug.show_channel_cdma(m, ch, o, nvdev->index);
			mutex_unlock(&ch->cdma.lock);
		}
		mutex_unlock(&ch->reflock);
	}

	return 0;
}

static void show_syncpts(struct nvhost_master *m, struct output *o)
{
	int i;
	BUG_ON(!nvhost_get_chip_ops()->syncpt.name);
	nvhost_debug_output(o, "---- syncpts ----\n");
	for (i = 0; i < nvhost_syncpt_nb_pts(&m->syncpt); i++) {
		u32 max = nvhost_syncpt_read_max(&m->syncpt, i);
		u32 min = nvhost_syncpt_update_min(&m->syncpt, i);
		if (!min && !max)
			continue;
		nvhost_debug_output(o, "id %d (%s) min %d max %d\n",
				i, nvhost_get_chip_ops()->syncpt.name(&m->syncpt, i),
				min, max);
	}

	for (i = 0; i < nvhost_syncpt_nb_pts(&m->syncpt); i++) {
		u32 base_val;
		base_val = nvhost_syncpt_read_wait_base(&m->syncpt, i);
		if (base_val)
			nvhost_debug_output(o, "waitbase id %d val %d\n",
					i, base_val);
	}

	nvhost_debug_output(o, "\n");
}

static void show_all(struct nvhost_master *m, struct output *o)
{
	nvhost_module_busy(m->dev);

	nvhost_get_chip_ops()->debug.show_mlocks(m, o);
	show_syncpts(m, o);
	nvhost_debug_output(o, "---- channels ----\n");
	bus_for_each_dev(&(nvhost_bus_get())->nvhost_bus_type, NULL, o,
			show_channels);

	nvhost_module_idle(m->dev);
}

#ifdef CONFIG_DEBUG_FS
static int show_channels_no_fifo(struct device *dev, void *data)
{
	struct nvhost_channel *ch;
	struct nvhost_device *nvdev = to_nvhost_device(dev);
	struct output *o = data;
	struct nvhost_master *m;

	if (nvdev == NULL)
		return 0;

	m = nvhost_get_host(nvdev);
	ch = nvdev->channel;
	if (ch) {
		mutex_lock(&ch->reflock);
		if (ch->refcount) {
			mutex_lock(&ch->cdma.lock);
			nvhost_get_chip_ops()->debug.show_channel_cdma(m,
					ch, o, nvdev->index);
			mutex_unlock(&ch->cdma.lock);
		}
		mutex_unlock(&ch->reflock);
	}

	return 0;
}

static void show_all_no_fifo(struct nvhost_master *m, struct output *o)
{
	nvhost_module_busy(m->dev);

	nvhost_get_chip_ops()->debug.show_mlocks(m, o);
	show_syncpts(m, o);
	nvhost_debug_output(o, "---- channels ----\n");
	bus_for_each_dev(&(nvhost_bus_get())->nvhost_bus_type, NULL, o,
			show_channels_no_fifo);

	nvhost_module_idle(m->dev);
}

static int nvhost_debug_show_all(struct seq_file *s, void *unused)
{
	struct output o = {
		.fn = write_to_seqfile,
		.ctx = s
	};
	show_all(s->private, &o);
	return 0;
}
static int nvhost_debug_show(struct seq_file *s, void *unused)
{
	struct output o = {
		.fn = write_to_seqfile,
		.ctx = s
	};
	show_all_no_fifo(s->private, &o);
	return 0;
}

static int nvhost_debug_open_all(struct inode *inode, struct file *file)
{
	return single_open(file, nvhost_debug_show_all, inode->i_private);
}

static const struct file_operations nvhost_debug_all_fops = {
	.open		= nvhost_debug_open_all,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int nvhost_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvhost_debug_show, inode->i_private);
}

static const struct file_operations nvhost_debug_fops = {
	.open		= nvhost_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void nvhost_debug_init(struct nvhost_master *master)
{
	struct dentry *de = debugfs_create_dir("tegra_host", NULL);

	debugfs_create_file("status", S_IRUGO, de,
			master, &nvhost_debug_fops);
	debugfs_create_file("status_all", S_IRUGO, de,
			master, &nvhost_debug_all_fops);

	debugfs_create_u32("null_kickoff_pid", S_IRUGO|S_IWUSR, de,
			&nvhost_debug_null_kickoff_pid);
	debugfs_create_u32("trace_cmdbuf", S_IRUGO|S_IWUSR, de,
			&nvhost_debug_trace_cmdbuf);

	if (nvhost_get_chip_ops()->debug.debug_init)
		nvhost_get_chip_ops()->debug.debug_init(de);

	debugfs_create_u32("force_timeout_pid", S_IRUGO|S_IWUSR, de,
			&nvhost_debug_force_timeout_pid);
	debugfs_create_u32("force_timeout_val", S_IRUGO|S_IWUSR, de,
			&nvhost_debug_force_timeout_val);
	debugfs_create_u32("force_timeout_channel", S_IRUGO|S_IWUSR, de,
			&nvhost_debug_force_timeout_channel);
}
#else
void nvhost_debug_init(struct nvhost_master *master)
{
}
#endif

void nvhost_debug_dump(struct nvhost_master *master)
{
	struct output o = {
		.fn = write_to_printk
	};
	show_all(master, &o);
}
