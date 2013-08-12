/*
* Copyright (C) 2012 Invensense, Inc.
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

/**
 *  @addtogroup  DRIVERS
 *  @brief       Hardware drivers.
 *
 *  @{
 *      @file    inv_yas53x_trigger.c
 *      @brief   Invensense implementation for yas530/yas532/yas533
 *      @details This driver currently works for the yas530/yas532/yas533
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include "iio.h"
#include "sysfs.h"
#include "trigger.h"

#include "inv_yas53x_iio.h"

static const struct iio_trigger_ops inv_yas53x_trigger_ops = {
	.owner = THIS_MODULE,
};

int inv_yas53x_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
	struct inv_compass_state *st = iio_priv(indio_dev);

	st->trig = iio_allocate_trigger("%s-dev%d",
					indio_dev->name,
					indio_dev->id);
	if (st->trig == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	/* select default trigger */
	st->trig->dev.parent = &st->client->dev;
	st->trig->private_data = indio_dev;
	st->trig->ops = &inv_yas53x_trigger_ops;
	ret = iio_trigger_register(st->trig);

	/* select default trigger */
	indio_dev->trig = st->trig;
	if (ret)
		goto error_free_trig;

	return 0;

error_free_trig:
	iio_free_trigger(st->trig);
error_ret:
	return ret;
}

void inv_yas53x_remove_trigger(struct iio_dev *indio_dev)
{
	struct inv_compass_state *st = iio_priv(indio_dev);

	iio_trigger_unregister(st->trig);
	iio_free_trigger(st->trig);
}
/**
 *  @}
 */

