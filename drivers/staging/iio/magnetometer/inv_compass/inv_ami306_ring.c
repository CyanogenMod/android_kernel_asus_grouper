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
 *      @file    inv_ami306_ring.c
 *      @brief   Invensense implementation for AMI306
 *      @details This driver currently works for the AMI306
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

#include "../../iio.h"
#include "../../kfifo_buf.h"
#include "../../trigger_consumer.h"
#include "../../sysfs.h"

#include "inv_ami306_iio.h"

#define AMI30X_CALIBRATION_PATH "/data/sensors/AMI304_Config.ini"
#define AMI306_CALIBRATION_PATH "/data/sensors/AMI306_Config.ini"
enum Compass_cali_File {
	AMI30X = 0,
	AMI306,
	AMICaliMax
};

/* function for loading compass calibration file. */
static int access_cali_file(int *gain, int target)
{
	char buf[256];
	int ret;
	struct file *fp = NULL;
	mm_segment_t oldfs;
	int data[23];
	int ii;

	oldfs = get_fs();
	set_fs(get_ds());
	memset(buf, 0, sizeof(u8)*256);

	if (target == AMI30X)
		fp = filp_open(AMI30X_CALIBRATION_PATH, O_RDONLY, 0);
	else if (target == AMI306)
		fp = filp_open(AMI306_CALIBRATION_PATH, O_RDONLY, 0);
	else
		goto LoadFileFail;

	if (!IS_ERR(fp)) {

		pr_info("ami306 open calibration file success\n");
		ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
		pr_info("ami306 calibration content is :\n%s\n", buf);
		sscanf(buf, "%6d\n%6d %6d %6d\n"
			"%6d %6d %6d\n%6d %6d %6d\n"
			"%6d %6d %6d\n%6d %6d %6d\n"
			"%6d %6d %6d\n%6d %6d %6d\n%6d\n",
			&data[0],
			&data[1], &data[2], &data[3],
			&data[4], &data[5], &data[6],
			&data[7], &data[8], &data[9],
			&data[10], &data[11], &data[12],
			&data[13], &data[14], &data[15],
			&data[16], &data[17], &data[18],
			&data[19], &data[20], &data[21],
			&data[22]);

		if ((data[19] > 150) || (data[19] < 50) ||
		    (data[20] > 150) || (data[20] < 50) ||
		    (data[21] > 150) || (data[21] < 50)) {
			for (ii = 0; ii < 3; ii++)
				gain[ii] = 100;
		} else {
			for (ii = 0; ii < 3; ii++)
				gain[ii] = data[ii + 19];
		}

		pr_info("gain: %d %d %d\n", gain[0], gain[1], gain[2]);

		return 0;
	} else {
		pr_info("Compass compensation: No target File. (%d)\n",
		    target);
		set_fs(oldfs);
		return -1;
	}

LoadFileFail:
	return -1;
}

static int put_scan_to_buf(struct iio_dev *indio_dev, unsigned char *d,
				short *s, int scan_index) {
	struct iio_buffer *ring = indio_dev->buffer;
	int st;
	int i, d_ind;
	d_ind = 0;
	for (i = 0; i < 3; i++) {
		st = iio_scan_mask_query(indio_dev, ring, scan_index + i);
		if (st) {
			memcpy(&d[d_ind], &s[i], sizeof(s[i]));
			d_ind += sizeof(s[i]);
		}
	}
	return d_ind;
}

/**
 *  inv_read_fifo() - Transfer data from FIFO to ring buffer.
 */
int inv_read_ami306_fifo(struct iio_dev *indio_dev)
{
	struct inv_ami306_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int result, status, d_ind;
	char b;
	char *tmp;
	s64 tmp_buf[2];
	int ii;

	result = i2c_smbus_read_i2c_block_data(st->i2c, REG_AMI_STA1, 1, &b);
	if (result < 0)
		goto end_session;
	if (b & AMI_STA1_DRDY_BIT) {
		status = ami306_read_raw_data(st, st->compass_data);
		if (status) {
			pr_err("error reading raw\n");
			goto end_session;
		}

		if (!st->data_chk.load_cali) {
			for (ii = 0; ii < AMICaliMax; ii++) {
				result =
				    access_cali_file(st->data_chk.gain, ii);
				if (!result) {
					st->data_chk.fexist = 0;
					break;
				}
			}
			st->data_chk.load_cali = true;
		}

		for (ii = 0; ii < 3; ii++) {
			st->data_chk.ori[ii] = st->compass_data[ii];

			if (st->data_chk.gain[ii] > 0)
				st->compass_data[ii] =
					(short)((int)st->compass_data[ii] *
					100 / st->data_chk.gain[ii]);

			st->data_chk.post[ii] = st->compass_data[ii];
		}

		tmp = (unsigned char *)tmp_buf;
		d_ind = put_scan_to_buf(indio_dev, tmp, st->compass_data,
						INV_AMI306_SCAN_MAGN_X);
		if (ring->scan_timestamp)
			tmp_buf[(d_ind + 7)/8] = st->timestamp;
		ring->access->store_to(indio_dev->buffer, tmp, st->timestamp);
	} else if (b & AMI_STA1_DOR_BIT)
		pr_err("not ready\n");
end_session:
	b = AMI_CTRL3_FORCE_BIT;
	result = i2c_smbus_write_i2c_block_data(st->i2c, REG_AMI_CTRL3, 1, &b);

	return IRQ_HANDLED;
}

void inv_ami306_unconfigure_ring(struct iio_dev *indio_dev)
{
	iio_kfifo_free(indio_dev->buffer);
};
static int inv_ami306_postenable(struct iio_dev *indio_dev)
{
	struct inv_ami306_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int result;

	/* when all the outputs are disabled, even though buffer/enable is on,
	   do nothing */
	if (!(iio_scan_mask_query(indio_dev, ring, INV_AMI306_SCAN_MAGN_X) ||
	    iio_scan_mask_query(indio_dev, ring, INV_AMI306_SCAN_MAGN_Y) ||
	    iio_scan_mask_query(indio_dev, ring, INV_AMI306_SCAN_MAGN_Z)))
		return 0;

	result = set_ami306_enable(indio_dev, true);
	if (result)
		return result;
	schedule_delayed_work(&st->work, msecs_to_jiffies(st->delay));

	return 0;
}

static int inv_ami306_predisable(struct iio_dev *indio_dev)
{
	struct iio_buffer *ring = indio_dev->buffer;
	struct inv_ami306_state_s *st = iio_priv(indio_dev);

	cancel_delayed_work_sync(&st->work);
	clear_bit(INV_AMI306_SCAN_MAGN_X, ring->scan_mask);
	clear_bit(INV_AMI306_SCAN_MAGN_Y, ring->scan_mask);
	clear_bit(INV_AMI306_SCAN_MAGN_Z, ring->scan_mask);

	return 0;
}

static const struct iio_buffer_setup_ops inv_ami306_ring_setup_ops = {
	.preenable = &iio_sw_buffer_preenable,
	.postenable = &inv_ami306_postenable,
	.predisable = &inv_ami306_predisable,
};

int inv_ami306_configure_ring(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct iio_buffer *ring;

	ring = iio_kfifo_allocate(indio_dev);
	if (!ring) {
		ret = -ENOMEM;
		return ret;
	}
	indio_dev->buffer = ring;
	/* setup ring buffer */
	ring->scan_timestamp = true;
	indio_dev->setup_ops = &inv_ami306_ring_setup_ops;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	return 0;
}
/**
 *  @}
 */

