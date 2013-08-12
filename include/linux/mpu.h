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
 *      @file    mpu.h
 *      @brief   mpu definition
 */

#ifndef __MPU_H_
#define __MPU_H_

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#endif

enum secondary_slave_type {
	SECONDARY_SLAVE_TYPE_NONE,
	SECONDARY_SLAVE_TYPE_ACCEL,
	SECONDARY_SLAVE_TYPE_COMPASS,
	SECONDARY_SLAVE_TYPE_PRESSURE,

	SECONDARY_SLAVE_TYPE_TYPES
};

enum ext_slave_id {
	ID_INVALID = 0,
	GYRO_ID_MPU3050,
	GYRO_ID_MPU6050A2,
	GYRO_ID_MPU6050B1,
	GYRO_ID_MPU6050B1_NO_ACCEL,
	GYRO_ID_ITG3500,

	ACCEL_ID_LIS331,
	ACCEL_ID_LSM303DLX,
	ACCEL_ID_LIS3DH,
	ACCEL_ID_KXSD9,
	ACCEL_ID_KXTF9,
	ACCEL_ID_BMA150,
	ACCEL_ID_BMA222,
	ACCEL_ID_BMA250,
	ACCEL_ID_ADXL34X,
	ACCEL_ID_MMA8450,
	ACCEL_ID_MMA845X,
	ACCEL_ID_MPU6050,

	COMPASS_ID_AK8963,
	COMPASS_ID_AK8975,
	COMPASS_ID_AK8972,
	COMPASS_ID_AMI30X,
	COMPASS_ID_AMI306,
	COMPASS_ID_YAS529,
	COMPASS_ID_YAS530,
	COMPASS_ID_HMC5883,
	COMPASS_ID_LSM303DLH,
	COMPASS_ID_LSM303DLM,
	COMPASS_ID_MMC314X,
	COMPASS_ID_HSCDTD002B,
	COMPASS_ID_HSCDTD004A,

	PRESSURE_ID_BMA085,
};

#define INV_PROD_KEY(ver, rev) (ver * 100 + rev)
/**
 * struct mpu_platform_data - Platform data for the mpu driver
 * @int_config:		Bits [7:3] of the int config register.
 * @level_shifter:	0: VLogic, 1: VDD
 * @orientation:	Orientation matrix of the gyroscope
 * @sec_slave_type:     secondary slave device type, can be compass, accel, etc
 * @sec_slave_id:       id of the secondary slave device
 * @secondary_i2c_address: secondary device's i2c address
 * @secondary_orientation: secondary device's orientation matrix
 * @key:                key for MPL library.
 *
 * Contains platform specific information on how to configure the MPU3050 to
 * work on this platform.  The orientation matricies are 3x3 rotation matricies
 * that are applied to the data to rotate from the mounting orientation to the
 * platform orientation.  The values must be one of 0, 1, or -1 and each row and
 * column should have exactly 1 non-zero value.
 */
struct mpu_platform_data {
	__u8 int_config;
	__u8 level_shifter;
	__s8 orientation[9];
	enum secondary_slave_type sec_slave_type;
	enum ext_slave_id sec_slave_id;
	__u16 secondary_i2c_addr;
	__s8 secondary_orientation[9];
	__u8 key[16];
};

#endif	/* __MPU_H_ */
