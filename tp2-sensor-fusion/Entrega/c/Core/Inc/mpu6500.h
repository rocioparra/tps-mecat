/*
 * mpu6500.h
 *
 *  Created on: Oct 13, 2020
 *      Author: rochi
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_


#include <stdint.h>

#define MPU_SELF_TEST_X_GYRO	0x00
#define MPU_SELF_TEST_Y_GYRO	0x01
#define MPU_SELF_TEST_Z_GYRO	0x02

#define MPU_SELF_TEST_X_ACCEL	0x0D
#define MPU_SELF_TEST_Y_ACCEL	0x0E
#define MPU_SELF_TEST_Z_ACCEL	0x0F

#define MPU_XG_OFFSET_H			0x13
#define MPU_XG_OFFSET_L			0x14
#define MPU_YG_OFFSET_H			0x15
#define MPU_YG_OFFSET_L			0x16
#define MPU_ZG_OFFSET_H			0x17
#define MPU_ZG_OFFSET_L			0x18
#define MPU_SMPLRT_DIV			0x19

#define MPU_CONFIG				0x1A
#define MPU_GYRO_CONFIG			0x1B
#define MPU_ACCEL_CONFIG		0x1C
#define MPU_ACCEL_CONFIG_2		0x1D

#define MPU_LP_ACCEL_ODR		0x1E
#define MPU_WOM_THR				0x1F

#define MPU_FIFO_EN				0x23

#define MPU_I2C_MST_CTRL		0x24
#define MPU_I2C_SLV0_ADDR		0x25
#define MPU_I2C_SLV0_REG		0x26
#define MPU_I2C_SLV0_CTRL		0x27
#define MPU_I2C_SLV1_ADDR		0x28
#define MPU_I2C_SLV1_REG		0x29
#define MPU_I2C_SLV1_CTRL		0x2A
#define MPU_I2C_SLV2_ADDR		0x2B
#define MPU_I2C_SLV2_REG		0x2C
#define MPU_I2C_SLV2_CTRL		0x2D

#define MPU_ACCEL_XOUT_H		0x3B
#define MPU_ACCEL_XOUT_L		0x3C
#define MPU_ACCEL_YOUT_H		0x3D
#define MPU_ACCEL_YOUT_L		0x3E
#define MPU_ACCEL_ZOUT_H		0x3F
#define MPU_ACCEL_ZOUT_L		0x40

#define MPU_ACCEL_OUT			MPU_ACCEL_XOUT_H

#define MPU_GYRO_XOUT_H			0x43
#define MPU_GYRO_XOUT_L			0x44
#define MPU_GYRO_YOUT_H			0x45
#define MPU_GYRO_YOUT_L			0x46
#define MPU_GYRO_ZOUT_H			0x47
#define MPU_GYRO_ZOUT_L			0x48

#define MPU_GYRO_OUT			MPU_GYRO_XOUT_H

#define MPU_PWR_MGMT_1			0x6B
#define MPU_WHO_AM_I			0x75


#define MPU_TIMEOUT	1000

#define MPU_ADDR_LSB	0b0
#define MPU_ADDR		((0b1101000 | MPU_ADDR_LSB) << 1)

#define MPU_USER_SMPLRT	1000  // frequency in hz
#define MPU_MAX_SMPLRT	8000
#define MPU_DIV			((uint8_t)8000/MPU_MAX_SMPLRT-1)
#define MPU_SMPLRT		(MPU_MAX_SMPLRT/(MPU_DIV+1))


#endif /* INC_MPU6500_H_ */
