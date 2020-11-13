///*
// * mpu6500.c
// *
// *  Created on: Oct 13, 2020
// *      Author: rochi
// */
//
//
//#include "mpu6500.h"
//
//
//
//static void * hi2c;
//
////HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef * hi2c1)
////{
////	hi2c = hi2c1;
////
////	unsigned char buffer[1];
////
////
////	// check if mpu is available
////	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_WHO_AM_I, 1, buffer, 1, MPU_TIMEOUT);
////
////	if (status == HAL_OK) {
////		// HAL_OK == 0, so we perform or with all outputs
////
////		// power up mpu
////		buffer[0] = 0;
////		status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_PWR_MGMT_1, 1, buffer, 1, MPU_TIMEOUT);
////
////		if (status == HAL_OK) {
////		// set sampling rate
////			buffer[0] = MPU_SMPLRT_DIV;
////			status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_SMPLRT_DIV, 1, buffer, 1, MPU_TIMEOUT);
////
////			if (status == HAL_OK) {
////				// configure accelerometer
////				buffer[0] = 0;
////				status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_ACCEL_CONFIG, 1, buffer, 1, MPU_TIMEOUT);
////
////				if (status == HAL_OK) {
////					// configure gyroscope
////					status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_GYRO_CONFIG, 1, buffer, 1, MPU_TIMEOUT);
////				}
////			}
////		}
////	}
////
////	return status;
////}
////
////
////
////
////HAL_StatusTypeDef MPU_ReadData(MPU_data_t * d)
////{
////	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
////	HAL_StatusTypeDef status = HAL_ERROR;
////
////	if (d != NULL) {
////		unsigned char buffer[6];
////		status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_ACCEL_OUT, 1, buffer, 6, MPU_TIMEOUT);
////
////		if (status == HAL_OK) {
////			int16_t accel_X_RAW = (int16_t)(buffer[0] << 8 | buffer [1]);
////			int16_t accel_Y_RAW = (int16_t)(buffer[2] << 8 | buffer [3]);
////			int16_t accel_Z_RAW = (int16_t)(buffer[4] << 8 | buffer [5]);
////
////			/*** convert the RAW values into acceleration in 'g'
////				 we have to divide according to the Full scale value set in FS_SEL
////				 I have configured FS_SEL = 0. So I am dividing by 16384.0
////				 for more details check ACCEL_CONFIG Register              ****/
////
////			d->accel.x = accel_X_RAW/16384.0;  // get the float g
////			d->accel.y = accel_Y_RAW/16384.0;
////			d->accel.z = accel_Z_RAW/16384.0;
////
////			status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_GYRO_OUT, 1, buffer, 6, MPU_TIMEOUT);
////
////			if (status == HAL_OK) {
////
////				int16_t Gyro_X_RAW = (int16_t)(buffer[0] << 8 | buffer [1]);
////				int16_t Gyro_Y_RAW = (int16_t)(buffer[2] << 8 | buffer [3]);
////				int16_t Gyro_Z_RAW = (int16_t)(buffer[4] << 8 | buffer [5]);
////
////				/*** convert the RAW values into dps (°/s)
////					 we have to divide according to the Full scale value set in FS_SEL
////					 I have configured FS_SEL = 0. So I am dividing by 131.0
////					 for more details check GYRO_CONFIG Register              ****/
////
////				d->gyro.x = Gyro_X_RAW/131.0;
////				d->gyro.y = Gyro_Y_RAW/131.0;
////				d->gyro.z = Gyro_Z_RAW/131.0;
////			}
////		}
////	}
////
////	return status;
////}
//
//bool MPU_Init(void * hi2c1)
//{
//	hi2c = hi2c1;
//
//	unsigned char buffer[1];
////
////	// check if mpu is available
////	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_WHO_AM_I, 1, buffer, 1, MPU_TIMEOUT);
////
////	if (status == HAL_OK) {
////		// HAL_OK == 0, so we perform or with all outputs
////
////		// power up mpu
////		buffer[0] = 0;
////		status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_PWR_MGMT_1, 1, buffer, 1, MPU_TIMEOUT);
////
////		if (status == HAL_OK) {
////		// set sampling rate
////			buffer[0] = MPU_SMPLRT_DIV;
////			status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_SMPLRT_DIV, 1, buffer, 1, MPU_TIMEOUT);
////
////			if (status == HAL_OK) {
////				// configure accelerometer
////				buffer[0] = 0;
////				status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_ACCEL_CONFIG, 1, buffer, 1, MPU_TIMEOUT);
////
////				if (status == HAL_OK) {
////					// configure gyroscope
////					status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_GYRO_CONFIG, 1, buffer, 1, MPU_TIMEOUT);
////				}
////			}
////		}
////	}
//
//	return true;
//}
//
//
//
//
//bool MPU_ReadData(MPU_data_t * d)
//{
//	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
////	HAL_StatusTypeDef status = HAL_ERROR;
////
////	if (d != NULL) {
////		unsigned char buffer[6];
////		status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_ACCEL_OUT, 1, buffer, 6, MPU_TIMEOUT);
////
////		if (status == HAL_OK) {
////			int16_t accel_X_RAW = (int16_t)(buffer[0] << 8 | buffer [1]);
////			int16_t accel_Y_RAW = (int16_t)(buffer[2] << 8 | buffer [3]);
////			int16_t accel_Z_RAW = (int16_t)(buffer[4] << 8 | buffer [5]);
////
////			/*** convert the RAW values into acceleration in 'g'
////				 we have to divide according to the Full scale value set in FS_SEL
////				 I have configured FS_SEL = 0. So I am dividing by 16384.0
////				 for more details check ACCEL_CONFIG Register              ****/
////
////			d->accel.x = accel_X_RAW/16384.0;  // get the float g
////			d->accel.y = accel_Y_RAW/16384.0;
////			d->accel.z = accel_Z_RAW/16384.0;
////
////			status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_GYRO_OUT, 1, buffer, 6, MPU_TIMEOUT);
////
////			if (status == HAL_OK) {
////
////				int16_t Gyro_X_RAW = (int16_t)(buffer[0] << 8 | buffer [1]);
////				int16_t Gyro_Y_RAW = (int16_t)(buffer[2] << 8 | buffer [3]);
////				int16_t Gyro_Z_RAW = (int16_t)(buffer[4] << 8 | buffer [5]);
////
////				/*** convert the RAW values into dps (°/s)
////					 we have to divide according to the Full scale value set in FS_SEL
////					 I have configured FS_SEL = 0. So I am dividing by 131.0
////					 for more details check GYRO_CONFIG Register              ****/
////
////				d->gyro.x = Gyro_X_RAW/131.0;
////				d->gyro.y = Gyro_Y_RAW/131.0;
////				d->gyro.z = Gyro_Z_RAW/131.0;
////			}
////		}
////	}
//
//	//return status;
//	return true;
//}
//



