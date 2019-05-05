/*
 * mpu6050.h
 *
 *  Created on: 09.04.2019
 *      Author: Kurat
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include "stm32f1xx_hal.h"

typedef enum{
	MPU6050_Status_Ok = 0x00,
	MPU6050_Status_Error,
	MPU6050_Status_DeviceNC,
	MPU6050_Status_DeviceInvalid
}MPU6050_Status;

typedef enum{
	MPU6050_Device_0 = 0x00,	/*!< AD0 pin is set to low */
	MPU6050_Device_1 = 0x01		/*!< AD0 pin is set to high */
}MPU6050_Device;

typedef enum{
	MPU6050_Accelerometer_2G 	= 0x00,
	MPU6050_Accelerometer_4G 	= 0x01,
	MPU6050_Accelerometer_8G 	= 0x02,
	MPU6050_Accelerometer_16G	= 0x03
}MPU6050_Accelerometer;

typedef enum{
	MPU6050_Gyroscope_250s 	= 0x00,
	MPU6050_Gyroscope_500s 	= 0x01,
	MPU6050_Gyroscope_1000s 	= 0x02,
	MPU6050_Gyroscope_2000s 	= 0x03
}MPU6050_Gyroscope;

typedef enum{
	MPU6050_SampleRate_100Hz 	= 79,
	MPU6050_SampleRate_125Hz 	= 63,
	MPU6050_SampleRate_250Hz 	= 31,
	MPU6050_SampleRate_500Hz 	= 15,
	MPU6050_SampleRate_1KHz 	= 7,
	MPU6050_SampleRate_2KHz		= 3,
	MPU6050_SampleRate_4KHz		= 1,
	MPU6050_SampleRate_8KHz		= 0
}MPU6050_SampleRate;

typedef enum{
	MPU6050_InterruptFlag_FIFO_OFLOW_EN = 0x10,
	MPU6050_InterruptFlag_I2C_MST_INT_EN = 0x08,
	MPU6050_InterruptFlag_DATA_RDY_EN = 0x01
}MPU6050_InterruptFlag;

typedef struct
{
	MPU6050_Device device;
	MPU6050_SampleRate sample_rate;
	MPU6050_Accelerometer accelerometer;
	MPU6050_Gyroscope gyroscope;
}MPU6050_Configuraton;

typedef struct
{
	I2C_HandleTypeDef *i2c_handler;
	uint8_t dev_address;
	MPU6050_Device device;
}mpu6050_t;


MPU6050_Status mpu6050_init(mpu6050_t *hmpu, I2C_HandleTypeDef *hi2c,  MPU6050_Configuraton configuration);
MPU6050_Status mpu6050_start(mpu6050_t *hmpu);

MPU6050_Status MPU6050_Set_SampleRate(mpu6050_t *hmpu, MPU6050_SampleRate sampleRate);
MPU6050_Status MPU6050_Set_Accelerometer(mpu6050_t *hmpu, MPU6050_Accelerometer accelerometer);
MPU6050_Status MPU6050_Set_Gyroscope(mpu6050_t *hmpu, MPU6050_Gyroscope gyroscope);

MPU6050_Status MPU6050_Read_Accelerometer(mpu6050_t *hmpu, int16_t data_out[]);

MPU6050_Status MPU6050_Interrupt_Set(mpu6050_t *hmpu, MPU6050_InterruptFlag int_flag);
#endif /* MPU6050_H_ */
