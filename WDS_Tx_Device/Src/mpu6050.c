/*
 * mpu6050.c
 *
 *  Created on: 09.04.2019
 *      Author: Kurat
 */

#include "mpu6050.h"
#include "mpu6050_registers.h"


#include "common.h"

#define IS_MPU6050_ADDR(ADDR) 	(((ADDR) == MPU6050_Device_0) || \
								((ADDR) == MPU6050_Device_1))

typedef enum
{
	STATUS_OK = 0,
	STATUS_TIMEOUT,
	STATUS_ERROR
}Status_t;

Status_t WaitUntilReady(mpu6050_t *hmpu, uint32_t timeout)
{
	uint32_t tickStart = HAL_GetTick();

	while(  (HAL_GetTick() - tickStart) <= timeout )
	{
		if( HAL_I2C_GetState(hmpu->i2c_handler) == HAL_I2C_STATE_READY )
		{
			return STATUS_OK;
		}
	}
	return STATUS_TIMEOUT;
}

MPU6050_Status MPU6050_Write(mpu6050_t *hmpu, uint8_t tx_data[], uint32_t tx_data_length, uint32_t timeout)
{
	if(WaitUntilReady(hmpu, timeout) != STATUS_OK)
	{
		return MPU6050_Status_Error;
	}

	if(HAL_I2C_Master_Transmit(hmpu->i2c_handler, hmpu->dev_address, tx_data, tx_data_length, timeout) != HAL_OK){
		return MPU6050_Status_Error;
	}

	return MPU6050_Status_Ok;
}
MPU6050_Status MPU6050_Read(mpu6050_t *hmpu, uint8_t *rx_data, uint32_t rx_data_length, uint32_t timeout)
{

	if(WaitUntilReady(hmpu, timeout) != STATUS_OK)
	{
		return MPU6050_Status_Error;
	}

	if(HAL_I2C_Master_Receive(hmpu->i2c_handler, hmpu->dev_address,
			rx_data, rx_data_length, timeout) ){
		return MPU6050_Status_Error;
	}

	return MPU6050_Status_Ok;
}
MPU6050_Status MPU6050_Read_it(mpu6050_t *hmpu, uint8_t *rx_data, uint32_t rx_data_length, uint32_t timeout)
{
	if(WaitUntilReady(hmpu, timeout) != STATUS_OK)
	{
		return MPU6050_Status_Error;
	}

	if(HAL_I2C_Master_Receive_IT(hmpu->i2c_handler, hmpu->dev_address, rx_data, rx_data_length) != HAL_OK)
	{
		return MPU6050_Status_Error;
	}
	return MPU6050_Status_Ok;
}

MPU6050_Status mpu6050_init(mpu6050_t *hmpu, I2C_HandleTypeDef *hi2c,  MPU6050_Configuraton configuration)
{
	if(hmpu == NULL || hi2c == NULL){
		return MPU6050_Status_Error;
	}

	hmpu->i2c_handler = hi2c;
	hmpu->dev_address = MPU6050_I2C_ADDR | configuration.device;

	assert_param(IS_MPU6050_ADDR(hmpu->dev_address));

	// Initialize MPU6050 device

	MPU6050_Status status;
	uint8_t rx_tmp;
	uint8_t who_am_i = MPU6050_WHO_AM_I;

	 if( (status = MPU6050_Write(hmpu, &who_am_i, 1, 1000)) != MPU6050_Status_Ok){
		 return status;
	 }

	 if( (status = MPU6050_Read(hmpu, &rx_tmp, 1, 1000)) != MPU6050_Status_Ok){
		 return status;
	 }

	if(rx_tmp != MPU6050_I_AM){
		return MPU6050_Status_DeviceInvalid;
	}


	// configure the device
	// todo: error check
	MPU6050_Set_SampleRate(hmpu, configuration.sample_rate);
	MPU6050_Set_Accelerometer(hmpu, configuration.accelerometer);
	MPU6050_Set_Gyroscope(hmpu, configuration.gyroscope);

	return MPU6050_Status_Ok;
}
MPU6050_Status mpu6050_start(mpu6050_t *hmpu)
{
	// Wakeup MPU6050
	MPU6050_Status status;
	uint8_t tx_data[2];


	tx_data[0] = MPU6050_PWR_MGMT_1;
	tx_data[1] = 0x00;

	status = MPU6050_Write(hmpu, tx_data, 2, 1000);
	return status;
}

MPU6050_Status MPU6050_Set_SampleRate(mpu6050_t *hmpu, MPU6050_SampleRate sampleRate)
{
	MPU6050_Status status;
	uint8_t tx_data[2];

	tx_data[0] = MPU6050_SMPLRT_DIV;
	tx_data[1] = sampleRate;

	status = MPU6050_Write(hmpu, tx_data, 2, 1000);
	return status;
}
MPU6050_Status MPU6050_Set_Accelerometer(mpu6050_t *hmpu, MPU6050_Accelerometer accelerometer)
{
	MPU6050_Status status;
	uint8_t accel_config = MPU6050_ACCEL_CONFIG;
	uint8_t accel_config_value;

	if( (status = MPU6050_Write(hmpu, &accel_config, 1, 1000)) != MPU6050_Status_Ok){
		return status;
	}

	if( (status = MPU6050_Read(hmpu, &accel_config_value, 1, 1000)) != MPU6050_Status_Ok){
		return status;
	}

	accel_config_value = (accel_config_value & 0b11100111)
			| (accelerometer << 3);

	uint8_t tx_data[2];
	tx_data[0] = MPU6050_ACCEL_CONFIG;
	tx_data[1] = accel_config_value;

	if( (status = MPU6050_Write(hmpu, &tx_data, 2, 1000)) != MPU6050_Status_Ok)
	{
		return status;
	}

	return MPU6050_Status_Ok;
}
MPU6050_Status MPU6050_Set_Gyroscope(mpu6050_t *hmpu, MPU6050_Gyroscope gyroscope)
{
	MPU6050_Status status;
	uint8_t gyro_config = MPU6050_GYRO_CONFIG;
	uint8_t gyro_config_value;


	if( (status = MPU6050_Write(hmpu, &gyro_config, 1, 1000)) != MPU6050_Status_Ok){
		return status;
	}

	if( (status = MPU6050_Read(hmpu, &gyro_config_value, 1, 1000)) != MPU6050_Status_Ok){
		return status;
	}

	gyro_config_value = (gyro_config_value & 0b11100111)
			| (gyroscope << 3);

	uint8_t tx_data[2];
	tx_data[0] = MPU6050_GYRO_CONFIG;
	tx_data[1] = gyro_config_value;
	if( (status = MPU6050_Write(hmpu, &tx_data, 2, 1000)) != MPU6050_Status_Ok)
	{
		return status;
	}

	return MPU6050_Status_Ok;
}

MPU6050_Status MPU6050_Read_Accelerometer(mpu6050_t *hmpu, int16_t data_out[])
{
	MPU6050_Status status;
	uint8_t accel_out_register = MPU6050_ACCEL_XOUT_H;
	uint8_t rx_data[6];


	if( (status = MPU6050_Write(hmpu, &accel_out_register, 1, 1000)) != MPU6050_Status_Ok){
		return status;
	}

	if( (status = MPU6050_Read_it(hmpu, rx_data, 6, 1000)) != MPU6050_Status_Ok){
		return status;
	}

	if (HAL_I2C_Master_Receive(hmpu->i2c_handler, hmpu->dev_address,
			rx_data, 6, 1000) != HAL_OK) {
		return MPU6050_Status_Error;
	}

	return MPU6050_Status_Ok;
}
