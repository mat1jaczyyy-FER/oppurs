/*
 * audio.c
 *
 *  Created on: Nov 13, 2023
 *      Author: mat1jaczyyy
 */

#include "stm32f4xx_hal.h"
#include "i2c.h"

#include "audio.h"

void init_AudioReset() {
	HAL_GPIO_WritePin(GPIOD, AUDIO_RESET_PIN, GPIO_PIN_SET);
}

void configAudio() {
	uint8_t bytes[2];
	init_AudioReset();

	/** Power sequence **/
	// Set Power Control Register to "on" state
	bytes[0] = 0x02;
	bytes[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	/** Initialization sequence **/
	bytes[0] = 0x00;
	bytes[1] = 0x99;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x47;
	bytes[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x32;
	bytes[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x32;
	bytes[1] = 0x0;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x00;
	bytes[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	/** Ctl registers configuration **/
	bytes[0] = 0x04;
	bytes[1] = 0xAF;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x0D;
	bytes[1] = 0x70;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x05;
	bytes[1] = 0x81;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x06;
	bytes[1] = 0x07;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x0A;
	bytes[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x27;
	bytes[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x1F;
	bytes[1] = 0x0F;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x22;
	bytes[1] = 0xC0;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x14;
	bytes[1] = 2;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x15;
	bytes[1] = 2;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x20;
	bytes[1] = 24;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes[0] = 0x21;
	bytes[1] = 24;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	/** Power up **/
	bytes[0] = 0x02;
	bytes[1] = 0x9E;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);
}
