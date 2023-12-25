/*
 * audio.h
 *
 *  Created on: Nov 13, 2023
 *      Author: mat1jaczyyy
 */

#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#define AUDIO_RESET_PIN		GPIO_PIN_4
#define AUDIO_I2C_ADDRESS	0x94

void configAudio();

#endif /* INC_AUDIO_H_ */
