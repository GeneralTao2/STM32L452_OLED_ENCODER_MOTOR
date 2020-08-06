/*
 * encoder.h
 *
 *  Created on: 13 июн. 2020 г.
 *      Author: ar725
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include "../Src/tools.h"

#define ENCODER_TUMBLER_PROGTIME_DELAY 50
#define ENCODER_BUTTON_PROGTIME_DELAY 1000
#define ENCODER_MAX_VAL 10
#define ENCODER_MIN_VAL -10

typedef struct EncoderConfigs {
	GPIO_PinConfigs CLK;
	GPIO_PinConfigs DT;
	GPIO_PinConfigs SW;

	/* Button state 0/1 */
	volatile uint8_t buttonState;

	/* Tumbler state 0/1 */
	volatile int16_t tumblerState;

	/* Counter for prevent contact of contacts of tumbler */
	volatile uint32_t tumblerProgTime;

	/* Counter for prevent contact of contacts of button */
	volatile uint32_t buttonProgTime;
} EncoderConfigs;

void EncoderInit(EncoderConfigs *configs);

#endif /* ENCODER_ENCODER_H_ */
