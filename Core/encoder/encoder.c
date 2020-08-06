/*
 * encoder.c
 *
 *  Created on: 13 июн. 2020 г.
 *      Author: ar725
 */

#include "encoder.h"

/* Initialize buttonState, tumblerState, tumblerProgTime,
 * buttonProgTime of encoder
 */
void EncoderInit(EncoderConfigs *configs) {
	configs->buttonState = 0;
	configs->tumblerState = 0;
	configs->tumblerProgTime = HAL_GetTick();
	configs->buttonProgTime = HAL_GetTick();
}
