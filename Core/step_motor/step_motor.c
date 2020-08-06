/*
 * step_motor.c
 *
 *  Created on: Jun 6, 2020
 *      Author: ar725
 */
#include "step_motor.h"

/* Sets the state of the step motor by ruling on 4 сoils */
void StepMotorWrite(StepMotorConfigs *configs, GPIO_PinState ps1,
												GPIO_PinState ps2,
												GPIO_PinState ps3,
												GPIO_PinState ps4) {
	HAL_GPIO_WritePin(configs->p1.GPIO, configs->p1.PIN, ps1);
	HAL_GPIO_WritePin(configs->p2.GPIO, configs->p2.PIN, ps2);
	HAL_GPIO_WritePin(configs->p3.GPIO, configs->p3.PIN, ps3);
	HAL_GPIO_WritePin(configs->p4.GPIO, configs->p4.PIN, ps4);
}

/* Alternates between 4 states of the step motor:
 * 1: 1 0 0 0
 * 2: 0 1 0 0
 * 3: 0 0 1 0
 * 4: 0 0 0 1
 */
void StepMotorRotate(StepMotorConfigs *configs) {
	switch(configs->phase) {
	case 1:
		StepMotorWrite(configs, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
	break;
	case 2:
		StepMotorWrite(configs, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
	break;
	case 3:
		StepMotorWrite(configs, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
	break;
	case 4:
		StepMotorWrite(configs, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
	break;
	default:
		StepMotorWrite(configs, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
	}

	if(configs->direction == 1) {
		configs->phase++;
	} else {
		configs->phase--;
	}
	if(configs->phase >= 5) {
		configs->phase = 1;
	}
	if(configs->phase <= 0) {
		configs->phase = 4;
	}
}

