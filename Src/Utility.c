/*
 * Utility.c - Some utility functions for STM32 Ali Gandomi
 *
 *  Created on: september 6, 2018
 *      Author: Ali Gandomi (gandomi110@gmail.com)
 */

#include "stm32f3xx_hal.h" // change this line accordingly
#include "Utility.h"
#include <stdbool.h>

void enable_keypad_intrrupt(bool * keypad_row){
	for(int i = 0; i < 4; i++){
		keypad_row[i] = 1;
	}
	write_keypad_row(keypad_row);
}

void write_keypad_row(bool * keypad_row){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, keypad_row[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, keypad_row[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, keypad_row[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3	, keypad_row[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool isNumber(char ch){
	if(ch >= 48 && ch <= 57)
		return true;
	return false;
}

bool isOperand(char ch){
	if(ch == 43 /* + */ || ch == 45 /* - */ ||  ch == 42 /* * */ ||  ch == 47 /* / */)
		return true;
	return false;
}

int char2int(char ch){
	return ch - '0';
}
