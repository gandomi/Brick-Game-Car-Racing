/*
 * Utility.c - Some utility functions for STM32 Ali Gandomi
 *
 *  Created on: september 6, 2018
 *      Author: Ali Gandomi (gandomi110@gmail.com)
 */

#include "stm32f3xx_hal.h" // change this line accordingly
#include "Utility.h"
#include <stdbool.h>

void shift_right(bool *arr, int size){
	for (int i = size - 1; i > 0; i--){        
    arr[i]=arr[i-1];
	}
	arr[0] = 0;
}

char find_button(bool *keypad_row, int size){
	
	char keypad[4][4] = { {'7', '8', '9', '/'},
												{'4', '5', '6', '*'},
												{'1', '2', '3', '-'},
												{'C', '0', '=', '+'}};
	
	// Reset three last rows
	for(int i = 1; i < size; i++)
	{
		keypad_row[i] = 0;
	}
	write_keypad_row(keypad_row);
	HAL_Delay(4);
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1){
		// S1 clicked
		return keypad[0][0];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 1){
		// S2 clicked
		return keypad[0][1];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1){
		// S3 clicked
		return keypad[0][2];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 1){
		// S4 clicked
		return keypad[0][3];
	}
	// END OF CHECK ROW 1
	
	
	// CHECK ROW 2
	
	//SHIFT OUTPUT
	shift_right(keypad_row, 4);
	write_keypad_row(keypad_row);
	HAL_Delay(4);
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1){
		// S5 clicked
		return keypad[1][0];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 1){
		// S6 clicked
		return keypad[1][1];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1){
		// S7 clicked
		return keypad[1][2];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 1){
		// S8 clicked
		return keypad[1][3];
	}
	// END OF CHECK ROW 2
	
	
	// CHECK ROW 3
	
	//SHIFT OUTPUT
	shift_right(keypad_row, 4);
	write_keypad_row(keypad_row);
	HAL_Delay(4);
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1){
		// S9 clicked
		return keypad[2][0];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 1){
		// S10 clicked
		return keypad[2][1];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1){
		// S11 clicked
		return keypad[2][2];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 1){
		// S12 clicked
		return keypad[2][3];
	}
	// END OF CHECK ROW 3
	
	// CHECK ROW 4
	
	//SHIFT OUTPUT
	shift_right(keypad_row, 4);
	write_keypad_row(keypad_row);
	HAL_Delay(4);
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1){
		// S13 clicked
		return keypad[3][0];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 1){
		// S14 clicked
		return keypad[3][1];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1){
		// S15 clicked
		return keypad[3][2];
	} else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 1){
		// S16 clicked
		return keypad[3][3];
	}
	// END OF CHECK ROW 4
	
	return 'e';
}

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

void print_life_on_led(void){
	
	uint16_t gpio_pin_x = 0x0100U;
	
	for(int i = 0; i < Life; i++){
		HAL_GPIO_WritePin(GPIOE, gpio_pin_x, GPIO_PIN_SET);
		gpio_pin_x = gpio_pin_x << 1;
		HAL_Delay(200);
	}
}

void print_get_level(void){
	clear();
	print("Level[1-10]:");
	setCursor(0, 1);
	print("C -> START");
	setCursor(13, 0);
}
