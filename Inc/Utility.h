/**
******************************************************************************
* @file           : Utility.h
* @brief          : Header for Utility.c file.
*                   This file contains some utility functions.
******************************************************************************
** This notice applies to any and all portions of this file
*
* COPYRIGHT(c) 2018 Ali Gandomi
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UTILITY_H__
#define __UTILITY_H__

/* Includes ------------------------------------------------------------------*/
#include "LiquidCrystal.h"
#include <stdbool.h>

char find_button(bool *keypad_row, int size);
void shift_right(bool *arr, int size);
void enable_keypad_intrrupt(bool * keypad_row);
void write_keypad_row(bool * keypad_row);
bool isNumber(char ch);
bool isOperand(char ch);
int char2int(char ch);
void print_life_on_led(void);
void print_get_level(void);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern uint8_t Life;

enum State {Idle, Getting_Level, Getting_Level_Error, Playing, GameOver, Finish};
enum Playing_State {Play, Pause};
enum Cell {empty, barrier, player}; 
struct Position 
{ 
   uint8_t row, col; 
};

#endif /* __UTILITY_H__ */

/************************ (C) COPYRIGHT Ali Gandomi *****END OF FILE****/
