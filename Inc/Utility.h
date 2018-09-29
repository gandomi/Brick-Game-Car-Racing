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
#include <stdbool.h>

char find_button(bool *keypad_row, int size);
void shift_right(bool *arr, int size);
void enable_keypad_intrrupt(bool * keypad_row);
void write_keypad_row(bool * keypad_row);
bool isNumber(char ch);
bool isOperand(char ch);
int char2int(char ch);
enum State {Getting_Level, Getting_Level_Error, Playing};

#endif /* __UTILITY_H__ */

/************************ (C) COPYRIGHT Ali Gandomi *****END OF FILE****/
