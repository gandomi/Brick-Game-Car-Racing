/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
#include "LiquidCrystal.h"
#include "Utility.h"
#include <stdbool.h>
#include <stdlib.h>

void level_error(void);
void generate_map(void);
void clear_map(void);
void print_map(void);
void print_level_on_7seg(void);
void print_to_7447(char num);
void player_RightLeft_move(void);
void Lose(void);
void Win(void);
void Game_Over(void);
void decrement_life(void);
void turn_off_lost_life_led(void);
bool is_Game_over(void);
uint16_t life_to_pin_number_cumulative(void);
uint16_t life_to_pin_number(void);

extern ADC_HandleTypeDef hadc2;

extern uint8_t Life, Level, temp_level;
extern enum State state;
extern enum Playing_State playing_state;
extern enum Cell map[16][2];
extern struct Position player_pos, barrier_pos[10];
// counters
extern uint8_t counter_7segment;
extern uint16_t counter_player_move;

extern bool keypad_row[4];
extern char keypad_btn; int counter = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if(state == Getting_Level){
		if(temp_level <= 10 && temp_level > 0){
			state = Playing;
			Level = temp_level;
			generate_map();
			// Start 1ms timer
			HAL_TIM_Base_Start_IT(&htim2);
		} else {
			level_error();
		}
	} else if(state == Playing && playing_state == Pause) {
		// Right/Left move
		playing_state = Play;
	} else if(state == Playing && playing_state == Play) {
		// Right/Left move
		player_RightLeft_move();
	} else {
			// Other state handler
	}
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	keypad_btn = find_button(keypad_row, 4);
	if(keypad_btn != 'e') {
		if(state == Getting_Level_Error){
			temp_level = 0;
			state = Getting_Level;
			setCursor(13, 0);
			print("   ");
			setCursor(13, 0);
		}
		if(state == Getting_Level){
			if(isNumber(keypad_btn)){
				temp_level = temp_level * 10 + char2int(keypad_btn);
				write(keypad_btn);
			} 
//			else if (keypad_btn == 'C'){
//				// OK
//				if(temp_level <= 10 && temp_level > 0){
//					state = Playing;
//					Level = temp_level;
//					generate_map();
//					// Start 1ms timer
//					HAL_TIM_Base_Start_IT(&htim2);
//				} else {
//					level_error();
//				}
//			}
		} else {
				// Other state handler
		}
	} else {
		// error
	}
	HAL_Delay(50);
	enable_keypad_intrrupt(keypad_row);
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	
	/*
	 * Add counters
	 */
	if(state == Playing)
		counter_7segment++;
	if(state == Playing && playing_state == Play)
		counter_player_move++;
	
	/*
	 * Check events
	 */
	if(state == Playing && counter_7segment == 5){
		// print level on 7 seg
		print_level_on_7seg();
		counter_7segment = 0;
	}
	if(state == Playing && playing_state == Play && counter_player_move == 500 + ((11 - Level) * 50)){
		// Move player
		counter_player_move = 0;
	}
	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void level_error(void){
	state = Getting_Level_Error;
	setCursor(13, 0);
	print("ERR");
}

void generate_map(void){
	clear_map();
	
	/*
	 * Random player position
	 */
	uint32_t adc2_value; // for seed
	HAL_ADC_Start(&hadc2);
	HAL_Delay(1);
	adc2_value = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	srand(adc2_value); // seed
	
	player_pos.row = 0;
	player_pos.col = rand() % 2;
	map[player_pos.row][player_pos.col] = player;
	
	/*
	 * Random barrier position
	 */
	do {
		for(int i = 0; i < Level; i++){
			barrier_pos[i].row = (rand() % 15) + 1;
			barrier_pos[i].col = rand() % 2;
			
			// Check not repetitive
			for(int j = 0; j < i; j++){
				if(barrier_pos[j].row == barrier_pos[i].row && barrier_pos[j].col == barrier_pos[i].col){
					barrier_pos[i].row = (rand() % 15) + 1;
					barrier_pos[i].col = rand() % 2;
					
					j = 0;
				}
			}
		}
	} while(false /* Map not valid */);
	
	/*
	 * Copy barriers to map
	 */
	for(int i = 0; i < Level; i++){
		map[barrier_pos[i].row][barrier_pos[i].col] = barrier;
	}
	
	/*
	 * print initial map
	 */
	print_map();
}

void clear_map(void){
	for(uint8_t i = 0; i < 16; i++){
		map[i][0] = map[i][1] = empty;
	}
}

void print_map(void){
	clear();
	for(int col = 1; col >= 0; col--){
		for(int row = 15; row >= 0; row--){
			if(map[row][col] == empty)
				print(" ");
			else if(map[row][col] == barrier)
				write(1);
			else
				write(0);
		}
		setCursor(0, 1);
		
//		char str[16];
//		sprintf(str, "%d", player_pos.col);
//		print(str);
	}
}

void print_level_on_7seg(void){
	// print Digit 1
	temp_level = Level;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
	print_to_7447(temp_level % 10);
	temp_level /= 10;
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
	
	// print Digit 2
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
	print_to_7447(temp_level % 10);
	temp_level /= 10;
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
}

void print_to_7447(char num){
	char IC7447[4] = { 0 };
	for(int i = 0; i < 4; i++)
	{
		if(num % 2 == 1)
			IC7447[i] = 1;
		num /= 2;
	}
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, IC7447[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, IC7447[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, IC7447[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, IC7447[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void player_RightLeft_move(void){
	struct Position new_player_pos;
	new_player_pos.row = player_pos.row;
	new_player_pos.col = 1 - player_pos.col;
	
	if(map[new_player_pos.row][new_player_pos.col] == barrier){
		Lose();
	}
}

void Lose(){
	decrement_life();
	if(is_Game_over()){
		Game_Over();
	}
}

void Win(){
	
}

void Game_Over(){
	
}

void decrement_life(){
	turn_off_lost_life_led();
	Life--;
}

void turn_off_lost_life_led(){
	// Turn Off with some visual effect
	HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_RESET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_SET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_RESET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_SET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_RESET);
	HAL_Delay(300);
}

bool is_Game_over(){
	return Life == 0;
}

uint16_t life_to_pin_number_cumulative(){
	switch(Life){
		case 1:
			return (uint16_t)0x0100U;
		
		case 2:
			return (uint16_t)0x0300U;
		
		case 3:
			return (uint16_t)0x0700U;
		
		case 4:
			return (uint16_t)0x0F00U;
		
		case 5:
			return (uint16_t)0x1F00U;
		
		case 6:
			return (uint16_t)0x3F00U;
		
		case 7:
			return (uint16_t)0x7F00U;
		
		case 8:
			return (uint16_t)0xFF00U;
		
		default:
			return (uint16_t)0x0000U;
	}
}

uint16_t life_to_pin_number(){
	switch(Life){
		case 1:
			return (uint16_t)0x0100U;
		
		case 2:
			return (uint16_t)0x0200U;
		
		case 3:
			return (uint16_t)0x0400U;
		
		case 4:
			return (uint16_t)0x0800U;
		
		case 5:
			return (uint16_t)0x1000U;
		
		case 6:
			return (uint16_t)0x2000U;
		
		case 7:
			return (uint16_t)0x4000U;
		
		case 8:
			return (uint16_t)0x8000U;
		
		default:
			return (uint16_t)0x0000U;
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
