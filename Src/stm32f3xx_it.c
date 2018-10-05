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

extern ADC_HandleTypeDef hadc2;

extern uint8_t Level, temp_level;
extern enum State state;
extern enum Cell map[16][2];
extern struct Position player_pos, barrier_pos[10];

extern bool keypad_row[4];
extern char keypad_btn;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

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
			} else if (keypad_btn == 'C'){
				// OK
				if(temp_level <= 10 && temp_level > 0){
					state = Playing;
					Level = temp_level;
					generate_map();
				} else {
					level_error();
				}
			}
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
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
