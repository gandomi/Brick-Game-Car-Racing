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
void print_player_move(void);
void print_win_all_levels(void);
void player_Forward_move(void);
void player_RightLeft_move(void);
void Lose(void);
void Win(void);
void Game_Over(void);
void All_Levels_passed_successfully(void);
void decrement_life(void);
void turn_off_a_LED(bool effect);
bool is_Game_over(void);
uint16_t life_to_pin_number_cumulative(void);
uint16_t life_to_pin_number(void);
bool validate_map(void);
void reset_all_counters(void);
void Stop_LEDs(void);
void Start_LEDs(void);
void Set_date_time(uint8_t * datetime);
void Calculate_forward_move_time(void);

extern uint8_t Life, Level, temp_level, volume;
extern enum State state;
extern enum Playing_State playing_state;
extern enum Cell map[16][2];
extern uint8_t UART_Data[1], UART_Command[25], UART_position;
extern struct Position player_pos, initial_player_pos, new_player_pos, barrier_pos[10];
// counters
extern uint8_t counter_7segment;
extern uint16_t counter_player_move, counter_treasure, counter_blink, forward_move_time;
extern uint32_t score;
extern RTC_TimeTypeDef myTime;
extern RTC_DateTypeDef myDate;

extern RTC_HandleTypeDef hrtc;

extern bool keypad_row[4];
extern char keypad_btn;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;

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
			HAL_TIM_Base_Start_IT(&htim4);
		} else {
			level_error();
		}
	} else if(state == Playing && playing_state == Pause) {
		// Right/Left move
		playing_state = Play;
		Start_LEDs();
	} else if(state == Playing && playing_state == Play) {
		// Right/Left move
		player_RightLeft_move();
	} else if(state == GameOver) {
		state = Getting_Level;
		print_get_level();
		Life = 8;
		print_life_on_led();
	} else if(state == Idle) {
		state = Getting_Level;
		print_get_level();
		Life = 8;
		print_life_on_led();
	} else {
			// Other state handler
	}
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	uint8_t LDR_value = HAL_ADC_GetValue(&hadc1);
	int noor_percent = (LDR_value*100)/25;
	noor_percent = noor_percent < 100 ? noor_percent : 100;
//	char value[16];
//	sprintf(value, "Noor: %d%%    ", noor_percent);
//	home();
//	print(value);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, noor_percent);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, noor_percent);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, noor_percent);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, noor_percent);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, noor_percent);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, noor_percent);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, noor_percent);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, noor_percent);
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
	HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END ADC1_2_IRQn 1 */
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
//					HAL_TIM_Base_Start_IT(&htim4);
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
//	if(state == Playing)
//		counter_7segment++;
	if(state == Playing && playing_state == Play)
		counter_player_move++;
	if(state == Finish)
		counter_treasure++;
	if(state == Playing && playing_state == Pause)
		counter_blink++;
	
	/*
	 * Check events
	 */
//	if(state == Playing && counter_7segment == 5){
//		// print level on 7 seg
//		print_level_on_7seg();
//		counter_7segment = 0;
//	}
	if(state == Playing && playing_state == Play && counter_player_move >= forward_move_time){
		// Move player
		player_Forward_move();
		counter_player_move = 0;
	}
	if(state == Finish && counter_treasure == 2000){
		state = Idle;
		
		counter_treasure = 0;
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
		/* 
	 * Stop 1ms timers
	 */
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim4);
	}
	if(state == Playing && playing_state == Pause && counter_blink == 1){ // TODO: FIX the bug of LED 8 14 15
		Stop_LEDs();
	}
	if(state == Playing && playing_state == Pause && counter_blink == 401){
		Start_LEDs();
	}
	if(counter_blink >= 800)
		counter_blink = 0;
	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	/*
	 * This Timer is for 7-segment only
	 */
	if(state == Playing)
		counter_7segment++;

	/*
	 * Check events
	 */
	if((state == Playing && playing_state == Play && counter_7segment == 5) || (state == Playing && playing_state == Pause && counter_blink <= 400 && counter_7segment == 5)){
		// print level on 7 seg
		print_level_on_7seg();
		counter_7segment = 0;
	}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	
  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
//	clear();
//	write(UART_Data[0]);
	UART_Command[UART_position++] = UART_Data[0];
	
	if(UART_position == 4 && UART_Command[0] == 'p' && UART_Command[1] == 'l' && UART_Command[2] == 'a' && UART_Command[3] == 'y'){
		// play
		playing_state = Play;
		
		/*
		 * Enable LED if stoped
		 */
		Start_LEDs();
		
		UART_position = 0;
	} else if(UART_position == 5 && UART_Command[0] == 'p' && UART_Command[1] == 'a' && UART_Command[2] == 'u' && UART_Command[3] == 's' && UART_Command[4] == 'e'){
		// pause
		playing_state = Pause;
	
		UART_position = 0;
	} else if(UART_position == 5 && UART_Command[0] == 'r' && UART_Command[1] == 'e' && UART_Command[2] == 's' && UART_Command[3] == 'e' && UART_Command[4] == 't'){
		// reset
		new_player_pos.row = initial_player_pos.row;
		new_player_pos.col = initial_player_pos.col;
		print_player_move();
		player_pos.row = initial_player_pos.row;
		player_pos.col = initial_player_pos.col;
	
		UART_position = 0;
		score = 0;
	} else if(UART_position == 25 && UART_Command[0] == 'R' && UART_Command[1] == 'T' && UART_Command[2] == 'C'){
		// Set Date & Time
		Set_date_time(UART_Command + 4);
	}
	
	if(UART_position >= 25 || (UART_position >= 5 && !(UART_Command[0] == 'R' && UART_Command[1] == 'T' && UART_Command[2] == 'C'))){
		UART_position = 0;
	}
	
	HAL_UART_Receive_IT(&huart3, UART_Data, sizeof(UART_Data));
  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles ADC4 interrupt.
*/
void ADC4_IRQHandler(void)
{
  /* USER CODE BEGIN ADC4_IRQn 0 */
	volume = HAL_ADC_GetValue(&hadc4);
	Calculate_forward_move_time();
//	char value[16];
//	sprintf(value, "Volume: %d%%   ", volume);
//	home();
//	print(value);
  /* USER CODE END ADC4_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc4);
  /* USER CODE BEGIN ADC4_IRQn 1 */
	HAL_ADC_Start_IT(&hadc4);
  /* USER CODE END ADC4_IRQn 1 */
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
	
	initial_player_pos.row = player_pos.row = 0;
	initial_player_pos.col = player_pos.col = rand() % 2;
	map[player_pos.row][player_pos.col] = player;
	
	/*
	 * Random barrier position
	 */
	for(int i = 0; i < Level; i++){
		barrier_pos[i].row = (rand() % 15) + 1;
		barrier_pos[i].col = rand() % 2;
		
		bool valid = true;
		
		// Check repetitive & path block
		for(int j = 0; j < i; j++){
			if((barrier_pos[j].row == barrier_pos[i].row || barrier_pos[j].row == barrier_pos[i].row - 1 || barrier_pos[j].row == barrier_pos[i].row + 1) && barrier_pos[j].col ==  1 - barrier_pos[i].col){
				// path block
				valid = false;
				break;
			} else if(barrier_pos[j].row == barrier_pos[i].row && barrier_pos[j].col == barrier_pos[i].col){
				// repetitive
				valid = false;
				break;
			}
		}
		if(valid == false){
			i--;
			continue;
		}
	}
	
	/*
	 * Copy barriers to map
	 */
	for(int i = 0; i < Level; i++){
		map[barrier_pos[i].row][barrier_pos[i].col] = barrier;
	}
	
	/*
	 * Print initial map
	 */
	print_map();
	
	/*
	 * Calculate forward move time
	 */
	Calculate_forward_move_time();
}

void clear_map(void){
	for(uint8_t i = 0; i < 16; i++){
		map[i][0] = map[i][1] = empty;
	}
}

void print_map(void){
	clear();
	for(int col = 0; col < 2; col++){
		for(int row = 0; row < 16; row++){
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

void print_player_move(void){
	setCursor(player_pos.row, player_pos.col);
	print(" ");
	setCursor(new_player_pos.row, new_player_pos.col);
	write(0);
}

void print_win_all_levels(void){
	setCursor(0, 0);
	print("* * YOU WIN * * ");
	setCursor(0, 1);
	print(" * * * * * * * *");
}

void print_game_over(void){
	clear();
	
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(' '); HAL_Delay(50);
	write('G'); HAL_Delay(50);
	write('A'); HAL_Delay(50);
	write('M'); HAL_Delay(50);
	write('E'); HAL_Delay(50);
	write(' '); HAL_Delay(50);
	write('O'); HAL_Delay(50);
	write('V'); HAL_Delay(50);
	write('E'); HAL_Delay(50);
	write('R'); HAL_Delay(50);
	write(' '); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	
	setCursor(0, 1);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
	write(2); HAL_Delay(50);
}

void player_Forward_move(void){
	new_player_pos.row = player_pos.row + 1;
	new_player_pos.col = player_pos.col;
	
	if(new_player_pos.row >= 16){
		Win();
	} else if(map[new_player_pos.row][new_player_pos.col] == barrier){
		Lose();
	} else {
		print_player_move();
		player_pos.row = new_player_pos.row;
		
		score += ((630 - forward_move_time) / 10) + 10;
	}
}

void player_RightLeft_move(void){
	new_player_pos.row = player_pos.row;
	new_player_pos.col = 1 - player_pos.col;
	
	if(map[new_player_pos.row][new_player_pos.col] == barrier){
		Lose();
	} else {
		print_player_move();
		player_pos.col = new_player_pos.col;
	}
}

void Lose(void){
	decrement_life();
	if(is_Game_over()){
		Game_Over();
	} else {
		playing_state = Pause;
		player_pos.row = initial_player_pos.row;
		player_pos.col = initial_player_pos.col;
		print_map();
	}
}

void Win(void){
	Level++;
	playing_state = Pause;
	if(Level >= 11){
		All_Levels_passed_successfully();
	} else {
		generate_map();
	}
}

void Game_Over(void){
	state = GameOver;
	/* 
	 * Stop 1ms timer
	 */
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim4);
	
	print_game_over();
	
	reset_all_counters();
}

void All_Levels_passed_successfully(void){
	state = Finish;
	
	print_win_all_levels();
	
	/* 
	 * Open Ganj
	 */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
	
	reset_all_counters();
}

void decrement_life(void){
	Life--;
	turn_off_a_LED(false);
}

void turn_off_a_LED(bool effect){
	// Turn Off with some visual effect
	
	switch(Life){
		case 7:
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
			if(effect){
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
				HAL_Delay(300);
			}
			break;
		case 6:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			if(effect){
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
				HAL_Delay(300);
			}
			break;
		case 5:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			if(effect){
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
				HAL_Delay(300);
			}
			break;
		case 4:
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			if(effect){
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
				HAL_Delay(300);
			}
			break;
		case 3:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			if(effect){
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_Delay(300);
			}
			break;
		case 2:
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			if(effect){
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				HAL_Delay(300);
			}
			break;
		case 1:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			if(effect){
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				HAL_Delay(300);
			}
			break;
		case 0:
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			if(effect){
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				HAL_Delay(300);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				HAL_Delay(300);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				HAL_Delay(300);
			}
			break;
	}
	
//	HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_RESET);
//	if(effect){
//		HAL_Delay(300);
//		HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_SET);
//		HAL_Delay(300);
//		HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_RESET);
//		HAL_Delay(300);
//		HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_SET);
//		HAL_Delay(300);
//		HAL_GPIO_WritePin(GPIOE, life_to_pin_number(), GPIO_PIN_RESET);
//		HAL_Delay(300);
//	}
}

bool is_Game_over(void){
	return Life == 0;
}

uint16_t life_to_pin_number_cumulative(void){
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

uint16_t life_to_pin_number(void){
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

bool validate_map(void){
	return false;
}

void reset_all_counters(void){
	counter_7segment = counter_blink = counter_player_move = 0;
}

void Stop_LEDs(void){
	if(Life >= 1) HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	if(Life >= 2) HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	if(Life >= 3) HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	if(Life >= 4) HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	if(Life >= 5) HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	if(Life >= 6) HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	if(Life >= 7) HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	if(Life >= 8) HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

void Start_LEDs(void){
	if(Life >= 1) HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	if(Life >= 2) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	if(Life >= 3) HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	if(Life >= 4) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	if(Life >= 5) HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	if(Life >= 6) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	if(Life >= 7) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	if(Life >= 8) HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void Set_date_time(uint8_t * datetime){
	// Format: YYYY/MM/DD HH:MM:SS W
	
	/*
	 * Set Date
	 */
	myDate.Year = (10 * char2int(datetime[2])) + char2int(datetime[3]);
	myDate.Month = (10 * char2int(datetime[5])) + char2int(datetime[6]);
	myDate.Date = (10 * char2int(datetime[8])) + char2int(datetime[9]);
	myDate.WeekDay = char2int(datetime[20]);
	HAL_RTC_SetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
	
	/*
	 * Set Time
	 */
	myTime.Hours = (10 * char2int(datetime[11])) + char2int(datetime[12]);
	myTime.Minutes = (10 * char2int(datetime[14])) + char2int(datetime[15]);
	myTime.Seconds = (10 * char2int(datetime[17])) + char2int(datetime[18]);
	HAL_RTC_SetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
}

void Calculate_forward_move_time(void){
	forward_move_time = /*500 + */((11 - Level) * volume);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
