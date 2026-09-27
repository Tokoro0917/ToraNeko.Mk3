/*
 * UI.c
 *
 *  Created on: May 30, 2024
 *      Author: akihi
 */

#include <UI.h>
#include "gpio.h"
#include "tim.h"
#include "PL_encoder.h"

void LED_Reset() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
}

void LED_ALL_ON() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 1);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 1);
}

void LED_Setup_Robot() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 1);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
}

void LED_batt_error() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 1);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 1);
	HAL_Delay(500);
}

void LED_Goal() {
	for (int i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
		HAL_Delay(100);
	}
	LED_Reset();
}

void LED_program_number(int N) {
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
	if (N >= 8) {
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
		N -= 8;
	}
	if (N >= 4) {
		HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
		N -= 4;
	}
	if (N >= 2) {
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 1);
		N -= 2;
	}
	if (N >= 1) {
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 1);
	}
}

void LED_program_mode(int N) {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	if (N >= 8) {
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		N -= 8;
	}
	if (N >= 4) {
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		N -= 4;
	}
	if (N >= 2) {
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
		N -= 2;
	}
	if (N >= 1) {
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
	}
}

void LED_ON_R() {
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 1);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 1);
}

void LED_ON_L() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
}

void LED_StartWait() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 0);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 1);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, 1);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
	HAL_Delay(200);
}

void Buzzer_Number_Change() {
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 10);
//	__HAL_TIM_SET_AUTORELOAD(&htim4, 5000);
//
//	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

void Buzzer_Mode_Change() {

}

void Buzzer_Enter() {
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 20);
	__HAL_TIM_SET_AUTORELOAD(&htim4, 3000);
	HAL_Delay(300);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

void Buzzer_Start() {
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 20);
	__HAL_TIM_SET_AUTORELOAD(&htim4, 3000);
	HAL_Delay(400);
	__HAL_TIM_SET_AUTORELOAD(&htim4, 1500);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

