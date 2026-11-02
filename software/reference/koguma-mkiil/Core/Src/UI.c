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

/* スイッチ(基板の S2、PB3。2026-10-27)。CubeMX では設定していないので、ここで入力にする(main の USER CODE 2 で呼ぶ)。
 * 外の 10k(R31)で 3.3V に引き上げてあり、押すと GND につながる(押すと 0) */
void Switch_Init() {
	GPIO_InitTypeDef init = { 0 };
	__HAL_RCC_GPIOB_CLK_ENABLE();
	init.Pin = SW_Pin;
	init.Mode = GPIO_MODE_INPUT;
	init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SW_GPIO_Port, &init);
}

/* スイッチが押されていれば 1 */
int Switch_Pressed() {
	return HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET;
}

/* ロード中(2026-10-27、ToraNeko.Mk3 と同じ)。LED1 から LED8 へ1個ずつ順に点けていき、全部点いたら消す(約1秒)。
 * フェイルセーフを解除してモード選択に戻るときに出す。このあいだは何をしても何も起きない */
void LED_Loading() {
	static GPIO_TypeDef *const port[8] = { LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port, LED4_GPIO_Port,
			LED5_GPIO_Port, LED6_GPIO_Port, LED7_GPIO_Port, LED8_GPIO_Port };
	static const uint16_t pin[8] = { LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin, LED5_Pin, LED6_Pin, LED7_Pin, LED8_Pin };
	LED_Reset();
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(port[i], pin[i], 1);
		HAL_Delay(120);
	}
	HAL_Delay(120);
	LED_Reset();
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

/* 最短経路が作れなかったとき(スタートからゴールへ行けない迷路)の知らせ。
 * 全LED点滅+低い音を3回。走り出さずに戻る */
void LED_Path_Error() {
	for (int i = 0; i < 3; i++) {
		LED_ALL_ON();
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 20);
		__HAL_TIM_SET_AUTORELOAD(&htim4, 6000);
		HAL_Delay(300);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
		LED_Reset();
		HAL_Delay(200);
	}
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

/* 全部走り終えてスタートに戻ったときの曲(2026-10-20、モード0 No.11)。
 * ブザーは TIM4(1MHz)で、周波数 f のとき AUTORELOAD = 1000000 / f - 1。音量はほかの音と同じ COMPARE 20 */
void Buzzer_Finish() {
	static const uint16_t freq[] = { 392, 523, 659, 784, 0, 659, 784, 0, 1047 };	//ソ ド ミ ソ - ミ ソ - ド
	static const uint16_t ms[] = { 150, 150, 150, 300, 80, 150, 450, 80, 700 };
	for (unsigned i = 0; i < sizeof(freq) / sizeof(freq[0]); i++) {
		if (freq[i] == 0) {
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			LED_Reset();
		} else {
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 20);
			__HAL_TIM_SET_AUTORELOAD(&htim4, 1000000 / freq[i] - 1);
			LED_ALL_ON();
		}
		HAL_Delay(ms[i]);
	}
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	LED_Reset();
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

