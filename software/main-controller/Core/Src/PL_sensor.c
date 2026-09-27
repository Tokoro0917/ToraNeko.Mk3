/*
 * PL_sensor.c
 *
 *  Created on: Jun 1, 2022
 *      Author: sf199
 */

#include <PL_sensor.h>
#include "adc.h"
#include "dma.h"
#include "gpio.h"//sensorLED用
#include "stdlib.h"

uint16_t g_ADCBuffer[5];
char AD_step;

uint16_t g_sensor_on[4];
uint16_t g_sensor_off[4];

int16_t g_sensor[4][10];
int16_t g_sensor_av[4];
int16_t g_sensor_av10[4];

float g_V_batt;

/*******************************************************************/
/*	電圧の取得			(pl_getbatt)	*/
/*******************************************************************/
/*	戻り値に電圧を返す．(AD変換単体)						*/
/*******************************************************************/
float pl_getbatt(void) {
	float batt;
	uint16_t battAD;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 50);
	battAD = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	batt = 3.4 * (float) battAD / 4095.0 * (103.0 + 47.0) / 47.0;

	return batt;
}

/*******************************************************************/
/*	callback用関数			(pl_callback_getSensor)	*/
/*******************************************************************/
/*	DMAがスタートしたら実行するコード(AD変換複数)					*/
/*******************************************************************/

void pl_callback_getSensor(void) {

	uint16_t V_battAD;
	volatile int j;
	HAL_ADC_Stop_DMA(&hadc1);

	switch (AD_step) {
	case 0:
		HAL_GPIO_WritePin(WallLED_12_GPIO_Port, WallLED_12_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(WallLED_34_GPIO_Port, WallLED_34_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		g_sensor_on[0] = g_ADCBuffer[4];
		g_sensor_on[1] = g_ADCBuffer[3];

		HAL_GPIO_WritePin(WallLED_12_GPIO_Port, WallLED_12_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WallLED_34_GPIO_Port, WallLED_34_Pin, GPIO_PIN_SET);

		break;
	case 2:

		g_sensor_on[2] = g_ADCBuffer[2];
		g_sensor_on[3] = g_ADCBuffer[1];
		HAL_GPIO_WritePin(WallLED_12_GPIO_Port, WallLED_12_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WallLED_34_GPIO_Port, WallLED_34_Pin, GPIO_PIN_RESET);

		break;
	case 3:
		g_sensor_off[0] = g_ADCBuffer[4];
		g_sensor_off[1] = g_ADCBuffer[3];
		g_sensor_off[2] = g_ADCBuffer[2];
		g_sensor_off[3] = g_ADCBuffer[1];

	}

	V_battAD = g_ADCBuffer[0];
	g_V_batt = 3.3 * (float) V_battAD / 4095.0 * (47.0 + 10.0) / 10.0;
	AD_step++;
//	if (AD_step != 4) {
//		HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,
//				sizeof(g_ADCBuffer) / sizeof(uint16_t));
//	} else {
//		AD_step = 0;
//		for (int i = 0; i < 4; i++) {
//			g_sensor[i][0] = abs(g_sensor_on[i] - g_sensor_off[i]);
////			if ((g_sensor[i][0] < 0) || (g_sensor[i][0] > 3500)) {
////				g_sensor[i][0] = g_sensor[i][1];
////			}
////			if (g_sensor[1][0] > 1500) {
////				g_sensor[1][0] = g_sensor[1][1];
////			}
////			if (g_sensor[2][0] > 1500) {
////				g_sensor[2][0] = g_sensor[2][1];
////			}
//			g_sensor[i][0] = g_sensor[i][0] * 0.85 + g_sensor[i][1] * 0.15;
//			for (int k = 4; k > 0; k--) {
//				g_sensor[i][k] = g_sensor[i][k - 1];
//			}
//			g_sensor_av[i] = (g_sensor[i][0] + g_sensor[i][1] + g_sensor[i][2]
//					+ g_sensor[i][3] + g_sensor[i][4]) / 5;
//		}
//	}

	if (AD_step != 4) {
		HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,
				sizeof(g_ADCBuffer) / sizeof(uint16_t));
	} else {
		AD_step = 0;
		for (int i = 0; i < 4; i++) {
			// 外乱光除去とLPF（0.85/0.15）
			g_sensor[i][0] = abs(g_sensor_on[i] - g_sensor_off[i]);
			g_sensor[i][0] = g_sensor[i][0] * 0.85 + g_sensor[i][1] * 0.15;

			// データのシフト（10個分）
			for (int k = 9; k > 0; k--) {
				g_sensor[i][k] = g_sensor[i][k - 1];
			}

			// 従来の5回移動平均
			g_sensor_av[i] = (g_sensor[i][0] + g_sensor[i][1] + g_sensor[i][2]
					+ g_sensor[i][3] + g_sensor[i][4]) / 5;

			// 新設の10回移動平均
			uint32_t sum10 = 0;
			for (int j = 0; j < 10; j++) {
				sum10 += g_sensor[i][j];
			}
			g_sensor_av10[i] = sum10 / 10;
		}
	}
}

/*******************************************************************/
/*	割り込み用動作関数(センサー取得)			(interupt_calSensor)	*/
/*******************************************************************/
/*	センサーの情報を取得する割り込み関数．						*/
/*******************************************************************/
void pl_interupt_getSensor(void) {

	HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,
			sizeof(g_ADCBuffer) / sizeof(uint16_t));

}
