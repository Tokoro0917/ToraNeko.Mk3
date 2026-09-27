/*
 * PL_encoder.c
 *
 *  Created on: Jun 14, 2025
 *      Author: akihi
 */

#include "PL_encoder.h"
#include "spi.h"
#include "stdio.h"
#include "Define.h"
#include <math.h>
#include "lsm6dsr.h"
#include "UI.h"
float encoder_R, encoder_L; //エンコーダ検出角度

float Tire_Speed_L_LOG[10000], Tire_Speed_R_LOG[10000], GYRO_LOG[1000];

float G_Tire_Speed_R, G_Tire_Speed_L; //タイヤスピード

float Tire_Speed_R[20], Tire_Speed_L[20];

float encoder_R_old, encoder_L_old; //前回のエンコーダ角度

float encoder_R_delta, encoder_L_delta;

float encoder_R_sum, encoder_L_sum;

int Program_number, Program_mode;

uint16_t encoder_read_byte_R(uint16_t address, uint16_t data) {

	uint8_t addBuffer[2];
//	uint16_t data;
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( ENCO_R_NSS_GPIO_Port, ENCO_R_NSS_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address | 0x4000; //先頭から2つ目のbitを1に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0] = address >> 8;
	addBuffer[1] = address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*) addBuffer, 2, 100);
//	HAL_SPI_Transmit(&hspi3, address, 2, 100);

	HAL_GPIO_WritePin( ENCO_R_NSS_GPIO_Port, ENCO_R_NSS_Pin, GPIO_PIN_SET); //cs = 1;

	for (int i = 0; i < 150; i++) {
	}

	HAL_GPIO_WritePin( ENCO_R_NSS_GPIO_Port, ENCO_R_NSS_Pin, GPIO_PIN_RESET); //cs = 0;

//	data=0xC000;
	dataBuffer[0] = data >> 8;
	dataBuffer[1] = data & 0x00FF;
	HAL_SPI_Receive(&hspi3, (uint8_t*) dataBuffer, 2, 100);
	data = ((uint16_t) (dataBuffer[0]) << 8) | (uint16_t) (dataBuffer[1]);
//	HAL_SPI_Transmit(&hspi3, data, 2, 100);
	HAL_GPIO_WritePin( ENCO_R_NSS_GPIO_Port, ENCO_R_NSS_Pin, GPIO_PIN_SET); //cs = 1;

	return data;

}

void encoder_write_byte_R(uint16_t address, uint16_t data) {

	uint8_t addBuffer[2];
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( ENCO_R_NSS_GPIO_Port, ENCO_R_NSS_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address & 0xBFFF; //先頭から2つ目のbitを1に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0] = address >> 8;
	addBuffer[1] = address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, addBuffer, 2, 100);

	HAL_GPIO_WritePin(ENCO_R_NSS_GPIO_Port, ENCO_R_NSS_Pin, GPIO_PIN_SET); //cs = 1;

	for (int i = 0; i < 100; i++) {
	}

	HAL_GPIO_WritePin( ENCO_R_NSS_GPIO_Port, ENCO_R_NSS_Pin, GPIO_PIN_RESET); //cs = 0;

	data = data & 0xBFFF; //先頭から2つ目のbitを0に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (data >> i) & 1;
	data = data | ((parity % 2) << 15);
	dataBuffer[0] = data >> 8;
	dataBuffer[1] = data & 0x00FF;
	HAL_SPI_Transmit(&hspi3, dataBuffer, 2, 100);

	HAL_GPIO_WritePin(ENCO_R_NSS_GPIO_Port, ENCO_R_NSS_Pin, GPIO_PIN_SET); //cs = 1;

}

uint16_t encoder_read_byte_L(uint16_t address, uint16_t data) {

	uint8_t addBuffer[2];
//	uint16_t data;
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin(ENCO_L_NSS_GPIO_Port, ENCO_L_NSS_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address | 0x4000; //先頭から2つ目のbitを1に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0] = address >> 8;
	addBuffer[1] = address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*) addBuffer, 2, 100);

	HAL_GPIO_WritePin( ENCO_L_NSS_GPIO_Port, ENCO_L_NSS_Pin, GPIO_PIN_SET); //cs = 1;

	for (int i = 0; i < 150; i++) {
	}

	HAL_GPIO_WritePin( ENCO_L_NSS_GPIO_Port, ENCO_L_NSS_Pin, GPIO_PIN_RESET); //cs = 0;

//	data=0x0000;
	dataBuffer[0] = data >> 8;
	dataBuffer[1] = data & 0x00FF;
	HAL_SPI_Receive(&hspi3, (uint8_t*) dataBuffer, 2, 100);
	data = ((uint16_t) (dataBuffer[0]) << 8) | (uint16_t) (dataBuffer[1]);
	HAL_GPIO_WritePin( ENCO_L_NSS_GPIO_Port, ENCO_L_NSS_Pin, GPIO_PIN_SET); //cs = 1;

	return data;

}

void encoder_write_byte_L(uint16_t address, uint16_t data) {

	uint8_t addBuffer[2];
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( ENCO_L_NSS_GPIO_Port, ENCO_L_NSS_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address & 0xBFFF; //先頭から2つ目のbitを1に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0] = address >> 8;
	addBuffer[1] = address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*) addBuffer, 2, 100);

	HAL_GPIO_WritePin( ENCO_L_NSS_GPIO_Port, ENCO_L_NSS_Pin, GPIO_PIN_SET); //cs = 1;

	for (int i = 0; i < 100; i++) {
	}

	HAL_GPIO_WritePin(ENCO_L_NSS_GPIO_Port, ENCO_L_NSS_Pin, GPIO_PIN_RESET); //cs = 0;

	data = data & 0xBFFF; //先頭から2つ目のbitを0に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (data >> i) & 1;
	data = data | ((parity % 2) << 15);
	dataBuffer[0] = data >> 8;
	dataBuffer[1] = data & 0x00FF;
	HAL_SPI_Transmit(&hspi3, (uint8_t*) dataBuffer, 2, 100);

	HAL_GPIO_WritePin( ENCO_L_NSS_GPIO_Port, ENCO_L_NSS_Pin, GPIO_PIN_SET); //cs = 1;

}

void AS5047_DataUpdate(void) {

	//encoder_read_byte_L(0x3FFF,0xC000);
	//HAL_Delay(5);
	encoder_R = (float) (encoder_read_byte_R(0x3FFF, 0x0000) & 0x3FFF) * 360
			/ 8192;
	//HAL_Delay(500);

	//encoder_read_byte_R(0x3FFF,0xC000);
	//HAL_Delay(5);
	encoder_L = (float) (encoder_read_byte_L(0x3FFF, 0x0000) & 0x3FFF) * 360
			/ 8192;
	//HAL_Delay(5);

}

void Encorder_Speed_Calculate() {
	float sumL = 0, sumR = 0;

	// 1. 過去値の保存
	encoder_R_old = encoder_R;
	encoder_L_old = encoder_L;

	// 2. 最新データの取得
	AS5047_DataUpdate();

	// 3. 角度変化量の計算（オーバーフロー対策含む）
	encoder_R_delta = encoder_R - encoder_R_old;
	if (encoder_R_delta < -300) {
		encoder_R_delta += 360;
	} else if (encoder_R_delta > 300) {
		encoder_R_delta -= 360;
	}

	encoder_L_delta = encoder_L - encoder_L_old;
	if (encoder_L_delta < -300) {
		encoder_L_delta += 360;
	} else if (encoder_L_delta > 300) {
		encoder_L_delta -= 360;
	}

	for (int i = 19; i > 0; i--) {
		Tire_Speed_R[i] = Tire_Speed_R[i - 1];
		Tire_Speed_L[i] = Tire_Speed_L[i - 1];
	}

	Tire_Speed_R[0] = -1 * encoder_R_delta / 360.0f
			* (TIREDIAMETER * PI / GEARRATIO) * 1000.0f;
	Tire_Speed_L[0] = encoder_L_delta / 360.0f * (TIREDIAMETER * PI / GEARRATIO)
			* 1000.0f;

	if (fabs(Tire_Speed_R[0]) < 30) {
		Tire_Speed_R[0] = 0;
	}
	if (fabs(Tire_Speed_L[0]) < 30) {
		Tire_Speed_L[0] = 0;
	}

	for (int i = 0; i < 20; i++) {
		sumR += Tire_Speed_R[i];
		sumL += Tire_Speed_L[i];
	}

	G_Tire_Speed_R = sumR / 20;
	G_Tire_Speed_L = sumL / 20;
}

void Encorder_count_mode() {
	encoder_R_sum += encoder_R_delta;
	encoder_L_sum += encoder_L_delta;

	if (encoder_R_sum > 300) {
		Program_number--;
		encoder_R_sum = 0;
	} else if (encoder_R_sum < -300) {
		Program_number++;
		encoder_R_sum = 0;
	}

	if (Program_number > 15) {
		Program_number = 0;
	} else if (Program_number < 0) {
		Program_number = 15;
	}

	if (encoder_L_sum > 300) {
		Program_mode++;
		encoder_L_sum = 0;
	} else if (encoder_L_sum < -300) {
		Program_mode--;
		encoder_L_sum = 0;
	}

	if (Program_mode > 15) {
		Program_mode = 0;
	} else if (Program_mode < 0) {
		Program_mode = 15;
	}

}

void Encorder_count_reset() {
	encoder_R_sum = 0;
	encoder_L_sum = 0;
	Program_number = 0;
	Program_mode = 0;
}

int Encorder_number_out() {
	return Program_number;
}

int Encorder_mode_out() {
	return Program_mode;
}
