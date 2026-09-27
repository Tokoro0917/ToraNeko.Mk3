/*
 * Wallsensor.c
 *
 *  Created on: Jun 21, 2024
 *      Author: akihi
 */

#include "Wallsensor.h"
#include "PL_sensor.h"
#include "UI.h"
#include "gpio.h"
#include "motor.h"
#include "math.h"

int G_Wall_data[] = { 0, 0, 0, 0 };
int Wall_threshold[] = { 60, 90, 90, 60 }; //80, 150, 150, 80

float Kp = 0.05; //0.025
float Kd = 0.01;

float Kp_Na = 0.1;
float Kd_Na = 0.00;

float Kp_FW = 0.1;
float Kd_FW = 0.0;

float Kp_FW_dire = 0.1;
float Kd_FW_dire = 0.0;

float KP, KD;

int Wall_L = 220; //370
int Wall_R = 200; //250

int Wall_TH_L = 80;
int Wall_TH_R = 80;

float FlontWall_Distance = 530;

int Sensor_diff_TH = 50;

float Wall_error = 0;
float Wall_old_error = 0;
float Wall_Delta_error = 0;

float Wall_error_NA = 0;
float Wall_old_error_NA = 0;
float Wall_Delta_error_NA = 0;

float Wall_error_FW = 0;
float Wall_old_error_FW = 0;
float Wall_Delta_error_FW = 0;

void Wall_search() {
	for (int i = 0; i < 4; i++) {
		G_Wall_data[i] = 0;
	}

	if (g_sensor_av10[0] > Wall_threshold[0]) { //正面左//g_sensor_av
		G_Wall_data[0] = 1;
	}
	if (g_sensor_av10[2] > Wall_threshold[2]) { //左
		G_Wall_data[1] = 1;
	}

	if (g_sensor_av10[1] > Wall_threshold[1]) { //右
		G_Wall_data[2] = 1;
	}
	if (g_sensor_av10[3] > Wall_threshold[3]) { //正面右
		G_Wall_data[3] = 1;
	}

}

int Sensor_Enter() {
	int i = 0;
	if (g_sensor_av[0] > 1400) {
		i = 1;
	}
	return i;
}

int Sensor_Start() {
	while (1) {
		LED_StartWait();
		if ((g_sensor_av[1] + (g_sensor_av[2])) > 2000) {
			break;
		}
	}
	Buzzer_Start();

}

float Wall_Flont_Av() {
	float av = 0;
	av = (g_sensor[0][0] + g_sensor[3][0]) / 2;
	return av;
}

void Wall_search_LED() {
	if (G_Wall_data[0] == 1) {
		HAL_GPIO_WritePin(LEDF1_GPIO_Port, LEDF1_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF1_GPIO_Port, LEDF1_Pin, 0);
	}
	if (G_Wall_data[1] == 1) {
		HAL_GPIO_WritePin(LEDF3_GPIO_Port, LEDF3_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF3_GPIO_Port, LEDF3_Pin, 0);
	}
	if (G_Wall_data[2] == 1) {
		HAL_GPIO_WritePin(LEDF2_GPIO_Port, LEDF2_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF2_GPIO_Port, LEDF2_Pin, 0);
	}
	if (G_Wall_data[3] == 1) {
		HAL_GPIO_WritePin(LEDF4_GPIO_Port, LEDF4_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF4_GPIO_Port, LEDF4_Pin, 0);
	}

}

float calWallConrol() {

	int Sensor_diff_L = abs(g_sensor[2][0] - g_sensor[2][1]);
	int Sensor_diff_R = abs(g_sensor[1][0] - g_sensor[1][1]);
	float PID_Wall = 0;
	int Wall_st = 0;
	if ((g_sensor_av[2] > Wall_TH_L) && (Sensor_diff_L < Sensor_diff_TH)) { //左あり
		if ((g_sensor_av[1] > Wall_TH_R) && (Sensor_diff_R < Sensor_diff_TH)) { //右あり
			Wall_st = 3;
		} else { //右なし
			Wall_st = 1;
		}
	} else {
		if ((g_sensor_av[1] > Wall_TH_R) && (Sensor_diff_R < Sensor_diff_TH)) {
			Wall_st = 2;
		} else {
			Wall_st = 0;
		}
	}

	if (G_Motor_V_Target >= 2000) {
		KP = Kp * (2000 / 500);
		KD = Kd * (2000 / 500);
	} else {
		KP = Kp * (G_Motor_V_Target / 500);
		KD = Kd * (G_Motor_V_Target / 500);
	}

	int Sensor_L = g_sensor[2][0];
	int Sensor_R = g_sensor[1][0];

	if (Sensor_L > 600) {
		Sensor_L = 600;
	}
	if (Sensor_R > 600) {
		Sensor_R = 600;
	}

	LED_Reset();
	if (Wall_Flont_Av() > 800) {
		Wall_error = 0;
		Wall_old_error = 0;
	} else if (Wall_st == 3) {
		Wall_error = (Sensor_L - Wall_L) - (Sensor_R - Wall_R);
		LED_ON_L();
		LED_ON_R();
	} else if (Wall_st == 2) {
		Wall_error = 2.0 * -(Sensor_R - Wall_R);
		LED_ON_R();
	} else if (Wall_st == 1) {
		Wall_error = 2.0 * (Sensor_L - Wall_L);
		LED_ON_L();
	} else {
		Wall_error = 0;
		Wall_old_error = 0;
	}

	Wall_Delta_error = Wall_error - Wall_old_error;
	Wall_old_error = Wall_error;

	PID_Wall = KP * Wall_error + KD * Wall_Delta_error;

	return PID_Wall;
}

float calWallConrol_NANAME() {
	float PID_Wall = 0;

	KP = Kp_Na;
	KD = Kd_Na;

	KP = Kp_Na * (G_Motor_V_Target / 1000);
	KD = Kd_Na * (G_Motor_V_Target / 1000);
	LED_Reset();
	if (g_sensor[0][0] > 110) {
		Wall_error_NA = g_sensor[0][0] - 110;
		LED_ON_L();
	} else if (g_sensor[3][0] > 110) {
		Wall_error_NA = -(g_sensor[3][0] - 110);
		LED_ON_R();
	} else {
		Wall_error_NA = 0;
		Wall_old_error_NA=0;
	}

	Wall_Delta_error_NA = Wall_error_NA - Wall_old_error_NA;
	Wall_old_error_NA = Wall_error_NA;
	PID_Wall = KP * Wall_error_NA + KD * Wall_Delta_error_NA;

	return PID_Wall;
}

float calWallConrol_Flontwall_ST() {
	float PID_Wall = 0;

	if (Wall_Flont_Av() > 1000) {
		Wall_error_FW = FlontWall_Distance - 1000;
	} else {
		Wall_error_FW = FlontWall_Distance - Wall_Flont_Av();
	}

	Wall_Delta_error_FW = Wall_error_FW - Wall_old_error_FW;
	Wall_old_error_FW = Wall_error_FW;
	PID_Wall = Kp_FW * Wall_error_FW + Kd_FW * Wall_Delta_error_FW;

	return PID_Wall;
}

float calWallConrol_Flontwall_Turn() {
	float PID_Wall = 0;

	Wall_error_FW = (g_sensor[0][0]) - (g_sensor[3][0]);

	Wall_Delta_error_FW = Wall_error_FW - Wall_old_error_FW;
	Wall_old_error_FW = Wall_error_FW;
	PID_Wall = Kp_FW_dire * Wall_error_FW + Kd_FW_dire * Wall_Delta_error_FW;

	return PID_Wall;
}
