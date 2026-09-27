/*
 * Failsafe.c
 *
 *  Created on: Jul 1, 2024
 *      Author: akihi
 */

#include "Failsafe.h"
#include "math.h"

#include "PL_encoder.h"
#include "motor.h"

int flag = 0;
int motordiff_TH = 100000;
int Vdiff = 2000;

void Failsafe_Flag_OFF() {
	flag = 0;
}

int Failsafe_Flag() {
	return flag;
}

void Failsafe_Motordiff() {
	float diff = fabs(G_Tire_Speed_L - G_Tire_Speed_R);
	if (diff > motordiff_TH) {
		flag = 1;
	}
}

void Failsafe_MotorTagetdiff() {
	int V = ((G_Tire_Speed_L + G_Tire_Speed_R) / 2);
	if (G_Motor_V_Target - V > Vdiff) {
		flag = 1;
	}
	if ((G_Motor_V_Target >= 500) && (G_Motor_V_Target <= 1200)&&(G_Motor_Ac==0)) {//探索時の壁に進み続ける防止
		if (abs(V) < 100) {
			flag = 1;
		}
	}
}

void Failsafe_interrupt() {
	Failsafe_Motordiff();
	Failsafe_MotorTagetdiff();
}
