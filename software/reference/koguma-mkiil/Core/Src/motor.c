/*
 * motor.c
 *
 *  Created on: Jun 12, 2025
 *      Author: akihi
 */

#include "motor.h"
#include "tim.h"
#include "math.h"
#include "PL_encoder.h"
#include "PL_timer.h"
#include "Define.h"
#include "lsm6dsr.h"
#include "PL_sensor.h"
#include "Wallsensor.h"
#include "Maze.h"
#include "UI.h"
#include "stdlib.h"

float G_Robot_speed = 0;

float G_Motor_X = 0;
float G_Motor_Ac = 0;
float G_Motor_V_Target = 0;

float G_Motor_Angle = 0;
float G_Motor_W_Ac = 0;
float G_Motor_W_Target = 0;

int G_Motor_Flag = 0;
float G_Motor_Count = 0;

float Motor_PWM_L, Motor_PWM_R;

float Enc_error = 0;
float Enc_Old_error = 0;
float Enc_Sigma_error = 0;
float Enc_Delta_error = 0;

float Gyro_error = 0;
float Gyro_Old_error = 0;
float Gyro_Sigma_error = 0;
float Gyro_Delta_error = 0;

float Ksp = 0.25; //0.8
float Ksi = 0.0015; //0.002
float Ksd = 0.01; //0.5

float Ksp_Fun = 0.23; //0.5
float Ksi_Fun = 0.0015; //0.005
float Ksd_Fun = 0.1; //0.01

float Ksp_AT = 0.3; //0.3
float Ksi_AT = 0.003; ////
float Ksd_AT = 0.01;

float Ktp = 0.7; //
float Kti = 0.01; //
float Ktd = 0.001; //

float Ktp_sula = 0.7; //0.12
float Kti_sula = 0.005; //0.06
float Ktd_sula = 0.001; //0.01

float Ktp_sula_20 = 1.2; //0.12
float Kti_sula_20 = 0.02; //0.06
float Ktd_sula_20 = 0.01; //0.01

float Ktp_sula_24 = 1.4; //0.12
float Kti_sula_24 = 0.02; //0.06
float Ktd_sula_24 = 0.01; //0.01

float Motor_FB_ST = 0;
float Motor_FB_Turn = 0;
float Motor_FF_ST = 0;
float Motor_FF_Turn = 0;

float Motor_Voltage_L = 0;
float Motor_Voltage_R = 0;

int PID_Mode = 0;
int Turn_Mode = 0;
int Fun_Flag = 0;

int Sula_Flag = 0;

float Alignment_TIME = 0.5;

int Flont_th = 130; //310

float Cut_R = 80;
float Cut_L = 80;

float Cut_R_NA = 150;
float Cut_L_NA = 150;

float Run_Voltage;

float Aff = 0.65;
float Bff = 0.50;

float FF_offset = 15.0;

void Motor_Setup() {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
}

void Motor_Setup_Voltage() {
	Run_Voltage = g_V_batt;
}

void Motor_Stop() {
	G_Motor_Flag = 1;
	PID_Mode = 0;
	G_Motor_V_Target = 0;
	G_Motor_Ac = 0;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
//	Motor_PWM_L = 0;
//	Motor_PWM_R = 0;
	wait_ms(50);
	Motor_PWM_L = 0;
	Motor_PWM_R = 0;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	G_Motor_Flag = 0;

}

void Suction_Start(int duty) {
	Fun_Flag = 1;
	if (duty == 50) {
		Sula_Flag = 1;
	} else if (duty >= 70) {
		Sula_Flag = 2;
	} else if (duty <= 20) {
		Fun_Flag = 0;
	}
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
	duty = duty * (15.8 / Run_Voltage);
	for (int i = 1; duty > i; i += 1) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
		HAL_Delay(10);
	}

	HAL_Delay(500);
}

void Fun_Flag_OFF() {
	Fun_Flag = 0;
}

void Suction_change(int duty) {
	duty = duty * (15.8 / Run_Voltage);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
}

void Suction_Stop() {
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	Sula_Flag = 0;
	Fun_Flag = 0;
}

float cal_turnV(float W) {
	float TurnV = ((float) TIREBETWEEN * PI / 360) * W;
	return TurnV;
}

void Motor_PWM_Generate() { //MP6550モーター出力

	Motor_Voltage_L = Motor_PWM_L * (REFERENCE_V / Run_Voltage);
	Motor_Voltage_R = Motor_PWM_R * (REFERENCE_V / Run_Voltage);

	if (Motor_Voltage_L > 200) {
		Motor_Voltage_L = 200;
	} else if (Motor_Voltage_L < -200) {
		Motor_Voltage_L = -200;
	}

	if (Motor_Voltage_R > 200) {
		Motor_Voltage_R = 200;
	} else if (Motor_Voltage_R < -200) {
		Motor_Voltage_R = -200;
	}

	if (Motor_Voltage_L > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200 - Motor_Voltage_L);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 200);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
				200 - fabs(Motor_Voltage_L));
	}
	if (Motor_Voltage_R > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 200 - Motor_Voltage_R);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 200);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 200);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,
				200 - fabs(Motor_Voltage_R));
	}
}

void Motor_Free() {
	G_Motor_V_Target = 0;
	G_Motor_Ac = 0;
	Suction_Stop();
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

void Motor_Back() {
	G_Motor_Flag = 5;
	G_Motor_V_Target = -300;
	G_Motor_Ac = 0;
	wait_ms(250);
	Enc_Sigma_error = 0;
	Motor_PWM_L = 0;
	Motor_PWM_R = 0;
	G_Motor_Flag = 0;
}

void Motor_Speed_PID_ST(int V, int Ac) { // mm/s

	float T = ((TIREDIAMETER / 2000) * ROBOT_M * (Ac / 1000)) / (2 * GEARRATIO)
			* Aff;
	//float w = 0;
	float w = (60 * GEARRATIO * (V / 1000)) / (2 * PI * (TIREDIAMETER / 2000))
			* Bff;
	Motor_FF_ST = (((MOTORR * T) / KT) + (w * KE)) / Run_Voltage * 200.0;

	if (V > 1) {
		Motor_FF_ST += FF_offset;
	} else if (V < -1) {
		Motor_FF_ST -= FF_offset;
	}

	G_Robot_speed = ((G_Tire_Speed_L + G_Tire_Speed_R) / 2); //)+ (read_average_acc_x()*0.01/2)

	Enc_error = V - G_Robot_speed;
	Enc_Delta_error = Enc_error - Enc_Old_error;
	Enc_Sigma_error += Enc_error;

	if (Enc_Sigma_error > 2000)
		Enc_Sigma_error = 2000;
	else if (Enc_Sigma_error < -2000)
		Enc_Sigma_error = -2000;

	if (Fun_Flag == 1) {
		Motor_FB_ST = Ksp_Fun * Enc_error + Ksi_Fun * Enc_Sigma_error
				+ Ksd_Fun * Enc_Delta_error;
	} else {
		Motor_FB_ST = (Ksp * Enc_error + Ksi * Enc_Sigma_error
				+ Ksd * Enc_Delta_error);
	}

	if (PID_Mode == 0) {
		Motor_PWM_L = Motor_FB_ST + Motor_FF_ST;
		Motor_PWM_R = Motor_FB_ST + Motor_FF_ST;
	} else if (PID_Mode == 1) {
//		if (Fun_Flag == 1) {
//			Motor_PWM_L = Motor_FF_ST + Motor_FB_ST + calWallConrol(); //
//			Motor_PWM_R = Motor_FF_ST + Motor_FB_ST - calWallConrol(); //
//		} else {
		Motor_PWM_L = Motor_FF_ST + Motor_FB_ST + calWallConrol(); //
		Motor_PWM_R = Motor_FF_ST + Motor_FB_ST - calWallConrol(); //
//		}
	} else if (PID_Mode == 2) {
		Motor_PWM_L = Motor_FB_ST + Motor_FF_ST + calWallConrol_NANAME();
		Motor_PWM_R = Motor_FB_ST + Motor_FF_ST - calWallConrol_NANAME();
	} else if (PID_Mode == 3) {
		Motor_PWM_L = calWallConrol_Flontwall_ST(); //
		Motor_PWM_R = calWallConrol_Flontwall_ST(); //
	}

	Enc_Old_error = Enc_error;

}

void Motor_Speed_PID_Turn(float W, float W_Ac) {
	float w = 0;
	if (PID_Mode == 1) {
		W = calWallConrol() * 0.4;
	} else if (PID_Mode == 2) {
		//W = calWallConrol() * 0.5;
	}
	float T = ((TIREDIAMETER / 2000) * ROBOT_M * (W_Ac / 1000))
			/ (2 * GEARRATIO);
//	float w = (60 * GEARRATIO * (W / 1000)) / (2 * PI * (TIREDIAMETER / 2000))
//			* Bff_Turn;
	Motor_FF_ST = (((MOTORR * T) / KT) + (w * KE)) / Run_Voltage * 7;
	//Motor_FF_Turn = 0;

	if (Turn_Mode == 1) { //右向き
		Gyro_error = -W - G_Gyro_Z;
	} else { //左向き
		Gyro_error = W - G_Gyro_Z;
	}

	Gyro_Delta_error = Gyro_error - Gyro_Old_error;
	Gyro_Sigma_error += Gyro_error;

	if (G_Motor_Flag == 1) { //直線
		Motor_FB_Turn = Ksp_AT * Gyro_error + Ksi_AT * Gyro_Sigma_error
				+ Ksd_AT * Gyro_Delta_error;
		Motor_PWM_L -= cal_turnV(Motor_FB_Turn);
		Motor_PWM_R += cal_turnV(Motor_FB_Turn);
	} else if (G_Motor_Flag == 2) { //超シンチ
		Motor_FB_Turn = Ktp * Gyro_error + Kti * Gyro_Sigma_error
				+ Ktd * Gyro_Delta_error;
		Motor_PWM_L -= cal_turnV(Motor_FB_Turn);
		Motor_PWM_R += cal_turnV(Motor_FB_Turn);
	} else if (G_Motor_Flag == 3) { //スラローム
		if (Sula_Flag == 1) {
			Motor_FB_Turn = Ktp_sula_20 * Gyro_error
					+ Kti_sula_20 * Gyro_Sigma_error
					+ Ktd_sula_20 * Gyro_Delta_error;
		} else if (Sula_Flag == 2) {
			Motor_FB_Turn = Ktp_sula_24 * Gyro_error
					+ Kti_sula_24 * Gyro_Sigma_error
					+ Ktd_sula_24 * Gyro_Delta_error; //
		} else {
			Motor_FB_Turn = Ktp_sula * Gyro_error + Kti_sula * Gyro_Sigma_error
					+ Ktd_sula * Gyro_Delta_error; //
		}
		Motor_PWM_L -= cal_turnV(Motor_FB_Turn + Motor_FF_Turn);
		Motor_PWM_R += cal_turnV(Motor_FB_Turn + Motor_FF_Turn);
	} else if (G_Motor_Flag == 4) {
		Motor_PWM_L -= calWallConrol_Flontwall_Turn();
		Motor_PWM_R += calWallConrol_Flontwall_Turn();
	} else if (G_Motor_Flag == 5) {
		Motor_FB_Turn = Ksp_AT * Gyro_error;
		Motor_PWM_L -= cal_turnV(Motor_FB_Turn);
		Motor_PWM_R += cal_turnV(Motor_FB_Turn);
	}

	Gyro_Old_error = Gyro_error;
}

void Motor_trapezoid(float Vst, float Vmax, float Vend, float Ac, float X) {
	G_Motor_Flag = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;

	G_Motor_Count = 0;

	float Acceleration_X = (Vmax * Vmax - Vst * Vst) / (4 * Ac) * PI;
	float Deceleration_X = 0;
	float Constant_X = 0;

	float x13 = PI / (2 * Ac) * (Vmax * Vmax - ((Vst * Vst + Vend * Vend) / 2));
	if (x13 > X) {
		Vmax = sqrt((2 * Ac * X / PI) + ((Vst * Vst + Vend * Vend) / 2));
	}

	float Df1 = Vmax - Vst;
	float t1 = PI * Df1 / 2 / Ac;
	while (1) {

		if (Df1 > 0) {
			G_Motor_V_Target = Df1 / 2 * (1 - cos(2 * Ac / Df1 * G_Motor_Count))
					+ Vst;
			G_Motor_Ac = Ac * sin(2 * Ac / Df1 * G_Motor_Count);
		}

		if (Vmax <= Vend) {
			Deceleration_X = 0;
		} else {
			Deceleration_X = (G_Motor_V_Target * G_Motor_V_Target - Vend * Vend)
					/ (4 * Ac) * PI;
		}

		if (t1 <= G_Motor_Count) {
			G_Motor_V_Target = Vmax;
			Deceleration_X = (G_Motor_V_Target * G_Motor_V_Target - Vend * Vend)
					/ (4 * Ac) * PI;
			break;
		}
		if (X - G_Motor_X < Deceleration_X) {
			Acceleration_X = G_Motor_X;
			break;
		}

	}

	Constant_X = X - (Acceleration_X + Deceleration_X);

	G_Motor_X = 0;
	G_Motor_Ac = 0;

	while (1) {
		if (G_Motor_X > Constant_X) {
			break;
		}

	}
	G_Motor_Count = 0;
	float Df3 = Vmax - Vend;
	float t3 = PI * Df3 / 2 / Ac;
	G_Motor_Ac = -Ac;
	while (1) {
		//Df3 / 2* (1 - cos(2 * Ac / Df3 * (G_Motor_Count - t1))) + Vend;
		G_Motor_V_Target = Df3 / 2
				- (Df3 / 2) * (1 - cos(G_Motor_Count * ((2 * Ac) / Df3))) + Vend
				+ (Df3 / 2);

		if (Df3 > 0) {
			G_Motor_Ac = -Ac * sin(G_Motor_Count * ((2 * Ac) / Df3));
		}

		if (t3 <= G_Motor_Count) {
			break;
		}
	}
	G_Motor_V_Target = Vend;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

void Motor_trapezoid_PID(float Vst, float Vmax, float Vend, float Ac, float X) {
	PID_Mode = 1;
	Motor_trapezoid(Vst, Vmax, Vend, Ac, X);
}

void Motor_NANAME_PID(float Vst, float Vmax, float Vend, float Ac, float X) {
	PID_Mode = 2;
	Motor_trapezoid(Vst, Vmax, Vend, Ac, X);
}

void Motor_Multistage_PID(float Vst, float Vmax, float Vend, float Ac_base,
		float X) {
	// --- 加速度と閾値の設定（例） ---
	float Ac_1 = Ac_base * 1.2;  // 低速域：超加速
	float Ac_2 = Ac_base * 1.0;  // 中速域：標準
	float Ac_3 = Ac_base * 0.6;  // 高速域：飽和防止
	float Ac_De = Ac_base * 2;

	float V_th1 = 2000.0f;        // 1段階目の切り替え速度
	float V_th2 = 3500.0f;        // 2段階目の切り替え速度

	G_Motor_Flag = 1;
	PID_Mode = 1;
	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Count = 0;

	float prev_count = 0;
	float dt = 0;
	float Deceleration_X = 0;

	// ==========================================
	// 1. 加速フェーズ（3段階切り替え）
	// ==========================================
	while (G_Motor_V_Target < Vmax) {
		dt = G_Motor_Count - prev_count;
		prev_count = G_Motor_Count;

		// ★3段階の加速度判定
		if (G_Motor_V_Target < V_th1) {
			G_Motor_Ac = Ac_1;
		} else if (G_Motor_V_Target < V_th2) {
			G_Motor_Ac = Ac_2;
		} else {
			G_Motor_Ac = Ac_3;
		}

		G_Motor_V_Target += G_Motor_Ac * dt;
		if (G_Motor_V_Target > Vmax)
			G_Motor_V_Target = Vmax;

		// 常に減速開始距離をチェック（一定の Ac_base で減速すると仮定）
		Deceleration_X = (G_Motor_V_Target * G_Motor_V_Target - Vend * Vend)
				/ (2 * Ac_De);
		if (X - G_Motor_X <= Deceleration_X)
			goto DECEL_START;
	}

	// ==========================================
	// 2. 等速フェーズ
	// ==========================================
	G_Motor_Ac = 0;
	while (1) {
		prev_count = G_Motor_Count;
		Deceleration_X = (Vmax * Vmax - Vend * Vend) / (2 * Ac_De);
		if (X - G_Motor_X <= Deceleration_X)
			break;
	}

	DECEL_START:
	// ==========================================
	// 3. 減速フェーズ（一定）
	// ==========================================
	G_Motor_Ac = -Ac_De;
	while (G_Motor_V_Target > Vend) {
		dt = G_Motor_Count - prev_count;
		prev_count = G_Motor_Count;
		G_Motor_V_Target += G_Motor_Ac * dt;
		if (G_Motor_V_Target <= Vend) {
			G_Motor_V_Target = Vend;
			break;
		}
	}

	G_Motor_V_Target = Vend;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

void Motor_trapezoid_Asymmetric_PID(float Vst, float Vmax, float Vend, float Ac,
		float X) {
	G_Motor_Flag = 1;
	PID_Mode = 1;
	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;

	G_Motor_Count = 0;

	// ★減速加速度を加速の2倍に固定
	float Ac_dec = Ac * 2.0f;

	// 加速・減速それぞれの理論上の必要距離
	float Acceleration_X = (Vmax * Vmax - Vst * Vst) / (4 * Ac) * PI;
	float Deceleration_X = (Vmax * Vmax - Vend * Vend) / (4 * Ac_dec) * PI;
	float Constant_X = 0;

	// 三角加速（等速区間がない場合）の判定とVmax再計算
	if (Acceleration_X + Deceleration_X > X) {
		// 非対称加速度（Acと2Ac）の場合の到達可能最高速度を算出
		Vmax = sqrt(
				((8.0f * Ac * X / PI) + 2.0f * Vst * Vst + Vend * Vend) / 3.0f);
	}

	float Df1 = Vmax - Vst;
	float t1 = PI * Df1 / 2 / Ac;

	// --- 1. 加速フェーズ ---
	while (1) {
		if (Df1 > 0) {
			G_Motor_V_Target = Df1 / 2 * (1 - cos(2 * Ac / Df1 * G_Motor_Count))
					+ Vst;
			G_Motor_Ac = Ac * sin(2 * Ac / Df1 * G_Motor_Count);
		}

		// 走行中に常に減速距離（Ac_decを使用）をチェック
		float current_Decel_X = (G_Motor_V_Target * G_Motor_V_Target
				- Vend * Vend) / (4 * Ac_dec) * PI;

		if (t1 <= G_Motor_Count) {
			G_Motor_V_Target = Vmax;
			G_Motor_Ac = 0;
			Deceleration_X = current_Decel_X; // 確定
			break;
		}
		if (X - G_Motor_X < current_Decel_X) {
			Acceleration_X = G_Motor_X;
			Deceleration_X = current_Decel_X; // 確定
			break;
		}
	}

	// 等速区間の計算
	Constant_X = X - (Acceleration_X + Deceleration_X);
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	// --- 2. 等速フェーズ ---
	while (1) {
		if (G_Motor_X > Constant_X) {
			break;
		}
	}

	// --- 3. 減速フェーズ（加速度は2倍の Ac_dec を使用） ---
	G_Motor_Count = 0;
	float Df3 = Vmax - Vend;
	float t3 = PI * Df3 / 2 / Ac_dec;

	while (1) {
		if (Df3 > 0) {
			// 減速のcosカーブ。終了点(Vend)に向かって滑らかに着地
			G_Motor_V_Target = Df3 / 2
					* (1 + cos(2 * Ac_dec / Df3 * G_Motor_Count)) + Vend;
			G_Motor_Ac = -Ac_dec * sin(2 * Ac_dec / Df3 * G_Motor_Count);
		}

		if (t3 <= G_Motor_Count) {
			break;
		}
	}

	G_Motor_V_Target = Vend;
	G_Motor_Ac = 0;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

void Motor_Sula_before(float Vst, float Vmax, float Vend, float Ac, float X) {
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Gyro_Sigma_error = 0;
	//Enc_Sigma_error = 0;

	if (Wall_Flont_Av() > Flont_th - 70) {		//壁補正前進用
		X = X + 15;
	}

	Wall_search();
	Maze_Wall_Update();

	while (1) {
		if (G_Motor_X > X) {
			break;
		}

		if (Wall_Flont_Av() > Flont_th) {
			break;
		}
	}

	PID_Mode = 0;
	G_Motor_Flag = 0;

}

void Motor_Sula_ST(float Vst, float Vmax, float Vend, float Ac, float X) {
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Gyro_Sigma_error = 0;
	//Enc_Sigma_error = 0;

	if (Wall_Flont_Av() > Flont_th - 70) {		//壁補正前進用
		X = X + 15;
	}

	while (1) {
		if (G_Motor_X > X) {
			break;
		}

		if (Wall_Flont_Av() > Flont_th) {
			break;
		}
	}

	PID_Mode = 0;
	G_Motor_Flag = 0;

}

void Motor_trapezoid_Turn(float Angle, float Wmax, float W_Ac) {
	int SANKAKU = 0;
	PID_Mode = 0;
	G_Motor_Flag = 2;
	G_Motor_V_Target = 0;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = W_Ac;
	G_Motor_W_Target = 0;

	if (Angle < 0) {
		Angle = -Angle;
		Turn_Mode = 1;				//右回転
	} else {
		Turn_Mode = 0;				//左回転
	}

	Gyro_Sigma_error = 0;
	Enc_Sigma_error = 0;

	float Acceleration_X = (Wmax * Wmax) / (4 * W_Ac) * PI;
	float Deceleration_X = 0;
	float Constant_X = 0;

	float x13 = PI / (2 * W_Ac) * (Wmax * Wmax);
	if (x13 > Angle) {
		Wmax = sqrt(2 * W_Ac * Angle / PI);
		SANKAKU = 1;
	}
	float Df1 = Wmax;
	float t1 = PI * Df1 / 2 / W_Ac;
	G_Motor_Count = 0;
	while (1) {
		G_Motor_W_Target = Df1 / 2 * (1 - cos(2 * W_Ac / Df1 * G_Motor_Count));
		if (t1 <= G_Motor_Count) {
			G_Motor_W_Target = Wmax;
			Deceleration_X = (G_Motor_W_Target * G_Motor_W_Target)
					/ (4 * W_Ac)* PI;
			break;
		}
		if (Angle - G_Motor_Angle < Deceleration_X) {
			Acceleration_X = fabs(G_Motor_Angle);
			break;
		}

	}
	Constant_X = fabs(Angle - (Acceleration_X + Deceleration_X));

	G_Motor_W_Ac = 0;
	G_Motor_Angle = 0;

	while (1) {
		if (SANKAKU == 1) {
			SANKAKU = 0;
			break;
		}
		if (fabs(G_Motor_Angle) > Constant_X) {
			break;
		}
	}

	G_Motor_Count = 0;
	float Df3 = G_Motor_W_Target;
	float t3 = PI * Df3 / 2 / W_Ac;
	while (1) {
		G_Motor_W_Target = Df3 / 2
				* (1 - cos(2 * W_Ac / Df3 * (G_Motor_Count - t1)));
		if (t3 <= G_Motor_Count) {
			break;
		}
	}
	G_Motor_Flag = 0;
}

void Motor_Sula_COS(float V, float Angle, float Wmax, float W_Ac) {
	int SANKAKU = 0;
	G_Motor_Flag = 3;
	G_Motor_V_Target = V;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	if (Angle < 0) {
		Angle = -Angle;
		Turn_Mode = 1;				//右回転
	} else {
		Turn_Mode = 0;				//左回転
	}

	Gyro_Sigma_error = 0;
	Enc_Sigma_error = 0;
	G_Motor_W_Ac = W_Ac;
	float Acceleration_X = (Wmax * Wmax) / (4 * W_Ac) * PI;
	float Deceleration_X = 0;
	float Constant_X = 0;

	float x13 = PI / (2 * W_Ac) * (Wmax * Wmax);
	if (x13 > Angle) {
		Wmax = sqrt(2 * W_Ac * Angle / PI);
		SANKAKU = 1;
	}
	float Df1 = Wmax;
	float t1 = PI * Df1 / 2 / W_Ac;
	G_Motor_Count = 0;
	while (1) {
		//printf("Ac____G_Motor_W_Target:%f----G_Motor_Angle:%f\n\r",
		//G_Motor_W_Target, G_Motor_Angle);
		G_Motor_W_Target = Df1 / 2 * (1 - cos(2 * W_Ac / Df1 * G_Motor_Count));
		if (t1 <= G_Motor_Count) {
			G_Motor_W_Target = Wmax;
			Deceleration_X = (G_Motor_W_Target * G_Motor_W_Target)
					/ (4 * W_Ac)* PI;
			break;
		}
		if (Angle - G_Motor_Angle < Deceleration_X) {
			Acceleration_X = fabs(G_Motor_Angle); //加速時の補填
			break;
		}

	}
	Constant_X = fabs(Angle - (Acceleration_X + Deceleration_X));

	G_Motor_W_Ac = 0;
	G_Motor_Angle = 0;

	while (1) {
		//printf("Co____G_Motor_W_Target:%f----G_Motor_Angle:%f\n\r",
		//G_Motor_W_Target, G_Motor_Angle + Acceleration_X);
		if (SANKAKU == 1) {
			SANKAKU = 0;
			break;
		}
		if (fabs(G_Motor_Angle) > Constant_X) {
			break;
		}
	}

	G_Motor_Count = 0;
	G_Motor_W_Ac = -W_Ac;
	float Df3 = G_Motor_W_Target;
	float t3 = PI * Df3 / 2 / W_Ac;
	while (1) {
		G_Motor_W_Target = Df3 / 2
				* (1 - cos(2 * W_Ac / Df3 * (G_Motor_Count - t1)));
		//printf("%f\n\r",G_Motor_V_Target);
		if (t3 <= G_Motor_Count) {
			break;
		}
	}
	Enc_Sigma_error = 0;
	//Gyro_Sigma_error = 0;
	G_Motor_Flag = 0;
}

void Motor_Wallcut_ST(float Vmax, float X, int direction) {		//壁の有無を変数に入れる
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;
	float X_act = 0;

	Enc_Sigma_error = 0;
	//Gyro_Sigma_error = 0;

	G_Motor_X = 0;
	while (1) {
		if (G_Motor_X > X * 0.50) {
			X_act = G_Motor_X;
			break;
		}
	}
	if (direction == 0) { //左旋回
		if (G_Wall_data[1] == 1) {
			while (1) {
				if (g_sensor_av[2] < Cut_L) {
					break;
				}
				LED_ON_L();
			}
		}
	} else if (direction == 1) { //右旋回
		if (G_Wall_data[2] == 1) {
			while (1) {
				if (g_sensor_av[1] < Cut_R) {
					break;
				}
				LED_ON_R();
			}
		}
	}
	LED_Reset();
	G_Motor_X = 0;
	while (1) {
		if (G_Motor_X > X - X_act) {
			break;
		}
	}
	PID_Mode = 0;
}

void Motor_Wallcut_END(float Vmax, float X, int direction) {
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;

	Wall_search();
	int Wall_L = G_Wall_data[1];
	int Wall_R = G_Wall_data[2];

	G_Motor_X = 0;

	while (1) {
		if (G_Motor_X > X) {
			break;
		}
		if (Wall_L == 1) {
			if (g_sensor_av[2] < Cut_L) {
				break;
			}
		}
		if (Wall_R == 1) {
			if (g_sensor_av[1] < Cut_R) {
				break;
			}
		}
	}
	PID_Mode = 0;
}

void Motor_Wallcut_ST_NANAME(float Vmax, float X, int direction) {
	G_Motor_Flag = 1;
	PID_Mode = 2; //2

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	Wall_search();
	G_Motor_X = 0;
	if (direction == 0) { //左旋回
		while (1) {
			if (g_sensor_av[2] < Cut_L_NA) {
				break;
			}
		}
	} else if (direction == 1) { //右旋回
		while (1) {
			if (g_sensor_av[1] < Cut_R_NA) {
				break;
			}
		}
	}
	G_Motor_X = 0;
	while (1) {
		if (G_Motor_X > X) {
			break;
		}
	}
	PID_Mode = 0;
}

void Motor_Wallcut_END_NANAME(float Vmax, float X, int direction) {
	G_Motor_Flag = 1;
	PID_Mode = 2; //2

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;

	G_Motor_X = 0;

	while (1) {
		if (G_Motor_X > X) {
			break;
		}
	}
	Wall_search();
	if (G_Wall_data[2] == 1 && direction == 0) {
		while (1) {
			if (g_sensor_av[1] < Cut_R_NA) {
				break;
			}
		}
	} else if (G_Wall_data[1] == 1 && direction == 1) {
		while (1) {
			if (g_sensor_av[2] < Cut_L_NA) {
				break;
			}
		}
	}

	PID_Mode = 0;
}

void Motor_Robot_Alignment() {
	G_Motor_Flag = 4;
	PID_Mode = 3;
	G_Motor_V_Target = 0;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;
	Gyro_Sigma_error = 0;
	Enc_Sigma_error = 0;
	G_Motor_Count = 0;
	while (1) {
		if (G_Motor_Count > Alignment_TIME) {
			break;
		}
	}
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

