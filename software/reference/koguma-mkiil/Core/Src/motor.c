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
#include "PillarEdge.h"
#include "Maze.h"
#include "WallDistance.h"
#include "UI.h"
#include "Failsafe.h"
#include "stdlib.h"
#include <stdio.h>

float G_Robot_speed = 0;

float G_Motor_X = 0;
int G_Log_Phase = LOG_PH_NONE;	//いま何の動作中か(ログ用、LOG_PH_*)
/* リセットしない走行距離[mm](2026-10-12)。G_Motor_X は動作ごとに0に戻るので、
 * 柱の端の位置(Wallsensor.c の Wall_Edge_Sample)はこちらで記録する */
float G_Odo_X = 0;
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
float Ksi_AT = 0.0045; //2026-10-21: 0.003 から 1.5倍。ターン後の直線で向きのずれ(約0.5°)を戻しきれなかった(0.5° → 0.39°)
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

/* 直線の向きの基準(Gyro_Sigma_error、向きのずれの積分)を、直線から直線へ続けて走るときは引き継ぐ(2026-10-21)。
 * 今までは直線の関数を呼ぶたびに 0 に戻していた。ずれた向きを戻すのに 0.5 秒ほどかかるので、区間に分けて走ると
 * 戻しきる前に基準が変わっていた(モード5 No.0 のターン後、後距離 50mm と減速 180mm で合わせて -0.9°)。
 * ターン・壁合わせ・後退(G_Motor_Flag >= 2)のあとは今までどおり 0 から始める(ターン中の積分は直線の向きの基準ではない)。
 * 止まったとき(Motor_Stop)と Motor_Setup も今までどおり 0 にする。0 にすると今までどおり(直線の関数を呼ぶたびに 0) */
int G_Heading_Keep = 1;
static volatile int s_heading_turned = 1;	//前の直線のあとに G_Motor_Flag >= 2 の動きがあったか(割り込みで立てる)
static void Straight_Heading_Start(void) {
	if (!G_Heading_Keep || s_heading_turned) {
		Gyro_Sigma_error = 0;
	}
	s_heading_turned = 0;
}
int Turn_Mode = 0;
int Fun_Flag = 0;

int Sula_Flag = 0;
static int Suction_Duty = 0;	//今の吸引のduty(Suction_Start/Suction_changeで指定した値)

float Alignment_TIME = 0.5;	//前壁合わせの上限[s]
/* 前壁合わせを早く終える(2026-10-20)。前壁合わせは比例制御だけなので、目標の近くでは出力が小さく摩擦で止まり、
 * 止まったあとは待っても良くならない。それでも毎回0.5秒続けていた。車輪が止まった状態が Align_Still_Time 続いたら終わる
 * (はじめは止まっているので Align_Min_Time までは終わらない)。ジャイロは吸引の振動で揺れるので、車輪の速度で見る */
float Align_Min_Time = 0.10f;	//これより早くは終わらない[s]
float Align_Still_Time = 0.05f;	//止まっている状態がこれだけ続いたら終わる[s]
float Align_Still_V = 10.0f;	//左右の車輪の速さがこれより小さければ止まっている[mm/s]
int G_Align_Count = 0;	//前壁合わせの回数(電源を入れてから)
int G_Align_Timeout = 0;	//そのうち上限まで続いた回数
float G_Align_Time_Sum = 0;	//かかった時間の合計[s]
float G_Align_Err_Sum = 0;	//終わったときの前後の誤差の大きさの合計[mm](左右の距離の平均 - Align_Front_mm)
float G_Align_Diff_Sum = 0;	//終わったときの左右の距離の差の大きさの合計[mm](向きのずれ)


float Cut_R = 80;
float Cut_L = 80;

float Cut_R_NA = 150;
float Cut_L_NA = 150;

/* 壁切れを柱で見る(2026-10-12、設計は docs/pillar_cut_design.md)。最短走行の間だけ main.c の
 * Short_WallCut_Pillar に従って立てる(探索・斜めの壁切れはいつも従来どおり)。
 * 0: 従来。前距離は「Xの半分から、曲がる側の壁があればセンサ値が Cut_* を下回るまで待ち、残り X/2」、
 *    後距離は「壁があればセンサ値が Cut_* を下回ったら終わる。X は最大」。
 * 1: 柱。割り込みで見つけた柱の端から区画の境目を求め(Wallsensor.c の Edge_Cut_Find)、
 *    前距離は「区画中心(前距離の始め)から X」を、境目の実測で直す。壁の有無によらず同じ方法。
 *    後距離はXだけ走る(ターンのあと次の境目の柱はターンの終わりのすぐ後に見えるので、後距離のうちには使えない。
 *    その境目は次の前距離で使う) */
int G_WallCut_Pillar = 0;
/* 前距離で、境目の推定を探す範囲(区画中心 ± 90 からの ±)[mm]。
 * 2026-10-18 の最短走行(2000)で、境目の推定が「区画中心 + 90」から 13〜38mm 先に出ていた(距離のずれ)ので
 * 20 → 40 にした。隣の境目までは 180mm あるので、40 でも取り違えない */
float Edge_Win = 40;
/* 柱のときの前距離の読み替え: 曲がり始め = 境目 - 90 + (X × Pillar_X_Scale + Pillar_X_Offs)。
 * 前距離の X は従来の壁切れ(壁が切れてから X の半分で曲がる)に合わせて調整してある。2026-10-18 の最短走行(2000)では、
 * 曲がる側に壁がある前距離で、従来は実際には「区画中心 + X/2 - 5mm」くらいで曲がっていたので、その値から始める。
 * ターンの種類ごとに合わせ直すまでのつなぎ(1.0 / 0 にすると X そのまま)。境目が見つからないときは 区画中心 + Xp + Pillar_Wait_Ofs */
float Pillar_X_Scale = 0.5f;
float Pillar_X_Offs = -5.0f;
/* 柱のとき、境目が見つからないうちに曲がり始めるのは 区画中心 + Xp + Pillar_Wait_Ofs まで待ってから。
 * 曲がる側は境目の先が開いているので、前の境目の柱の端は必ずある。2026-10-18 の最短走行(2000)では、
 * 境目は「区画中心 + 90」より 10〜38mm(平均約22mm)先にあり、その端は前距離の始めの前後(約 -10〜+15mm)で見える。
 * 距離だけ(区画中心 + X)で曲がり始めると、端が見える前に曲がってしまうことがあった(端が1mm後に見えた) */
float Pillar_Wait_Ofs = 25.0f;
/* X がこれより小さい前距離(2400の斜め入り、X = 3)は、曲がる側の境目だけを使い、見つからなければ従来と同じ
 * 区画中心 + X で曲がり始める。2026-10-18 の 2400 の最短(柱)で、このターンだけ従来より 19〜25mm 遅く曲がり始め
 * (反対側の境目を使った +30.1、待つ距離いっぱいの +23.7。従来は +5)、斜めの出口のあとで右の壁にぶつかった。
 * 2000 の前距離(X = 5 以上)は今までどおり */
float Pillar_Short_X = 5.0f;
#define EDGE_SRC_BEHIND 4	//前距離で、後ろの境目(区画中心 - 90)を使った
#define EDGE_SRC_CONV_CUT 16	//従来の前距離で、壁切れした(Wallsensor.h の EDGE_SRC_CONV_* と同じ値)
#define EDGE_SRC_CONV_NONE 32	//従来の前距離で、曲がる側に壁がなく距離だけで走った

/* 壁があるときの壁切れの判定に使うセンサ値(2026-10-12)。
 * 0 は従来どおり g_sensor_av(5回の移動平均。約2ms遅れるので、反応する位置が速度×2ms 後ろにずれる。
 * 1000で2mm、2400で4.8mm)。1 は直近3回の中央値(遅れ約1ms)。
 * 1にすると壁切れの位置が変わるので、前距離・後距離を合わせ直す。斜めの壁切れ(Cut_*_NA)も同じ */
int G_WallCut_Fast = 0;

float Run_Voltage;

float Aff = 0.65;
float Bff = 0.50;

float FF_offset = 15.0;

void Motor_Setup() {
	if (Failsafe_Flag() == 1) {	//フェイルセーフで止まっているあいだは、モーターを動かさない(解除は main の Failsafe_Wait_Reset)
		return;
	}
	/* 走り出す前にジャイロのゼロ点を測る(0.3秒。2026-10-19)。動いているときは測らない */
	if (fabsf(G_Tire_Speed_L) < 20 && fabsf(G_Tire_Speed_R) < 20) {
		Gyro_Calibrate(300);
	}
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
	G_Log_Phase = LOG_PH_STOP;
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
	if (Failsafe_Flag() == 1) {	//フェイルセーフで止まっているあいだは、吸引も回さない
		return;
	}
	Suction_Duty = duty;
	Fun_Flag = 1;
	if (duty == 50) {
		Sula_Flag = 1;
	} else if (duty >= 70) {
		Sula_Flag = 2;
	} else if (duty <= 20) {
		Fun_Flag = 0;
	}
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 99);	//周期 20us。車輪のPWM(TIM3)とぴったり同じにする(PL_sensor.c の sensor_pwm_sync)
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
	Suction_Duty = duty;
	duty = duty * (15.8 / Run_Voltage);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
}

void Suction_Stop() {
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	Sula_Flag = 0;
	Fun_Flag = 0;
	Suction_Duty = 0;
}

/* 一時的に吸引を上げる(最短走行の最初のターンなど)。今の吸引が duty 以上なら何もしない。
 * 吸引と一緒に、直線とターンのPIDゲインの組(Fun_Flag / Sula_Flag)も Suction_Start(duty) と同じにする。
 * 吸引が上がりきるまで wait[ms] 待つので、止まっているときに呼ぶ。Suction_Boost_End() で元に戻す */
static int Boost_On = 0;
static int Boost_Duty_Prev = 0;
static int Boost_Fun_Prev = 0;
static int Boost_Sula_Prev = 0;

void Suction_Boost_Start(int duty, int wait) {
	Boost_On = 0;
	if (Suction_Duty >= duty) {
		return;
	}
	Boost_Duty_Prev = Suction_Duty;
	Boost_Fun_Prev = Fun_Flag;
	Boost_Sula_Prev = Sula_Flag;
	Suction_change(duty);
	Fun_Flag = 1;
	if (duty >= 70) {
		Sula_Flag = 2;
	} else if (duty == 50) {
		Sula_Flag = 1;
	}
	Boost_On = 1;
	HAL_Delay(wait);
}

void Suction_Boost_End() {
	if (!Boost_On) {
		return;
	}
	Suction_change(Boost_Duty_Prev);
	Fun_Flag = Boost_Fun_Prev;
	Sula_Flag = Boost_Sula_Prev;
	Boost_On = 0;
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
		float wall = Wall_Control_Update();	//横壁制御はここで1msに1回だけ計算する
		Motor_PWM_L = Motor_FF_ST + Motor_FB_ST + wall; //
		Motor_PWM_R = Motor_FF_ST + Motor_FB_ST - wall; //
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
	if (G_Motor_Flag >= 2) {	//ターン・壁合わせ・後退: 次の直線は向きの基準を 0 から始める
		s_heading_turned = 1;
	}
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

	if (G_Motor_Flag == 1) { //直線: W は横壁制御(左の壁に近いと正 -> 右へ回す)
		/* 以前は直前のターンの向き(Turn_Mode)で符号が決まり、左ターンのあとは
		 * 壁に寄る向きに効いて、左右のPWMへの横壁制御と打ち消し合っていた */
		Gyro_error = -W - G_Gyro_Z;
	} else if (Turn_Mode == 1) { //右向き
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
	G_Log_Phase = (PID_Mode == 2) ? LOG_PH_DIAG : LOG_PH_STRAIGHT;
	G_Motor_Flag = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Enc_Sigma_error = 0;
	Straight_Heading_Start();

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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}

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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (G_Motor_X > Constant_X) {
			break;
		}

	}
	G_Motor_Count = 0;
	float Df3 = Vmax - Vend;
	float t3 = PI * Df3 / 2 / Ac;
	G_Motor_Ac = -Ac;
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
	G_Log_Phase = LOG_PH_STRAIGHT;
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
	G_Log_Phase = LOG_PH_STRAIGHT;
	G_Motor_Flag = 1;
	PID_Mode = 1;
	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Enc_Sigma_error = 0;
	Straight_Heading_Start();

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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (G_Motor_X > Constant_X) {
			break;
		}
	}

	// --- 3. 減速フェーズ（加速度は2倍の Ac_dec を使用） ---
	G_Motor_Count = 0;
	float Df3 = Vmax - Vend;
	float t3 = PI * Df3 / 2 / Ac_dec;

	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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

/* スラロームの前の直進(Motor_Sula_before / Motor_Sula_ST)で、前壁が見えているときは、
 * 前センサで測った前壁までの距離からターンを始める位置を決める。
 * 区画の境目にいるとき、その区画の前壁までは 84 + 90 = 174mm。
 * 前距離 X なら、前壁までが 174 - X mm になったところでターンに入る
 * (以前はセンサ値130で打ち切っていたが、これは境目より手前の約178mmで、
 *  前壁があるとすぐ打ち切られて境目からターンしていた) */
#define SULA_BOUNDARY_FRONT_MM 174.0f
#define SULA_FRONT_WALL_TH 60	//境目でこれより大きければ前壁あり(境目で前壁は約140)

static float Front_Wall_mm(void) {
	return (WallDist_mm(WALLDIST_FL, g_sensor[0][0])
			+ WallDist_mm(WALLDIST_FR, g_sensor[3][0])) / 2;
}

/* ---- 探索の直進の壁切れ(2026-10-18 作り直し、設計は docs/search_wallcut_design.md) ----
 * 探索の行動は「判断位置」(Motor_Sula_before で壁を読む位置。区画の境目のはず)で始まる。1区画まっすぐ進むときは、
 * 区画の Search_Look_From(4割)を過ぎたら次の境目の柱の端(Wallsensor.c の Edge_Cut_Find、最短走行の柱の壁切れと
 * 同じ推定)を探し、dc = 推定 - 距離から予想した境目 を出す。
 *  - 探索のターン(前距離15・後距離50など)は、今の判断位置のまま調整してある。判断位置が本当の境目からずれていても
 *    (最短走行の2000では、ターンのあとの境目は予想より 13〜38mm 先だった)、ターンはそのずれ込みで合っている。
 *    推定にそのまま合わせると判断位置が動いてターンの位置がずれるので、「今のずれの中央値」Search_Edge_Ref を基準にし、
 *    dc - Search_Edge_Ref だけ直す(ずれの平均は今のまま、ばらつきだけ消す)
 *  - Search_WallCut: 0 使わない、1 記録だけ(走りは今までどおり。Search_Edge_Ref を測る)、2 直す
 *  - 直すとき: 推定が 予想 + Ref ± Search_Edge_Win にあり、左右の推定が食い違っていない(EDGE_SRC_DISAGREE でない)ときだけ。
 *    直す量 = Search_Corr_Gain × (dc - Ref) を ± Search_Corr_Max まで
 *  - 両側とも壁が続くところは端がないので、どの段階でも何もしない
 * 記録は Search_Cut_Print(出力のモード4 No.7、SCUT)。ビューアが基準のずれ(中央値)を出す */
int Search_WallCut = 1;
float Search_Edge_Ref = 0;		//基準のずれ[mm](記録で測った dc の中央値を書き写す)
float Search_Edge_Win = 20;		//直すとき、予想 + Ref ± これの推定だけ使う[mm]
float Search_Rec_Win = 60;		//記録するとき、予想 ± これの推定を残す[mm](隣の境目までは180mm)
float Search_Corr_Max = 10;		//1回に直す量の上限[mm]
float Search_Corr_Gain = 1.0f;	//dc - Ref のうち、どれだけ直すか
float Search_Look_From = 0.4f;	//区画(180mm)のこの割合を進んでから探す

#define SCUT_N 200
typedef struct {
	float b;		//予想した次の境目(G_Odo_X)
	float dc;		//推定 - 予想[mm](推定がなければ 0)
	float corr;		//直した量[mm]
	uint16_t act;	//探索を始めてから何番目の行動か(1から)
	int8_t prev;	//直前の行動(G_Robot_Lastaction: 0 直進、1 右、2 Uターン、3 左、5 まだなし。止まって向きを変えたときは進んだ向き)
	int8_t src;		//推定(EDGE_SRC_*、0 は見つからず)
	int8_t x, y, dir;	//区画と向き(Maze_Wall_Update のあと = いま入った区画)
} SearchCutRec;
static SearchCutRec Scut[SCUT_N];
static int Scut_Count = 0;
static int Scut_Lost = 0;	//入りきらなかった数
static uint16_t Scut_Action = 0;	//探索の行動の数(Motor_Sula_before のたびに1増やす)

/* ---- 迷路に書く前壁(2026-10-19、docs/search_stability_research.md) ----
 * 迷路には左右の前センサがどちらも閾値を超えたときだけ前壁を書く(Maze.c)。同じ判断位置で前距離を決める front_wall は
 * 左右の平均(SULA_FRONT_WALL_TH)で見ているので、左右が分かれた(片方だけ超えた)ときは2つの判断が食い違うことがある。
 * 境目で前壁があれば左右とも約140なので、壁があって片方が60を切ることは考えにくく、むしろ壁がないのに片方が柱などを
 * 見ていることがありうる。そこで、まず分かれたときを記録して(FWALL)、本当の壁と比べてから決める。
 *  - Front_Wall_Map: 0 今までどおり、1 記録だけ(迷路は今までどおり)、2 平均でも壁ありにする(Wallsensor.c の Wall_Front_For_Map) */
int Front_Wall_Map = 1;
#define FWALL_N 64
typedef struct {
	uint16_t act;	//探索を始めてから何番目の行動か
	int16_t fl, fr;	//前センサ(左・右、g_sensor_av10)
	int8_t x, y, dir;	//いま入った区画と向き(前壁はこの区画の dir 側)
	int8_t avg;		//左右の平均で見たら壁ありか(Wall_Front_For_Map)
} FrontWallRec;
static FrontWallRec Fwall[FWALL_N];
static int Fwall_Count = 0;
static int Fwall_Lost = 0;

/* Motor_Sula_before の Wall_search() のあとに呼ぶ。迷路に書く前壁を G_Maze_Front_Override に入れ、左右が分かれたら記録する */
static void Front_Wall_Decide(void) {
	int split = (G_Wall_data[0] != G_Wall_data[3]);
	int avg = Wall_Front_For_Map();
	G_Maze_Front_Override = (Front_Wall_Map == 2) ? avg : -1;
	if (!split || Front_Wall_Map == 0) {
		return;
	}
	if (Fwall_Count >= FWALL_N) {
		Fwall_Lost++;
		return;
	}
	FrontWallRec *r = &Fwall[Fwall_Count++];
	r->act = Scut_Action;
	r->fl = (int16_t) g_sensor_av10[0];
	r->fr = (int16_t) g_sensor_av10[3];
	r->avg = (int8_t) avg;
	r->x = r->y = r->dir = -1;	//区画は Maze_Wall_Update のあとに入れる(Front_Wall_Rec_Cell)
}

static void Front_Wall_Rec_Cell(void) {
	if (Fwall_Count > 0 && Fwall[Fwall_Count - 1].act == Scut_Action && Fwall[Fwall_Count - 1].x < 0) {
		FrontWallRec *r = &Fwall[Fwall_Count - 1];
		r->x = (int8_t) G_Robot_MAZE_X;
		r->y = (int8_t) G_Robot_MAZE_Y;
		r->dir = (int8_t) G_Robot_Direction;
	}
}

void Search_Cut_Reset(void) {
	Scut_Count = 0;
	Scut_Lost = 0;
	Scut_Action = 0;
	Fwall_Count = 0;
	Fwall_Lost = 0;
}

int Search_Cut_Has_Data(void) {
	return Scut_Count > 0 || Fwall_Count > 0;
}

void Search_Cut_Print(void) {
	printf("\r\n#BEGIN FWALL n=%d lost=%d mode=%d th=%d,%d ratio=%.2f\r\n", Fwall_Count, Fwall_Lost, Front_Wall_Map,
			Wall_threshold[0], Wall_threshold[3], Front_Wall_Min_Ratio);
	printf("act,x,y,dir,fl,fr,avg\r\n");
	for (int i = 0; i < Fwall_Count; i++) {
		const FrontWallRec *r = &Fwall[i];
		printf("%d,%d,%d,%d,%d,%d,%d\r\n", r->act, r->x, r->y, r->dir, r->fl, r->fr, r->avg);
	}
	printf("#END FWALL\r\n");

	printf("\r\n#BEGIN SCUT n=%d lost=%d mode=%d ref=%.1f win=%.0f rec_win=%.0f max=%.0f gain=%.2f look=%.2f align_n=%d align_ms=%.0f align_to=%d align_mm=%d align_err=%.1f align_diff=%.1f gyro_ofs=%.3f gyro_sd=%.3f gyro_rej=%d gyro_torn=%lu gyro_try=",
			Scut_Count, Scut_Lost, Search_WallCut, Search_Edge_Ref, Search_Edge_Win, Search_Rec_Win, Search_Corr_Max,
			Search_Corr_Gain, Search_Look_From, G_Align_Count,
			G_Align_Count ? 1000.0f * G_Align_Time_Sum / G_Align_Count : 0.0f, G_Align_Timeout, Align_Use_mm,
			G_Align_Count ? G_Align_Err_Sum / G_Align_Count : 0.0f, G_Align_Count ? G_Align_Diff_Sum / G_Align_Count : 0.0f,
			gyro_z_offset,
			G_Gyro_Offset_SD, G_Gyro_Cal_Reject, (unsigned long) G_Gyro_Torn);
	/* 最近測った順に「ばらつき:平均」(使わなかったものも) */
	int k0 = (G_Gyro_Cal_Count > GYRO_CAL_TRY_N) ? G_Gyro_Cal_Count - GYRO_CAL_TRY_N : 0;
	for (int k = k0; k < G_Gyro_Cal_Count; k++) {
		printf("%s%.2f:%.3f", (k > k0) ? "," : "", G_Gyro_Cal_SD[k % GYRO_CAL_TRY_N], G_Gyro_Cal_Mean[k % GYRO_CAL_TRY_N]);
	}
	printf("\r\n");
	printf("act,prev,x,y,dir,b,dc,corr,src\r\n");
	for (int i = 0; i < Scut_Count; i++) {
		const SearchCutRec *r = &Scut[i];
		printf("%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%d\r\n", r->act, r->prev, r->x, r->y, r->dir, r->b, r->dc, r->corr,
				r->src);
	}
	printf("#END SCUT\r\n");
}

/* 探索の1区画の直進。X は今の位置から次の境目までの距離(判断位置から始まる行動の、Motor_Sula_before のあとの残り) */
void Motor_Search_Straight(float V, float X) {
	if (Search_WallCut == 0) {
		Motor_trapezoid_PID(V, V, V, 5000, X);
		return;
	}
	G_Log_Phase = LOG_PH_STRAIGHT;
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = V;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Enc_Sigma_error = 0;
	Straight_Heading_Start();
	G_Motor_Count = 0;

	int correct = (Search_WallCut == 2);
	float b = G_Odo_X + X;	//予想した次の境目
	float lo = correct ? b + Search_Edge_Ref - Search_Edge_Win : b - Search_Rec_Win;
	float hi = correct ? b + Search_Edge_Ref + Search_Edge_Win : b + Search_Rec_Win;
	float look = X - 180.0f * (1.0f - Search_Look_From);	//ここまで進んだら端を探す
	float corr = 0, dc = 0;
	int src = 0;
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (G_Motor_X >= look) {
			float cc;
			int sr;
			if (Edge_Cut_Find(lo, hi, EDGE_SRC_R | EDGE_SRC_L, &cc, &sr)) {
				dc = cc - b;
				src = sr;
				if (correct && !(sr & EDGE_SRC_DISAGREE)) {
					corr = Search_Corr_Gain * (dc - Search_Edge_Ref);
					if (corr > Search_Corr_Max) corr = Search_Corr_Max;
					if (corr < -Search_Corr_Max) corr = -Search_Corr_Max;
				}
			}
		}
		if (G_Motor_X >= X + corr) {
			break;
		}
	}
	if (Scut_Count < SCUT_N) {
		SearchCutRec *r = &Scut[Scut_Count++];
		r->b = b;
		r->dc = dc;
		r->corr = corr;
		r->act = Scut_Action;
		r->prev = (int8_t) G_Robot_Lastaction;
		r->src = (int8_t) src;
		r->x = (int8_t) G_Robot_MAZE_X;
		r->y = (int8_t) G_Robot_MAZE_Y;
		r->dir = (int8_t) (G_Robot_Direction % 4);
	} else {
		Scut_Lost++;
	}
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

void Motor_Sula_before(float Vst, float Vmax, float Vend, float Ac, float X) {
	Scut_Action++;	//探索の直進の壁切れの記録で、何番目の行動かを残す
	G_Log_Phase = LOG_PH_PRE;
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Straight_Heading_Start();
	//Enc_Sigma_error = 0;

	float X0 = X;
	int front_wall = (Wall_Flont_Av() > SULA_FRONT_WALL_TH);
	if (front_wall) {		//壁補正前進用
		X = X + 15;
	}

	Wall_search();
	Front_Wall_Decide();	//迷路に書く前壁(左右が分かれたら記録する。2026-10-19)
	if (Failsafe_Flag() == 0) {	//止まったあとの、ずれた位置で読んだ壁は書かない
		Maze_Wall_Update();
	}
	G_Maze_Front_Override = -1;
	Front_Wall_Rec_Cell();

	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (G_Motor_X > X) {
			break;
		}

		if (front_wall && Front_Wall_mm() <= SULA_BOUNDARY_FRONT_MM - X0) {
			break;
		}
	}

	PID_Mode = 0;
	G_Motor_Flag = 0;

}

void Motor_Sula_ST(float Vst, float Vmax, float Vend, float Ac, float X) {
	G_Log_Phase = LOG_PH_POST;
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Straight_Heading_Start();
	//Enc_Sigma_error = 0;

	float X0 = X;
	int front_wall = (Wall_Flont_Av() > SULA_FRONT_WALL_TH);
	if (front_wall) {		//壁補正前進用
		X = X + 15;
	}

	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (G_Motor_X > X) {
			break;
		}

		if (front_wall && Front_Wall_mm() <= SULA_BOUNDARY_FRONT_MM - X0) {
			break;
		}
	}

	PID_Mode = 0;
	G_Motor_Flag = 0;

}

void Motor_trapezoid_Turn(float Angle, float Wmax, float W_Ac) {
	G_Log_Phase = LOG_PH_PIVOT;
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		G_Motor_W_Target = Df3 / 2
				* (1 - cos(2 * W_Ac / Df3 * (G_Motor_Count - t1)));
		if (t3 <= G_Motor_Count) {
			break;
		}
	}
	G_Motor_Flag = 0;
}

void Motor_Sula_COS(float V, float Angle, float Wmax, float W_Ac) {
	G_Log_Phase = LOG_PH_TURN;
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
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

/* ---- 前距離・後距離の壁切れ(G_WallCut_Pillar で従来と柱を切り替える) ----
 * tick: 待っているあいだ毎回呼ぶ関数(加速しながら走るときの目標速度の更新。なければ NULL) */
typedef void (*CutTick)(void);

static float Cut_Sensor(int i) {
	return G_WallCut_Fast ? Wall_Sensor_Med3(i) : g_sensor_av[i];
}

/* 従来の前距離。X_act(Xの半分)まで走ったところで呼ぶ。曲がる側に壁があれば、センサ値が Cut_* を
 * 下回るまで待ってから、残り X - X_act を走る。
 * x_center は前距離の始め(区画中心、G_Odo_X)。壁切れのログに、柱のときと同じ形(st / st_cut)で残す */
static void Cut_ST_Conv(int direction, float x_center, float X, float X_act, CutTick tick) {
	/* 壁の有無は待ち始めの時点で決める(G_Wall_data は1msごとに更新され、壁が終わると0になるため) */
	int cut_wall_L = G_Wall_data[1];
	int cut_wall_R = G_Wall_data[2];
	int src = EDGE_SRC_CONV_NONE;
	float c = 0;
	Edge_Log_Mark(EDGE_LOG_ST, direction, x_center, X, 0);
	if (direction == 0) { //左旋回
		if (cut_wall_L) {
			while (1) {
				if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
					break;
				}
				if (tick) tick();
				if (Cut_Sensor(2) < Cut_L) {
					break;
				}
				LED_ON_L();
			}
			Edge_Log_Mark(EDGE_LOG_CUT, 1, G_Odo_X, 0, 0);
			src = EDGE_SRC_CONV_CUT;
			c = G_Odo_X;
		}
	} else if (direction == 1) { //右旋回
		if (cut_wall_R) {
			while (1) {
				if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
					break;
				}
				if (tick) tick();
				if (Cut_Sensor(1) < Cut_R) {
					break;
				}
				LED_ON_R();
			}
			Edge_Log_Mark(EDGE_LOG_CUT, 0, G_Odo_X, 0, 0);
			src = EDGE_SRC_CONV_CUT;
			c = G_Odo_X;
		}
	}
	LED_Reset();
	float X0 = G_Motor_X;
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (tick) tick();
		if (G_Motor_X - X0 > X - X_act) {
			break;
		}
	}
	Edge_Log_Mark(EDGE_LOG_ST_CUT, src, G_Odo_X, c, 0);
}

/* 柱の前距離。前距離の始めは区画中心(x_center、G_Odo_X)で、そこから X 走ったら曲がり始める。
 * 境目の推定があれば、その実測で曲がり始めを直す:
 *   前の境目(x_center + 90 ± Edge_Win)があれば  曲がり始め = 境目 + Xp - 90
 *   なければ後ろの境目(x_center - 90 ± Edge_Win)  曲がり始め = 境目 + 90 + Xp
 *   どちらもなければ x_center + Xp + Pillar_Wait_Ofs(距離だけ。それまでに端が見えたらそちらで直す)
 * X < Pillar_Short_X のときは曲がる側の境目だけを使い、どちらもなければ x_center + X
 * Xp = X × Pillar_X_Scale + Pillar_X_Offs(従来に合わせて調整した X の読み替え)
 * 横センサは区画中心で柱の真ん中を見る向きなので、前の境目の柱の端は区画中心の少し先で見える。
 * Xがそれより短いと前の境目は間に合わず、後ろの境目(前の区間で見たもの)を使う */
static void Cut_ST_Pillar(int direction, float x_center, float X, CutTick tick) {
	float b0 = x_center + 90.0f;
	float b1 = x_center - 90.0f;
	float Xp = X * Pillar_X_Scale + Pillar_X_Offs;	//境目から合わせるときの X
	int short_x = X < Pillar_Short_X;
	int sides = short_x ? (direction == 1 ? EDGE_SRC_R : EDGE_SRC_L) : (EDGE_SRC_R | EDGE_SRC_L);
	float wait = short_x ? x_center + X : x_center + Xp + Pillar_Wait_Ofs;
	float target = wait;
	float c = 0;
	int src = 0;
	Edge_Log_Mark(EDGE_LOG_ST, direction, x_center, X, 0);
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (tick) tick();
		float cc;
		int sr;
		if (Edge_Cut_Find(b0 - Edge_Win, b0 + Edge_Win, sides, &cc, &sr)) {
			target = cc + Xp - 90.0f;
			c = cc;
			src = sr;
		} else if (Edge_Cut_Find(b1 - Edge_Win, b1 + Edge_Win, sides, &cc, &sr)) {
			target = cc + 90.0f + Xp;
			c = cc;
			src = sr | EDGE_SRC_BEHIND;
		} else {
			target = wait;
			src = 0;
		}
		if (G_Odo_X >= target) {
			break;
		}
	}
	Edge_Log_Mark(EDGE_LOG_ST_CUT, src, G_Odo_X, c, target);
}

/* 後距離。G_Motor_X を0にして Wall_search() したあとに呼ぶ。
 * 従来: 壁があった側のセンサ値が Cut_* を下回ったら終わる。X は最大。
 * 柱: X だけ走る(Cut_ST_Pillar の説明) */
static void Cut_END(float X, CutTick tick) {
	int Wall_L = G_Wall_data[1];
	int Wall_R = G_Wall_data[2];
	int src = 0;
	Edge_Log_Mark(EDGE_LOG_END, 0, G_Odo_X, X, 0);
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (tick) tick();
		if (G_Motor_X > X) {
			break;
		}
		if (G_WallCut_Pillar) {
			continue;
		}
		if (Wall_L && Cut_Sensor(2) < Cut_L) {
			src = 2;
			break;
		}
		if (Wall_R && Cut_Sensor(1) < Cut_R) {
			src = 1;
			break;
		}
	}
	Edge_Log_Mark(EDGE_LOG_END_CUT, src, G_Odo_X, 0, 0);
}

void Motor_Wallcut_ST(float Vmax, float X, int direction) {		//壁の有無を変数に入れる
	G_Log_Phase = LOG_PH_PRE;
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;
	float X_act = 0;

	Enc_Sigma_error = 0;
	//Gyro_Sigma_error = 0;

	G_Motor_X = 0;
	float x_center = G_Odo_X;	//前距離の始め(区画中心)
	if (G_WallCut_Pillar) {
		Cut_ST_Pillar(direction, x_center, X, NULL);
		PID_Mode = 0;
		return;
	}
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (G_Motor_X > X * 0.50) {
			X_act = G_Motor_X;
			break;
		}
	}
	Cut_ST_Conv(direction, x_center, X, X_act, NULL);
	PID_Mode = 0;
}

void Motor_Wallcut_END(float Vmax, float X, int direction) {
	G_Log_Phase = LOG_PH_POST;
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Straight_Heading_Start();

	Wall_search();
	G_Motor_X = 0;
	Cut_END(X, NULL);
	PID_Mode = 0;
}

void Motor_Wallcut_ST_NANAME(float Vmax, float X, int direction) {
	G_Log_Phase = LOG_PH_PRE_DIAG;
	G_Motor_Flag = 1;
	PID_Mode = 2; //2

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Straight_Heading_Start();
	Wall_search();
	G_Motor_X = 0;
	if (direction == 0) { //左旋回
		while (1) {
			if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
				break;
			}
			if (Cut_Sensor(2) < Cut_L_NA) {
				break;
			}
		}
	} else if (direction == 1) { //右旋回
		while (1) {
			if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
				break;
			}
			if (Cut_Sensor(1) < Cut_R_NA) {
				break;
			}
		}
	}
	G_Motor_X = 0;
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (G_Motor_X > X) {
			break;
		}
	}
	PID_Mode = 0;
}

void Motor_Wallcut_END_NANAME(float Vmax, float X, int direction) {
	G_Log_Phase = LOG_PH_POST_DIAG;
	G_Motor_Flag = 1;
	PID_Mode = 2; //2

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Straight_Heading_Start();

	G_Motor_X = 0;

	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		if (G_Motor_X > X) {
			break;
		}
	}
	Wall_search();
	if (G_Wall_data[2] == 1 && direction == 0) {
		while (1) {
			if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
				break;
			}
			if (Cut_Sensor(1) < Cut_R_NA) {
				break;
			}
		}
	} else if (G_Wall_data[1] == 1 && direction == 1) {
		while (1) {
			if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
				break;
			}
			if (Cut_Sensor(2) < Cut_L_NA) {
				break;
			}
		}
	}

	PID_Mode = 0;
}

/* ---- 加速しながらの前距離・後距離(2026-10-07、最短走行の最初のターン用) ----
 * Motor_Wallcut_ST / Motor_Wallcut_END / Motor_Wallcut_END_NANAME と同じ動きを、
 * Vst から Vmax まで cos加速(Motor_trapezoid の加速と同じ形、加速度 Ac)しながら行う。
 * Vmax < Vst なら同じ形で減速する(2026-10-07、ターンごとの速度で後距離のうちに次のターンの速度へ変える用)。
 * 加減速は関数の最初からの経過時間で決める。終わったときの目標速度は G_Motor_V_Target に残る */
static void Accel_Target_Update(float Vst, float Vmax, float Ac) {
	float Df = Vmax - Vst;
	if (Df == 0 || Ac <= 0) {
		G_Motor_V_Target = Vmax;
		G_Motor_Ac = 0;
		return;
	}
	float Dabs = fabs(Df);
	float t1 = PI * Dabs / 2 / Ac;
	if (G_Motor_Count < t1) {
		G_Motor_V_Target = Df / 2 * (1 - cos(2 * Ac / Dabs * G_Motor_Count)) + Vst;
		G_Motor_Ac = (Df > 0 ? Ac : -Ac) * sin(2 * Ac / Dabs * G_Motor_Count);
	} else {
		G_Motor_V_Target = Vmax;
		G_Motor_Ac = 0;
	}
}

static void Accel_Start(float Vst, int pid_mode) {
	G_Motor_Flag = 1;
	PID_Mode = pid_mode;
	G_Motor_V_Target = Vst;
	G_Motor_Ac = 0;
	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;
	Enc_Sigma_error = 0;
	Straight_Heading_Start();
	G_Motor_Count = 0;
	G_Motor_X = 0;
}

/* 加速しながら待つときの目標速度の更新(Cut_ST_* / Cut_END に渡す) */
static float TK_Vst, TK_Vmax, TK_Ac;
static void Tick_Accel(void) {
	Accel_Target_Update(TK_Vst, TK_Vmax, TK_Ac);
}

/* Motor_Wallcut_ST の加速版。X_pre は前距離の前に足す距離(スタート位置から区画中心までの24mmなど) */
void Motor_Wallcut_ST_Accel(float Vst, float Vmax, float Ac, float X_pre,
		float X, int direction) {
	G_Log_Phase = LOG_PH_PRE;
	Accel_Start(Vst, 1);
	TK_Vst = Vst;
	TK_Vmax = Vmax;
	TK_Ac = Ac;
	float x_center = G_Odo_X + X_pre;	//前距離の始め(区画中心)
	if (G_WallCut_Pillar) {
		Cut_ST_Pillar(direction, x_center, X, Tick_Accel);
		PID_Mode = 0;
		return;
	}
	float X_act = 0;
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		Accel_Target_Update(Vst, Vmax, Ac);
		if (G_Motor_X > X_pre + X * 0.50) {
			X_act = G_Motor_X - X_pre;
			break;
		}
	}
	Cut_ST_Conv(direction, x_center, X, X_act, Tick_Accel);
	PID_Mode = 0;
}

/* Motor_Wallcut_END の加速版 */
void Motor_Wallcut_END_Accel(float Vst, float Vmax, float Ac, float X,
		int direction) {
	G_Log_Phase = LOG_PH_POST;
	Accel_Start(Vst, 1);
	TK_Vst = Vst;
	TK_Vmax = Vmax;
	TK_Ac = Ac;
	Wall_search();
	Cut_END(X, Tick_Accel);
	PID_Mode = 0;
}

/* Motor_Wallcut_END_NANAME の加速版 */
void Motor_Wallcut_END_NANAME_Accel(float Vst, float Vmax, float Ac, float X,
		int direction) {
	G_Log_Phase = LOG_PH_POST_DIAG;
	Accel_Start(Vst, 2);
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		Accel_Target_Update(Vst, Vmax, Ac);
		if (G_Motor_X > X) {
			break;
		}
	}
	Wall_search();
	if (G_Wall_data[2] == 1 && direction == 0) {
		while (1) {
			if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
				break;
			}
			Accel_Target_Update(Vst, Vmax, Ac);
			if (Cut_Sensor(1) < Cut_R_NA) {
				break;
			}
		}
	} else if (G_Wall_data[1] == 1 && direction == 1) {
		while (1) {
			if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
				break;
			}
			Accel_Target_Update(Vst, Vmax, Ac);
			if (Cut_Sensor(2) < Cut_L_NA) {
				break;
			}
		}
	}
	PID_Mode = 0;
}

/* ---- 加速しながら曲がる最初のターン(2026-10-09) ----
 * スタートから後距離の終わりまで、目標速度を 0 から Vt まで1本の cos加速(加速度 Ac)で上げ続ける。
 * ターン中も加速するので、ターンは時間ではなく走った距離で角度を決める:
 *   基準の速度 v_ref で Motor_Sula_COS(v_ref, ang, w, w_ac) を走ったときの軌跡(曲率 = 角速度 / v_ref)を、
 *   ターンに入ってから走った距離の関数として使い、目標角速度 = 曲率 × 今の速度 にする。
 * こうすると速度に関係なく、v_ref で合わせたのと同じ軌跡を通る(G_First_Turn の前距離・角度・後距離をそのまま使える)。
 * 前距離・後距離の壁切れは Motor_Wallcut_ST_Accel / Motor_Wallcut_END_Accel / Motor_Wallcut_END_NANAME_Accel と同じ */

/* Motor_Sula_COS(v, A, W, Wac) の角速度の形(時間 t の関数)。台形にならないときは三角形 */
typedef struct {
	float W, Wac, t1, tc, T;
} SulaRef;

static void SulaRef_Init(SulaRef *r, float A, float W, float Wac) {
	float acc = PI * W * W / (4 * Wac);
	if (2 * acc > A) {
		W = sqrt(2 * Wac * A / PI);
		acc = A / 2;
	}
	r->W = W;
	r->Wac = Wac;
	r->t1 = PI * W / 2 / Wac;
	r->tc = (A - 2 * acc) / W;
	r->T = 2 * r->t1 + r->tc;
}

/* 時間 t での角速度 w [deg/s] と角加速度 wa [deg/s^2] */
static void SulaRef_At(const SulaRef *r, float t, float *w, float *wa) {
	float k = 2 * r->Wac / r->W;
	if (t < r->t1) {
		*w = r->W / 2 * (1 - cos(k * t));
		*wa = r->Wac * sin(k * t);
	} else if (t < r->t1 + r->tc) {
		*w = r->W;
		*wa = 0;
	} else if (t < r->T) {
		float u = t - r->t1 - r->tc;
		*w = r->W / 2 * (1 + cos(k * u));
		*wa = -r->Wac * sin(k * u);
	} else {
		*w = 0;
		*wa = 0;
	}
}

/* v_ref で Motor_Sula_COS(v_ref, A, W, Wac) を走ったときのターンの長さ[mm] */
float Sula_Ref_Length(float v_ref, float A, float W, float Wac) {
	SulaRef r;
	SulaRef_Init(&r, fabs(A), W, Wac);
	return v_ref * r.T;
}

static float FT_T0, FT_Vt, FT_Ac;

/* 目標速度: 0 から FT_Vt までの cos加速(Accel_Target_Update と同じ形)。時間は関数の最初から通しで数える */
static void FT_Target_Update(void) {
	float t = G_Motor_Count - FT_T0;
	float t1 = PI * FT_Vt / 2 / FT_Ac;
	if (t < t1) {
		G_Motor_V_Target = FT_Vt / 2 * (1 - cos(2 * FT_Ac / FT_Vt * t));
		G_Motor_Ac = FT_Ac * sin(2 * FT_Ac / FT_Vt * t);
	} else {
		G_Motor_V_Target = FT_Vt;
		G_Motor_Ac = 0;
	}
}

/* 最初のターンを、スタートから後距離の終わりまで加速しながら走る。
 * X_pre: 前距離の前に足す距離(スタート位置から区画中心まで)、pre/post: 前距離・後距離[mm]、
 * v_ref・Angle(左が正)・W・Wac: 軌跡を決める基準のターン、naname: 1 なら後距離は斜め、
 * Vt: 目標速度の上限、Ac: 加速度。戻り値は後距離の終わりの目標速度 */
float Motor_First_Turn_Accel(float X_pre, float pre, float post, float v_ref,
		float Angle, float W, float Wac, int naname, int direction, float Vt,
		float Ac) {
	G_Log_Phase = LOG_PH_PRE;
	SulaRef r;
	SulaRef_Init(&r, fabs(Angle), W, Wac);

	FT_Vt = Vt;
	FT_Ac = Ac;

	/* 前距離(Motor_Wallcut_ST_Accel と同じ) */
	Accel_Start(0, 1);
	FT_T0 = G_Motor_Count;
	float x_center = G_Odo_X + X_pre;	//前距離の始め(区画中心)
	if (G_WallCut_Pillar) {
		Cut_ST_Pillar(direction, x_center, pre, FT_Target_Update);
	} else {
		float X_act = 0;
		while (1) {
			if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
				break;
			}
			FT_Target_Update();
			if (G_Motor_X > X_pre + pre * 0.50) {
				X_act = G_Motor_X - X_pre;
				break;
			}
		}
		Cut_ST_Conv(direction, x_center, pre, X_act, FT_Target_Update);
	}

	/* ターン: 走った距離 s から基準の時間 s / v_ref を求め、その曲率で曲がる */
	G_Log_Phase = LOG_PH_TURN;
	PID_Mode = 0;
	Turn_Mode = (direction == 1) ? 1 : 0;
	Gyro_Sigma_error = 0;
	Enc_Sigma_error = 0;
	G_Motor_Angle = 0;
	G_Motor_X = 0;
	G_Motor_W_Target = 0;
	G_Motor_W_Ac = 0;
	G_Motor_Flag = 3;
	float L = v_ref * r.T;
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		FT_Target_Update();
		float s = G_Motor_X;
		if (s >= L) {
			break;
		}
		float w, wa;
		SulaRef_At(&r, s / v_ref, &w, &wa);
		float v = (G_Tire_Speed_L + G_Tire_Speed_R) / 2;
		if (v < 100) {
			v = 100;
		}
		float kappa = w / v_ref;					//曲率 [deg/mm]
		float dkappa = wa / (v_ref * v_ref);		//曲率の変化 [deg/mm^2]
		G_Motor_W_Target = kappa * v;
		G_Motor_W_Ac = dkappa * v * v + kappa * G_Motor_Ac;
	}
	G_Motor_W_Target = 0;
	G_Motor_W_Ac = 0;

	/* 後距離(Motor_Wallcut_END_Accel / Motor_Wallcut_END_NANAME_Accel と同じ) */
	G_Log_Phase = naname ? LOG_PH_POST_DIAG : LOG_PH_POST;
	G_Motor_Flag = 1;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	G_Motor_Angle = 0;
	G_Motor_X = 0;
	if (naname) {
		PID_Mode = 2;
		while (1) {
			if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
				break;
			}
			FT_Target_Update();
			if (G_Motor_X > post) {
				break;
			}
		}
		Wall_search();
		if (G_Wall_data[2] == 1 && direction == 0) {
			while (1) {
				if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
					break;
				}
				FT_Target_Update();
				if (Cut_Sensor(1) < Cut_R_NA) {
					break;
				}
			}
		} else if (G_Wall_data[1] == 1 && direction == 1) {
			while (1) {
				if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
					break;
				}
				FT_Target_Update();
				if (Cut_Sensor(2) < Cut_L_NA) {
					break;
				}
			}
		}
	} else {
		PID_Mode = 1;
		Wall_search();
		Cut_END(post, FT_Target_Update);
	}
	PID_Mode = 0;
	return G_Motor_V_Target;
}


void Motor_Robot_Alignment() {
	G_Log_Phase = LOG_PH_ALIGN;
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
	float still_from = 0;	//止まり始めた時刻
	while (1) {
		if (Failsafe_Flag() == 1) {	//フェイルセーフで止まったら、すぐに抜ける
			break;
		}
		float t = G_Motor_Count;
		if (t > Alignment_TIME) {
			G_Align_Timeout++;
			break;
		}
		if (fabsf(G_Tire_Speed_L) >= Align_Still_V || fabsf(G_Tire_Speed_R) >= Align_Still_V) {
			still_from = t;
		} else if (t >= Align_Min_Time && t - still_from >= Align_Still_Time) {
			break;
		}
	}
	G_Align_Count++;
	G_Align_Time_Sum += G_Motor_Count;
	{
		float fl = WallDist_mm(WALLDIST_FL, g_sensor[0][0]);
		float fr = WallDist_mm(WALLDIST_FR, g_sensor[3][0]);
		G_Align_Err_Sum += fabsf((fl + fr) * 0.5f - Align_Front_mm);
		G_Align_Diff_Sum += fabsf(fl - fr);
	}
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

