/*
 * LOG.c
 *
 *  Created on: Jun 30, 2024
 *      Author: akihi
 *
 *  走行ログ(2026-10-12 に作り直した。決め方は docs/log_design.md)。
 *  1msの割り込みで G_Log_Div 回に1回、全部の列を残す(普段は1、最短走行は4)。
 *  RAMを減らすため、走った距離のほかは単位を決めた int16 で持つ(LOG_print で元の単位に戻して出す)。
 *  出力は tools/viewer.html で読む。
 */

#include"LOG.h"
#include "stdio.h"
#include <stdint.h>

#include "PL_encoder.h"
#include "lsm6dsr.h"
#include "motor.h"
#include "PL_sensor.h"
#include "Wallsensor.h"

#define LOG_MAX 2000

/* 何回に1回残すか(1: 1msおき・2秒、4: 4msおき・8秒)。LOG_get_start の前に決める */
int G_Log_Div = 1;

/* 列ごとの単位(int16 に入れる前に掛ける値) */
#define SC_V 1.0f		//速度[mm/s]
#define SC_W 10.0f		//角速度[deg/s]、角度[deg]
#define SC_D 10.0f		//壁までの距離[mm]
#define SC_ERR 10.0f	//横壁の誤差
#define SC_PID 100.0f	//横壁制御の P項・D項
#define SC_VOLT 10.0f	//電圧(PWM)
#define SC_BATT 100.0f	//電池電圧[V]

static float L_x[LOG_MAX];			//走った距離[mm](ログ開始から)
static int16_t L_v[LOG_MAX], L_vt[LOG_MAX];		//速度・目標
static int16_t L_w[LOG_MAX], L_wt[LOG_MAX];		//角速度・目標
static int16_t L_ang[LOG_MAX];				//角度(動作ごとに0から)
static int16_t L_sL[LOG_MAX], L_sR[LOG_MAX];	//横センサの値(LPFだけ。1回だけの落ち込みも見える)
static int16_t L_dL[LOG_MAX], L_dR[LOG_MAX];	//横の壁までの距離(3回の中央値から。横壁制御が使う値)
static int16_t L_err[LOG_MAX], L_p[LOG_MAX], L_d[LOG_MAX];	//横壁の誤差、P項、D項
static int16_t L_vL[LOG_MAX], L_vR[LOG_MAX];	//電圧(PWM)
static int16_t L_batt[LOG_MAX];			//電池電圧
static int8_t L_wall[LOG_MAX];			//使った壁(1 左、2 右、3 両方、0 なし)
static int8_t L_ph[LOG_MAX];			//動作(motor.h の LOG_PH_*)

int LOG_flag = 0;
int LOG_count = 0;
static int LOG_used = 0;	//最後に取ったログの行数
static int LOG_div_used = 1;	//最後に取ったログの G_Log_Div
static int LOG_tick = 0;
static float LOG_x0 = 0;

static int16_t S16(float v, float sc) {
	float a = v * sc;
	if (a > 32767) a = 32767;
	if (a < -32767) a = -32767;
	return (int16_t) (a >= 0 ? a + 0.5f : a - 0.5f);
}

void LOG_get_start() {
	if (G_Log_Div < 1) G_Log_Div = 1;
	LOG_flag = 0;
	LOG_count = 0;
	LOG_used = 0;
	LOG_tick = 0;
	LOG_div_used = G_Log_Div;
	LOG_x0 = G_Odo_X;
	LOG_flag = 1;
}

void LOG_get_end() {
	LOG_flag = 0;
	LOG_used = LOG_count;
	LOG_count = 0;
}

int LOG_Is_On() {
	return LOG_flag;
}

int LOG_Has_Data() {
	return LOG_used > 0 || LOG_count > 0;
}

void LOG_get_interrupt() {
	if (LOG_count >= LOG_MAX) {
		LOG_flag = 0;
	}
	if (LOG_flag != 1) {
		return;
	}
	if (++LOG_tick < LOG_div_used) {
		return;
	}
	LOG_tick = 0;
	int i = LOG_count;
	L_x[i] = G_Odo_X - LOG_x0;
	L_v[i] = S16((G_Tire_Speed_L + G_Tire_Speed_R) / 2, SC_V);
	L_vt[i] = S16(G_Motor_V_Target, SC_V);
	L_w[i] = S16(G_Gyro_Z, SC_W);
	L_wt[i] = S16(G_Motor_W_Target, SC_W);
	L_ang[i] = S16(G_Motor_Angle, SC_W);
	L_sL[i] = g_sensor[2][0];
	L_sR[i] = g_sensor[1][0];
	L_dL[i] = S16(G_Wall_dL, SC_D);
	L_dR[i] = S16(G_Wall_dR, SC_D);
	L_err[i] = S16(Wall_error, SC_ERR);
	L_p[i] = S16(G_Wall_P, SC_PID);
	L_d[i] = S16(G_Wall_D, SC_PID);
	L_vL[i] = S16(Motor_Voltage_L, SC_VOLT);
	L_vR[i] = S16(Motor_Voltage_R, SC_VOLT);
	L_batt[i] = S16(g_V_batt, SC_BATT);
	L_wall[i] = (int8_t) G_Wall_St;
	L_ph[i] = (int8_t) G_Log_Phase;
	LOG_count++;
}

/* ログをUARTに出す(tools/viewer.html で読む)。
 *   #BEGIN LOG dt=間隔[s] n=行数 div=何回に1回
 *   列名の行
 *   値の行
 *   #END LOG
 * 列(単位): t[s], x 走った距離[mm], v 速度, v_target 目標[mm/s], w 角速度, w_target 目標[deg/s], angle 角度[deg],
 *   sensor_L / sensor_R 横センサの値, dist_L / dist_R 横の壁までの距離[mm], wall 使った壁(1 左 2 右 3 両方 0 なし),
 *   wall_err 横壁の誤差, wall_p P項, wall_d D項, wall_out 出力(P+D), volt_L / volt_R 電圧(PWM), batt 電池[V], phase 動作 */
void LOG_print() {
	int n = (LOG_used > 0) ? LOG_used : LOG_count;
	float dt = 0.001f * LOG_div_used;
	printf("\r\n#BEGIN LOG dt=%.3f n=%d div=%d\r\n", dt, n, LOG_div_used);
	printf("t,x,v,v_target,w,w_target,angle,sensor_L,sensor_R,dist_L,dist_R,wall,wall_err,wall_p,wall_d,wall_out,volt_L,volt_R,batt,phase\r\n");
	for (int i = 0; i < n; i++) {
		float p = L_p[i] / SC_PID, d = L_d[i] / SC_PID;
		printf("%.3f,%.1f,%d,%d,%.1f,%.1f,%.1f,%d,%d,%.1f,%.1f,%d,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%d\r\n",
				i * dt, L_x[i], L_v[i], L_vt[i], L_w[i] / SC_W, L_wt[i] / SC_W, L_ang[i] / SC_W,
				L_sL[i], L_sR[i], L_dL[i] / SC_D, L_dR[i] / SC_D, L_wall[i], L_err[i] / SC_ERR, p, d, p + d,
				L_vL[i] / SC_VOLT, L_vR[i] / SC_VOLT, L_batt[i] / SC_BATT, L_ph[i]);
	}
	printf("#END LOG\r\n");
}
