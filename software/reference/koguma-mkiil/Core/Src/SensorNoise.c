/*
 * SensorNoise.c
 *
 * 吸引のときに壁センサにのるノイズを調べるテスト(モード3 No.10、2026-10-12)。
 *
 * 止まったまま(モーターは使わない)、次の組み合わせを全部測る。
 *   読み方(cond) 4通り: LEDを点けて/消してから読むまでの待ち 0 / NOISE_SETTLE_US [us]
 *                        × ADCの取り込み時間 3 / 56 サイクル
 *   吸引 NOISE_DUTY の4通り(0 は吸引なし)
 * 1つの組み合わせごとに、吸引を変えて NOISE_WAIT_MS 待ってから NOISE_N 回(1msごと)読み、
 * 点灯時・消灯時の生のAD値と、その差(点灯 - 消灯)の平均・ばらつき・最小・最大を出す。
 * はじめの NOISE_TRACE 回は、生の値と吸引のPWMの位置もそのまま出す(tools/viewer.html でグラフにする)。
 *
 * 見かた(docs/sensor_noise.md):
 *   消灯時の値も吸引で揺れる → 電気(GNDやADC)のノイズ
 *   点灯時の値だけ揺れる     → LEDの点け方(電源の揺れ、立ち上がりの途中を読んでいる)
 *   待ちを入れると減る       → 立ち上がりの途中を読んでいた
 *   取り込み時間を伸ばすと減る → 取り込み時間が短すぎた
 *   差が吸引のPWMの位置で変わる → PWMの電流の揺れを拾っている
 */

#include "SensorNoise.h"
#include "main.h"
#include "motor.h"
#include <stdio.h>
#include <math.h>

#define NOISE_N 500			//1つの組み合わせで読む回数(1msごと)
#define NOISE_TRACE 100		//そのうち、生の値も出す回数
#define NOISE_WAIT_MS 500	//吸引を変えてから測り始めるまで
#define NOISE_SETTLE_US 30	//待ちを入れるときの待ち時間[us]
#define NOISE_NC 4			//読み方の数
#define NOISE_ND 4			//吸引の数

static const uint8_t noise_duty[NOISE_ND] = { 0, 35, 50, 70 };
static const uint16_t noise_settle[NOISE_NC] = { 0, NOISE_SETTLE_US, 0, NOISE_SETTLE_US };
static const uint8_t noise_smp[NOISE_NC] = { 0, 0, 3, 3 };	//ADC_SAMPLETIME_3CYCLES、ADC_SAMPLETIME_56CYCLES
static const uint8_t noise_smp_cyc[NOISE_NC] = { 3, 3, 56, 56 };
static const char *noise_ch[4] = { "FL", "R", "L", "FR" };	//g_sensor と同じ並び
static const char *noise_kind[3] = { "on", "off", "diff" };

extern volatile uint16_t g_sensor_settle_us;
extern volatile int8_t g_sensor_smp_req;

/* 測っているあいだの集計(割り込みで足す) */
typedef struct {
	int32_t sum;
	uint64_t sq;	//diff は負にもなるので、符号は sum のほうで扱い、sq は2乗の和
	int16_t min, max;
} NoiseAcc;

static volatile int noise_rec = 0;	//1 のあいだ SensorNoise_Sample が記録する
static volatile int noise_n = 0;
static NoiseAcc noise_acc[4][3];	//[センサ][on/off/diff]
static NoiseAcc noise_batt;
static int16_t noise_tr[NOISE_TRACE][8];	//on 4つ、off 4つ
static uint8_t noise_tr_ph[NOISE_TRACE][3];

/* 結果(出力モードでもう一度出す) */
typedef struct {
	float mean, sd;
	int16_t min, max;
} NoiseStat;
static NoiseStat noise_res[NOISE_NC][NOISE_ND][4][3];
static NoiseStat noise_res_batt[NOISE_NC][NOISE_ND];
static int noise_done = 0;	//測り終えた組み合わせの数

static void acc_clear(NoiseAcc *a) {
	a->sum = 0;
	a->sq = 0;
	a->min = 32767;
	a->max = -32768;
}

static void acc_add(NoiseAcc *a, int v) {
	a->sum += v;
	a->sq += (uint64_t) ((int64_t) v * v);
	if (v < a->min) {
		a->min = v;
	}
	if (v > a->max) {
		a->max = v;
	}
}

static NoiseStat acc_stat(const NoiseAcc *a, int n) {
	NoiseStat s;
	double mean = (double) a->sum / n;
	double var = n > 1 ? ((double) a->sq - mean * mean * n) / (n - 1) : 0.0;
	s.mean = (float) mean;
	s.sd = var > 0 ? (float) sqrt(var) : 0.0f;
	s.min = a->min;
	s.max = a->max;
	return s;
}

void SensorNoise_Sample(const uint16_t on[4], const uint16_t off[4], uint16_t batt, const uint8_t ph[3]) {
	if (!noise_rec) {
		return;
	}
	int k = noise_n;
	for (int i = 0; i < 4; i++) {
		acc_add(&noise_acc[i][0], on[i]);
		acc_add(&noise_acc[i][1], off[i]);
		acc_add(&noise_acc[i][2], (int) on[i] - (int) off[i]);
	}
	acc_add(&noise_batt, batt);
	if (k < NOISE_TRACE) {
		for (int i = 0; i < 4; i++) {
			noise_tr[k][i] = on[i];
			noise_tr[k][4 + i] = off[i];
		}
		for (int j = 0; j < 3; j++) {
			noise_tr_ph[k][j] = ph[j];
		}
	}
	noise_n = k + 1;
	if (noise_n >= NOISE_N) {
		noise_rec = 0;
	}
}

static void print_header(int trace) {
	printf("\r\n#BEGIN NOISE n=%d trace=%d settle_us=%d wait_ms=%d pwm_top=%lu\r\n", NOISE_N, trace,
	NOISE_SETTLE_US, NOISE_WAIT_MS, (unsigned long) TIM2->ARR);
}

static void print_stat(int c, int d) {
	for (int i = 0; i < 4; i++) {
		for (int k = 0; k < 3; k++) {
			const NoiseStat *s = &noise_res[c][d][i][k];
			printf("stat,%d,%d,%d,%d,%s,%s,%.1f,%.2f,%d,%d\r\n", c, noise_settle[c], noise_smp_cyc[c],
					noise_duty[d], noise_ch[i], noise_kind[k], s->mean, s->sd, s->min, s->max);
		}
	}
	const NoiseStat *b = &noise_res_batt[c][d];
	printf("batt,%d,%d,%d,%d,%.1f,%.2f,%d,%d\r\n", c, noise_settle[c], noise_smp_cyc[c], noise_duty[d], b->mean,
			b->sd, b->min, b->max);
}

/* 吸引の強さを duty にする(0 なら止める)。now は今の強さ */
static void set_duty(int now, int duty) {
	if (duty == now) {
		return;
	}
	if (duty == 0) {
		Suction_Stop();
	} else if (now == 0) {
		Suction_Start(duty);
	} else {
		Suction_change(duty);
	}
}

void SensorNoise_Run(void) {
	int duty_now = 0;
	noise_done = 0;
	HAL_Delay(1000);	//手を離すまで待つ
	print_header(NOISE_TRACE);
	for (int c = 0; c < NOISE_NC; c++) {
		g_sensor_settle_us = noise_settle[c];
		g_sensor_smp_req = noise_smp[c];
		HAL_Delay(5);
		for (int d = 0; d < NOISE_ND; d++) {
			set_duty(duty_now, noise_duty[d]);
			duty_now = noise_duty[d];
			HAL_Delay(NOISE_WAIT_MS);

			for (int i = 0; i < 4; i++) {
				for (int k = 0; k < 3; k++) {
					acc_clear(&noise_acc[i][k]);
				}
			}
			acc_clear(&noise_batt);
			noise_n = 0;
			noise_rec = 1;
			while (noise_rec) {
				HAL_Delay(1);
			}

			for (int i = 0; i < 4; i++) {
				for (int k = 0; k < 3; k++) {
					noise_res[c][d][i][k] = acc_stat(&noise_acc[i][k], NOISE_N);
				}
			}
			noise_res_batt[c][d] = acc_stat(&noise_batt, NOISE_N);
			noise_done = c * NOISE_ND + d + 1;

			/* 出しているあいだも吸引は回したまま(次の組み合わせの待ちの前に出す) */
			print_stat(c, d);
			for (int k = 0; k < NOISE_TRACE; k++) {
				const int16_t *t = noise_tr[k];
				printf("tr,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", c, noise_duty[d], k, t[0], t[1], t[2],
						t[3], t[4], t[5], t[6], t[7], noise_tr_ph[k][0], noise_tr_ph[k][1], noise_tr_ph[k][2]);
			}
		}
	}
	Suction_Stop();
	g_sensor_settle_us = 0;
	g_sensor_smp_req = 0;	//もとの3サイクルに戻す
	HAL_Delay(5);
	printf("#END NOISE\r\n");
}

int SensorNoise_Has_Data(void) {
	return noise_done > 0;
}

void SensorNoise_Print(void) {
	print_header(0);
	for (int c = 0; c < NOISE_NC; c++) {
		for (int d = 0; d < NOISE_ND; d++) {
			if (c * NOISE_ND + d < noise_done) {
				print_stat(c, d);
			}
		}
	}
	printf("#END NOISE\r\n");
}
