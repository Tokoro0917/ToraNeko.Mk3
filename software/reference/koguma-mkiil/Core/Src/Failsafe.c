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
#include "main.h"
#include "UI.h"

volatile int flag = 0;	//割り込み(フェイルセーフの判定)で立て、走りのループで読む
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

/* スイッチ(Switch_Pressed)を 10ms ごとに読み、2回続けて want になるまで待つ(チャタリングを取り除く)。
 * 待っているあいだは、0.5秒ごとに LED を全部点滅させる */
static void Failsafe_Wait_Switch(int want, uint32_t *tick) {
	int same = 0;
	while (same < 2) {
		same = (Switch_Pressed() == want) ? (same + 1) : 0;
		if ((*tick / 50) % 2 == 0) {
			LED_ALL_ON();
		} else {
			LED_Reset();
		}
		HAL_Delay(10);
		*tick = *tick + 1;
	}
}

/* フェイルセーフで止まったあと、走りを抜けてから main で呼ぶ(2026-10-27、ToraNeko.Mk3 と同じ戻り方)。
 * モーターと吸引を止めたまま LED を全部点滅させ、スイッチ(S2)を押したら、
 * 「ロード中」を出してフェイルセーフを解除し、モード0・No.0 に戻す。迷路は RAM に残っている
 * (探索なら、最後に壁を書いた5区画を戻して保存してある。main.c の Maze_Search)。
 * スイッチを押したまま止まったときに解除しないよう、まず離れているのを見てから押されるのを待つ。
 * ロード中のあいだはスイッチを押しても何も起きない */
void Failsafe_Wait_Reset() {
	uint32_t tick = 0;
	Motor_Free();
	Failsafe_Wait_Switch(0, &tick);
	Failsafe_Wait_Switch(1, &tick);
	LED_Loading();
	Failsafe_Flag_OFF();
	Encorder_count_reset();
}
