/*
 * SpeedPlan.h
 *
 * 最短走行の速度計画。ターンごとに通過速度を変えたときに、
 * 直線区間の始点・終点速度と、ターンの通過速度を決める。
 * HALに依存しないので、PC上でテストできる(test/test_speedplan.c)。
 */

#ifndef INC_SPEEDPLAN_H_
#define INC_SPEEDPLAN_H_

#include <stdint.h>

/* G_Short_Pass_NANAME[] の1要素の種類 */
#define SP_SKIP      0	/* -1などの穴埋め。走行ループでも何もしない */
#define SP_STRAIGHT  1	/* 区間前進(1以上、半区画=90mm単位) */
#define SP_DIAGONAL  2	/* 斜め直線(-50の倍数、127.3mm単位) */
#define SP_TURN      3	/* 大回り・斜め入り/出・V90 */

int SpeedPlan_Kind(int16_t code);

#define SPEEDPLAN_MAX_ELEM 256	/* 圧縮後のパスの要素数の上限 */
#define SPEEDPLAN_BACK_MARGIN 0.8f	/* 後距離のうち速度の変化に使ってよい割合(壁切れで早く終わる分の余裕) */

typedef struct {
	/* 入力 */
	float (*turn_v)(int16_t code);	/* ターンの希望通過速度[mm/s] */
	float v_max;		/* 直線の最高速度(= MAX)。ターン速度もこれを超えない */
	float ac;			/* 直線の加速度(= AC) */
	float v_start;		/* スタート区間の終わりの速度の上限 */
	float v_goal;		/* ゴール停止区間に入る速度の上限 */
	float ac_diag;		/* 斜め直線の加速度。0以下なら ac - 5000 */
	/* ターンが直線を挟まずにつながるときの速度の変え方。
	 * turn_back が NULL か ac_back が0以下なら、前後のターンは遅い方にそろう。
	 * そうでなければ、前のターンの後距離(turn_back の値)のうち SPEEDPLAN_BACK_MARGIN の割合で、
	 * 加速度 ac_back で次のターンの速度へ変える(変えきれない分だけ速い方を下げる) */
	float (*turn_back)(int16_t code);	/* ターンの後距離[mm] */
	float ac_back;		/* 後距離で速度を変えるときの加速度 */
	/* 出力(要素ごと)。ターンは v_in が通過速度で、v_out は後距離の終わりの速度
	 * (次がターンに直結していれば次のターンの速度、そうでなければ v_in と同じ) */
	float v_in[SPEEDPLAN_MAX_ELEM];
	float v_out[SPEEDPLAN_MAX_ELEM];
	float v_start_out;	/* スタート区間の終わりの速度 */
	float v_goal_in;	/* ゴール停止区間に入る速度 */
} SpeedPlan_t;

/* pass[] を 0 終端まで見て速度を決める。要素数が上限を超えたら -1 を返す */
int SpeedPlan_Make(const int16_t *pass, SpeedPlan_t *plan);

#endif /* INC_SPEEDPLAN_H_ */
