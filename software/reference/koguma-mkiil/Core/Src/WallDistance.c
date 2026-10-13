/*
 * WallDistance.c
 *
 *  壁センサ値 -> 距離[mm] 変換 (表引き + 直線補間)
 *
 *  横(L/R)は実測値(2026-10-06、モード3 No.4)。区画の内のり168mm(区画18cm、
 *  壁の厚さ1.2cm)、機体の幅70mm(中心線から側面まで35mm)として、機体の側面を
 *  反対側の壁につけ、壁の板(12mm)を n 枚はさんで、測る側の壁までを
 *  133 - 12n mm(n = 0〜8)にして測った。
 *
 *  前(FL/FR)も実測値(2026-10-06)。全長120mm(中心からお尻まで60mm)として、
 *  お尻を後ろの壁につけ、壁の板を n 枚はさんで、前の壁までを 108 - 12n mm
 *  (n = -8〜4、60〜204mm)にして測った(n が負は、お尻を後ろの壁から離した)。
 *  60mmより近い点はまだ測っていない(表の範囲外は端の値になる)。
 *
 *  表の書き方:
 *    - 距離は小さい順(近い順)に並べる
 *    - センサ値は距離が増えるほど小さくなること(単調減少)
 *    - 点の数はセンサごとに変えてよい(最大 WALLDIST_TABLE_MAX)
 */

#include "WallDistance.h"

#define WALLDIST_TABLE_MAX 16

typedef struct {
	int n;
	float mm[WALLDIST_TABLE_MAX];
	int v[WALLDIST_TABLE_MAX];
} WallDistTable;

static const WallDistTable wall_dist_table[4] = {
	/* FL (実測) */
	{ 13,
	  { 60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180, 192, 204 },
	  { 1366, 902, 645, 470, 356, 295, 230, 195, 167, 146, 132, 123, 112 } },
	/* R (実測) */
	{ 9,
	  { 37, 49, 61, 73, 85, 97, 109, 121, 133 },
	  { 780, 522, 356, 262, 201, 160, 131, 110, 93 } },
	/* L (実測) */
	{ 9,
	  { 37, 49, 61, 73, 85, 97, 109, 121, 133 },
	  { 910, 603, 415, 308, 237, 183, 150, 125, 105 } },
	/* FR (実測) */
	{ 13,
	  { 60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180, 192, 204 },
	  { 1377, 940, 627, 461, 358, 281, 248, 204, 175, 155, 129, 120, 112 } },
};

/* 会場ごとの倍率。モード3 No.5 の出力の WallDist_Scale の行を書き写す。
 * 並びは FL, R, L, FR (g_sensor と同じ) */
/* 2026-10-10: 区画中心(前壁なし)のモード3 No.4 で L=242, R=198 → L 82.0mm, R 89.5mm だったので
 * L・R を84mmになるよう合わせた(FL・FR は前壁がなく測れないのでそのまま) */
float WallDist_Scale[4] = { 0.99f, 0.96f, 1.00f, 1.02f };

float WallDist_Value(int sensor, float mm) {
	const WallDistTable *t = &wall_dist_table[sensor];

	if (mm <= t->mm[0]) {
		return t->v[0];
	}
	for (int i = 0; i < t->n - 1; i++) {
		if (mm <= t->mm[i + 1]) {
			float r = (mm - t->mm[i]) / (t->mm[i + 1] - t->mm[i]);
			return t->v[i] + r * (t->v[i + 1] - t->v[i]);
		}
	}
	return t->v[t->n - 1];
}

float WallDist_mm(int sensor, int v_raw) {
	const WallDistTable *t = &wall_dist_table[sensor];
	float s = WallDist_Scale[sensor];
	int v = (s > 0.0f) ? (int) (v_raw / s + 0.5f) : v_raw;

	if (v >= t->v[0]) {
		return t->mm[0];
	}
	for (int i = 0; i < t->n - 1; i++) {
		if (v >= t->v[i + 1]) {
			float r = (float) (v - t->v[i + 1]) / (float) (t->v[i] - t->v[i + 1]);
			return t->mm[i + 1] + r * (t->mm[i] - t->mm[i + 1]);
		}
	}
	return t->mm[t->n - 1];
}
