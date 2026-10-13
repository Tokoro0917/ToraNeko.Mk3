/* 距離ごとのセンサ値の履歴(DistHist)のPC上での検証。
 * 柱の壁切れ(Wallsensor.c の Pillar_Edge)が「Pillar_Len 進む間の変化量」を
 * 速度に関係なく同じに見られるかを確かめる。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -I Core/Inc -o /tmp/td test/test_disthist.c Core/Src/DistHist.c -lm && /tmp/td
 */
#include <stdio.h>
#include <math.h>
#include "DistHist.h"

static int fail_count = 0;
#define CHECK(cond, ...)                                                      \
	do {                                                                      \
		if (!(cond)) {                                                        \
			fail_count++;                                                     \
			printf("  FAIL: ");                                               \
			printf(__VA_ARGS__);                                              \
			printf("\n");                                                     \
		}                                                                     \
	} while (0)

/* 柱の後ろの端(x = 100mm)で、センサ値が約25mmかけて 300 -> 60 に下がる形
 * (いちばん急なところで 5mm あたり約60) */
static float Edge_Value(float x) {
	return 60.0f + 240.0f / (1.0f + expf((x - 100.0f) / 5.0f));
}

/* 1msごとに速度 v[mm/s] で走り、「今の値 - len mm 前の値」が th 以上減った最初の距離を返す。
 * Wallsensor.c の新しい Pillar_Edge と同じ判定 */
static float Detect_Dist(float v, float len, float th) {
	DistHist h;
	DistHist_Reset(&h);
	for (int t = 0; t < 100000; t++) {
		float x = 40.0f + v * t * 0.001f;
		float now = Edge_Value(x);
		DistHist_Push(&h, x, now);
		float past;
		if (DistHist_At(&h, x - len, &past) && (past - now) >= th) {
			return x;
		}
		if (x > 200.0f) break;
	}
	return -1;
}

/* 以前の Pillar_Edge と同じ判定(len mm を目標速度から何ms前かに直し、1〜6msに丸める) */
static float Detect_Time(float v, float len, float th) {
	float hist[64];
	int k = (int) (len * 1000 / v + 0.5f);
	if (k < 1) k = 1;
	if (k > 6) k = 6;
	for (int t = 0; t < 100000; t++) {
		float x = 40.0f + v * t * 0.001f;
		hist[t % 64] = Edge_Value(x);
		if (t >= k && (hist[(t - k) % 64] - hist[t % 64]) >= th) {
			return x;
		}
		if (x > 200.0f) break;
	}
	return -1;
}

int main(void) {
	DistHist h;
	float out;

	/* 1. 直線の信号なら、どの速度でも5mm前の値が正確に取れる(変化量が速度で変わらない) */
	{
		const float speeds[] = { 300, 500, 1000, 1500, 2000, 2400, 3000, 4000, 6000 };
		for (unsigned s = 0; s < sizeof(speeds) / sizeof(speeds[0]); s++) {
			DistHist_Reset(&h);
			float v = speeds[s];
			for (int t = 0; t < 200; t++) {
				float x = 13.7f + v * t * 0.001f;
				DistHist_Push(&h, x, 1000.0f - 3.0f * x);
				if (x - 13.7f < 10.0f) continue;
				CHECK(DistHist_At(&h, x - 5.0f, &out), "v=%.0f: 5mm前が取れない", v);
				float drop = out - (1000.0f - 3.0f * x);
				CHECK(fabsf(drop - 15.0f) < 0.01f,
						"v=%.0f x=%.1f: 5mm進む間の変化量 %.3f (15 のはず)", v, x, drop);
			}
		}
	}

	/* 2. 柱の形の信号で、反応する位置が速度で変わらない(1msで進む距離以内) */
	{
		const float speeds[] = { 500, 1000, 2000, 2400, 3000 };
		float ref = Detect_Dist(500, 5.0f, 40);
		printf("柱の反応位置(Pillar_Len 5mm、閾値40):\n");
		printf("  速度   新(距離ごと)   以前(時間ごと)\n");
		for (unsigned s = 0; s < sizeof(speeds) / sizeof(speeds[0]); s++) {
			float v = speeds[s];
			float xd = Detect_Dist(v, 5.0f, 40);
			float xt = Detect_Time(v, 5.0f, 40);
			if (xt > 0) {
				printf("  %5.0f   %6.1f mm      %6.1f mm\n", v, xd, xt);
			} else {
				printf("  %5.0f   %6.1f mm      反応しない\n", v, xd);
			}
			CHECK(xd > 0, "v=%.0f: 反応しない", v);
			CHECK(fabsf(xd - ref) <= v * 0.001f + 0.5f,
					"v=%.0f: 反応位置 %.1f が 500 のとき(%.1f)と 1msで進む距離より離れている", v, xd, ref);
		}
	}

	/* 3. 範囲外は取れない */
	DistHist_Reset(&h);
	CHECK(!DistHist_At(&h, 0, &out), "空なのに取れた");
	for (int i = 0; i <= 100; i++) {
		DistHist_Push(&h, i * 1.0f, (float) i);
	}
	CHECK(!DistHist_At(&h, 100.5f, &out), "最後のサンプルより先が取れた");
	CHECK(DistHist_At(&h, 100.0f - (DISTHIST_N - 1) * DISTHIST_STEP_MM, &out),
			"いちばん古い点が取れない");
	CHECK(!DistHist_At(&h, 100.0f - DISTHIST_N * DISTHIST_STEP_MM, &out),
			"残している範囲より前が取れた");
	CHECK(DistHist_At(&h, 97.25f, &out) && fabsf(out - 97.25f) < 1e-3f,
			"点の間の補間 %.3f (97.25 のはず)", out);

	/* 4. 止まっているあいだは距離が進まず、値だけ入れ替わる */
	DistHist_Reset(&h);
	DistHist_Push(&h, 10.0f, 100);
	DistHist_Push(&h, 12.0f, 120);
	for (int i = 0; i < 50; i++) {
		DistHist_Push(&h, 12.0f, 500);
	}
	DistHist_Push(&h, 11.9f, 130);	//わずかに下がった
	CHECK(fabsf(DistHist_LastX(&h) - 12.0f) < 1e-6f, "止まっているのに距離が進んだ");
	CHECK(DistHist_At(&h, 11.0f, &out) && fabsf(out - 110.0f) < 1e-3f,
			"止まったあとに過去の点が変わった (%.3f)", out);
	CHECK(DistHist_At(&h, 12.0f, &out) && fabsf(out - 130.0f) < 1e-3f,
			"最後の値が入れ替わっていない (%.3f)", out);

	/* 5. 大きく飛んだときも、残せる分だけ埋めて壊れない */
	DistHist_Reset(&h);
	DistHist_Push(&h, 0.0f, 0);
	DistHist_Push(&h, 1000.0f, 1000);
	CHECK(h.count == DISTHIST_N, "点の数 %d", h.count);
	CHECK(DistHist_At(&h, 990.0f, &out) && fabsf(out - 990.0f) < 1e-2f, "飛んだあとの補間 %.3f", out);

	if (fail_count) {
		printf("NG: %d 件\n", fail_count);
		return 1;
	}
	printf("OK\n");
	return 0;
}
