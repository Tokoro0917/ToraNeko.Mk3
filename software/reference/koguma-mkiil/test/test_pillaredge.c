/* 柱の端の検出(PillarEdge)のPC上での検証。
 * 横センサが柱・壁を見るときの値を作り、ファームウェアと同じ流れ(1msごと、LPF 0.85/0.15、
 * 3回の中央値、遅れの分だけ距離を戻す)で PillarEdge_Update() に入れて、
 * 端の位置が速度・信号の大きさ・ノイズで変わらないかを確かめる。
 *
 * センサのモデル: 区画中心で柱の真ん中(前に90mm、横に84mm)を見る向き(tanθ = 90/84)。
 * 壁・柱の面を反射率1、ないところを0とし、光の広がりを幅 BEAM_SIGMA のぼかしで表す。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -I Core/Inc -o /tmp/tp test/test_pillaredge.c Core/Src/PillarEdge.c Core/Src/DistHist.c -lm && /tmp/tp
 */
#include <stdio.h>
#include <math.h>
#include "PillarEdge.h"

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

#define TAN 1.0714f		//90 / 84
#define BEAM_SIGMA 5.0f	//光の広がり[mm]
#define WALL_VALUE 240.0f	//区画中心で壁を見たときの値
#define LAG_S 0.0012f	//中央値(約1ms)とLPF(約0.2ms)の遅れ。ファームウェアと同じ値で戻す
#define BORDER 600.0f	//柱の中心(区画の境目)の位置[mm、壁の線に沿って]

/* 壁の並び。wall_a: 境目の手前に壁があるか、wall_b: 先に壁があるか */
static int wall_a, wall_b;
static int dropout_at = -1;	//この番号のサンプルだけ値を3割にする(1回だけの落ち込み)

/* 壁の線上の位置 u の反射率 */
static float Refl(float u) {
	if (u >= BORDER - 6 && u <= BORDER + 6) return 1.0f;	//柱
	if (u < BORDER - 6) return wall_a ? 1.0f : 0.0f;
	return wall_b ? 1.0f : 0.0f;
}

/* 光の広がりをぼかしで表した見え方 */
static float Seen(float u) {
	float sum = 0, wsum = 0;
	for (float k = -3 * BEAM_SIGMA; k <= 3 * BEAM_SIGMA; k += 0.25f) {
		float w = expf(-0.5f * k * k / (BEAM_SIGMA * BEAM_SIGMA));
		sum += w * Refl(u + k);
		wsum += w;
	}
	return sum / wsum;
}

static float Median3(float a, float b, float c) {
	if (a > b) { float t = a; a = b; b = t; }
	if (b > c) b = c;
	return (a > b) ? a : b;
}

typedef struct {
	float x_av80;	//従来の判定(5回の移動平均 < 80)で反応した機体の位置。比べるためだけ
	int n;			//見つけた端の数
	PillarEdgeEvent ev[4];
	float est[4];	//境目の推定(端の位置 + 横のずれの補正)。真の値との差を見る
} Result;

/* 速度 v[mm/s]、信号の大きさ gain、横のずれ y[mm、壁に近づく向きが正]で走らせる */
static Result Run(float v, float gain, float y) {
	PillarEdge e;
	PillarEdge_Reset(&e);
	Result r = { 0 };
	float d = 84.0f - y;	//壁までの距離
	float lpf = 0, m1 = 0, m2 = 0;
	float hist[5] = { 0 };
	int av_armed = 0;
	r.x_av80 = -1;
	float s0 = BORDER - 250.0f;
	for (int t = 0;; t++) {
		float s = s0 + v * t * 0.001f;	//機体の中心の位置
		if (s > BORDER + 80.0f) break;
		float raw = gain * WALL_VALUE * Seen(s + d * TAN);
		if (t == dropout_at) raw *= 0.3f;
		lpf = 0.85f * raw + 0.15f * lpf;
		/* 従来: 5回の移動平均。判定するのは次の1msの割り込みなので、そのときの位置で数える */
		for (int k = 4; k > 0; k--) hist[k] = hist[k - 1];
		hist[0] = lpf;
		float av = (hist[0] + hist[1] + hist[2] + hist[3] + hist[4]) / 5;
		if (av > 100) av_armed = 1;
		if (av_armed && av < 80 && r.x_av80 < 0) r.x_av80 = s + v * 0.001f;
		float med = Median3(lpf, m1, m2);
		m2 = m1;
		m1 = lpf;
		float dist = (med > 0.5f * gain * WALL_VALUE) ? d : 200.0f;
		if (PillarEdge_Update(&e, s - v * LAG_S, med, dist)) {
			if (r.n < 4) {
				r.ev[r.n] = e.ev;
				float corr = (e.ev.d_before >= 0) ? (e.ev.d_before - 84.0f) * TAN : 0;
				r.est[r.n] = e.ev.x + corr;
				r.n++;
			}
		}
	}
	return r;
}

static const float speeds[] = { 300, 500, 1000, 1500, 2000, 2400, 3000 };
static const float gains[] = { 0.6f, 0.8f, 1.0f, 1.2f, 1.4f };
#define NS (int)(sizeof(speeds) / sizeof(speeds[0]))
#define NG (int)(sizeof(gains) / sizeof(gains[0]))

/* 速度と信号の大きさを変えて、下がる端が1つだけ出て、位置のばらつきが tol 以内か */
static float Check_Spread(const char *name, int expect_kind, float tol) {
	float lo = 1e9f, hi = -1e9f, sum = 0;
	int cnt = 0;
	for (int i = 0; i < NS; i++) {
		for (int g = 0; g < NG; g++) {
			Result r = Run(speeds[i], gains[g], 0);
			CHECK(r.n == 1, "%s v=%.0f gain=%.1f: 端が %d 個(1個のはず)", name, speeds[i], gains[g], r.n);
			if (r.n < 1) continue;
			CHECK(r.ev[0].kind == expect_kind, "%s v=%.0f gain=%.1f: 種類 %d", name, speeds[i], gains[g],
					r.ev[0].kind);
			float x = r.est[0];
			if (x < lo) lo = x;
			if (x > hi) hi = x;
			sum += x;
			cnt++;
		}
	}
	float mean = cnt ? sum / cnt : 0;
	printf("  %-22s 境目からの位置 平均 %+6.2f mm  ばらつき %.2f mm(速度300〜3000、大きさ0.6〜1.4倍)\n",
			name, mean - BORDER, hi - lo);
	CHECK(hi - lo <= tol, "%s: ばらつき %.2f mm が %.1f mm を超えた", name, hi - lo, tol);
	return mean;
}

/* 従来の判定(閾値80)の位置のばらつき(比べるためだけ。確かめはしない) */
static void Print_Av80(const char *name) {
	float lo = 1e9f, hi = -1e9f, slo = 1e9f, shi = -1e9f, glo = 1e9f, ghi = -1e9f;
	for (int i = 0; i < NS; i++) {
		for (int g = 0; g < NG; g++) {
			float x = Run(speeds[i], gains[g], 0).x_av80;
			if (x < 0) continue;
			if (x < lo) lo = x;
			if (x > hi) hi = x;
			if (gains[g] == 1.0f) {
				if (x < slo) slo = x;
				if (x > shi) shi = x;
			}
			if (speeds[i] == 1000) {
				if (x < glo) glo = x;
				if (x > ghi) ghi = x;
			}
		}
	}
	printf("  %-22s 従来(移動平均 < 80): ばらつき %.2f mm(速度だけ %.2f mm、大きさだけ %.2f mm)\n", name,
			hi - lo, shi - slo, ghi - glo);
}

int main(void) {
	printf("下がる端の位置:\n");

	/* #2 壁の終わり */
	wall_a = 1;
	wall_b = 0;
	float m2 = Check_Spread("#2 壁 -> なし", EDGE_KIND_WALL, 1.0f);
	Print_Av80("#2 壁 -> なし");

	/* #3 柱だけの山。下がる端が1回だけ */
	wall_a = 0;
	wall_b = 0;
	float m3 = Check_Spread("#3 なし -> なし(柱)", EDGE_KIND_PILLAR, 1.0f);
	printf("  #2 と #3 の差 %.2f mm(実機では種類ごとにずれを持つ)\n", m3 - m2);

	/* #1 壁が続く: 何も出ない */
	wall_a = 1;
	wall_b = 1;
	for (int i = 0; i < NS; i++) {
		Result r = Run(speeds[i], 1.0f, 0);
		CHECK(r.n == 0, "#1 v=%.0f: 壁が続くのに端が %d 個", speeds[i], r.n);
	}

	/* #4 なし -> 壁: 下がる端は出ない */
	wall_a = 0;
	wall_b = 1;
	for (int i = 0; i < NS; i++) {
		Result r = Run(speeds[i], 1.0f, 0);
		CHECK(r.n == 0, "#4 v=%.0f: 上がるだけなのに下がる端が %d 個", speeds[i], r.n);
	}

	/* 壁の途中の1回だけの落ち込みでは出ない(中央値で消える) */
	wall_a = 1;
	wall_b = 1;
	for (int k = 50; k < 200; k += 7) {
		dropout_at = k;
		Result r = Run(1000, 1.0f, 0);
		CHECK(r.n == 0, "落ち込み(%d番目): 端が %d 個", k, r.n);
	}
	dropout_at = -1;

	/* 横のずれ: 片側だけでも、壁までの距離で直せば境目の推定が変わらない */
	wall_a = 1;
	wall_b = 0;
	{
		Result r0 = Run(1000, 1.0f, 0);
		const float ys[] = { -8, -5, -2, 2, 5, 8 };
		printf("横のずれ(#2、1000mm/s):\n");
		for (unsigned k = 0; k < sizeof(ys) / sizeof(ys[0]); k++) {
			Result r = Run(1000, 1.0f, ys[k]);
			CHECK(r.n == 1 && r0.n == 1, "y=%.0f: 端が %d 個", ys[k], r.n);
			if (r.n < 1 || r0.n < 1) continue;
			printf("  y=%+4.0f  補正なし %+6.2f mm  補正あり %+6.2f mm\n", ys[k], r.ev[0].x - r0.ev[0].x,
					r.est[0] - r0.est[0]);
			CHECK(fabsf(r.est[0] - r0.est[0]) <= 1.0f, "y=%.0f: 補正しても %.2f mm ずれる", ys[k],
					r.est[0] - r0.est[0]);
		}
	}

	/* 左右をまとめる */
	{
		float c;
		CHECK(PillarEdge_Fuse(0, 0, 0, 0, 0, 0, 15, &c) == 0, "どちらもないのに推定が出た");
		CHECK(PillarEdge_Fuse(1, 100, 200, 0, 0, 0, 15, &c) == EDGE_SRC_R && c == 100, "右だけ");
		CHECK(PillarEdge_Fuse(0, 0, 0, 1, 104, 150, 15, &c) == EDGE_SRC_L && c == 104, "左だけ");
		CHECK(PillarEdge_Fuse(1, 100, 200, 1, 106, 150, 15, &c) == (EDGE_SRC_R | EDGE_SRC_L)
				&& fabsf(c - 103) < 1e-4f, "両方の平均 %.2f", c);
		CHECK(PillarEdge_Fuse(1, 100, 120, 1, 130, 150, 15, &c) == (EDGE_SRC_L | EDGE_SRC_DISAGREE)
				&& c == 130, "離れていたら大きいほう");
		/* 向きのずれ: 片側ずつでは逆向きにずれ、平均すると打ち消される */
		float shift = 3.1f;	//1°で約3.1mm
		CHECK(PillarEdge_Fuse(1, 500 + shift, 200, 1, 500 - shift, 200, 15, &c) && fabsf(c - 500) < 1e-4f,
				"向きのずれが打ち消されない %.2f", c);
	}

	if (fail_count) {
		printf("NG: %d 件\n", fail_count);
		return 1;
	}
	printf("OK\n");
	return 0;
}
