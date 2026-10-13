/*
 * Wallsensor.c
 *
 *  Created on: Jun 21, 2024
 *      Author: akihi
 */

#include "Wallsensor.h"
#include "PL_sensor.h"
#include "UI.h"
#include "gpio.h"
#include "motor.h"
#include "math.h"
#include "WallDistance.h"
#include "PillarEdge.h"
#include "lsm6dsr.h"
#include "PL_encoder.h"
#include <stdio.h>

int G_Wall_data[] = { 0, 0, 0, 0 };
int Wall_threshold[] = { 60, 90, 90, 60 }; //80, 150, 150, 80

float Kp = 0.05; //0.025
float Kd = 0.01;

/* 横壁制御を距離[mm]で行うか (0: 従来のセンサ値, 1: 距離)
 * 2026-10-07 から 1 が標準(直進テストのログで、センサ値版より角速度の振れが小さかった)。
 * センサ値版に戻すときは 0 にする(モード4 No.8 はセンサ値版で走る) */
int G_WallCtrl_Use_mm = 1;
/* 区画中心付近で従来のKp, Kdと同じ効きになる値から始める
 * (仮の表では84mm付近の傾きが約5.2[値/mm] -> 0.05*5.2, 0.01*5.2)。
 * 2026-10-09: 壁に寄ったときの戻りが弱い(センサ値版は寄るほど強く効いていた)ので Kp_mm を 0.26 -> 0.35 */
float Kp_mm = 0.30;	//2026-10-10 0.35 -> 0.30(センサ値版の Kp=0.05 と区画中心付近で同じ効き。中心付近は約6[値/mm])
float Kd_mm = 0.06;	//2026-10-10 0.07 -> 0.06(センサ値版の Kd=0.01 と同じ効き)。2026-10-09 0.05 -> 0.07(横壁制御で左右に揺れながら走り、ターンに入るときの向きがずれるのを抑える)

float Kp_Na = 0.1;
float Kd_Na = 0.00;

float Kp_FW = 0.1;
float Kd_FW = 0.0;

float Kp_FW_dire = 0.1;
float Kd_FW_dire = 0.0;

float KP, KD;

/* 区画中心(壁まで84mm)での横センサの値。2026-10-06の実測の表(WallDistance.c)から
 * L=243、R=206(以前は220/200で、片側の壁だけのときは左壁から約89mm/右壁から
 * 約85mm、両側の壁では中心から右へ約1.7mmずれた位置に合わせていた) */
int Wall_L = 243; //220 //370
int Wall_R = 206; //200 //250

int Wall_TH_L = 80;
int Wall_TH_R = 80;

/* 前壁合わせの目標(前センサFL/FRの平均)。区画中心(前の壁まで84mm)で
 * FL=645、FR=627(2026-10-06実測)の平均。以前の530は前の壁から約92mmで、
 * 区画中心より約8mm手前で止まっていた */
float FlontWall_Distance = 636; //530

int Sensor_diff_TH = 50;

/* 距離版(G_WallCtrl_Use_mm = 1)の横壁制御だけで使う値(2026-10-07、ログから決めた)。
 * 遠い壁や壁の切れ目では mm の誤差が大きくなり、センサ値版の約2倍のキックが出ていた */
float WallCtrl_mm_MaxDist = 120;	//これより遠い壁は使わない(区画中心から約±36mm)。104では片寄ったときに使う/使わないが切り替わり続けて中心に戻れなかった
float WallCtrl_mm_ErrMax = 20;	//片側の誤差の頭打ち[mm]
/* 壁の切れ目の判定(2026-10-10)。壁が見え始め・消えかけのときはセンサ値が20mmほどかけて
 * 変わり、その途中では壁を実際より遠いと読んで誤差が±20に張り付いていた。
 * 1msの変化で見ると速度で効き方が変わり、1000mm/sでは一度も効かなかったので、
 * WallCtrl_mm_StableLen 進む間の距離の変化が WallCtrl_mm_StableTH を超えた壁は使わない */
float WallCtrl_mm_StableLen = 10;	//何mm前の距離と比べるか
float WallCtrl_mm_StableTH = 15;	//この間に距離がこれ以上変わった壁は使わない[mm]。2026-10-10 8 -> 15(左センサのノイズで壁を使う/使わないが数msおきに切り替わっていた。ノイズが直れば8に戻す)
float WallCtrl_mm_GainMin = 0;	//距離版のゲイン倍率(速度/500)の下限。減速中に効かなくなって片寄っていたので0.5にしたが、2026-10-10 センサ値版と比べるため0(下限なし)

float Wall_error = 0;
float Wall_old_error = 0;
float Wall_Delta_error = 0;

float Wall_error_NA = 0;
float Wall_old_error_NA = 0;
float Wall_Delta_error_NA = 0;

float Wall_error_FW = 0;
float Wall_old_error_FW = 0;
float Wall_Delta_error_FW = 0;

void Wall_search() {
	for (int i = 0; i < 4; i++) {
		G_Wall_data[i] = 0;
	}

	if (g_sensor_av10[0] > Wall_threshold[0]) { //正面左//g_sensor_av
		G_Wall_data[0] = 1;
	}
	if (g_sensor_av10[2] > Wall_threshold[2]) { //左
		G_Wall_data[1] = 1;
	}

	if (g_sensor_av10[1] > Wall_threshold[1]) { //右
		G_Wall_data[2] = 1;
	}
	if (g_sensor_av10[3] > Wall_threshold[3]) { //正面右
		G_Wall_data[3] = 1;
	}

}

int Sensor_Enter() {
	int i = 0;
	if (g_sensor_av[0] > 1400) {
		i = 1;
	}
	return i;
}

int Sensor_Start() {
	while (1) {
		LED_StartWait();
		if ((g_sensor_av[1] + (g_sensor_av[2])) > 2000) {
			break;
		}
	}
	Buzzer_Start();

}

float Wall_Flont_Av() {
	float av = 0;
	av = (g_sensor[0][0] + g_sensor[3][0]) / 2;
	return av;
}

void Wall_search_LED() {
	if (G_Wall_data[0] == 1) {
		HAL_GPIO_WritePin(LEDF1_GPIO_Port, LEDF1_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF1_GPIO_Port, LEDF1_Pin, 0);
	}
	if (G_Wall_data[1] == 1) {
		HAL_GPIO_WritePin(LEDF3_GPIO_Port, LEDF3_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF3_GPIO_Port, LEDF3_Pin, 0);
	}
	if (G_Wall_data[2] == 1) {
		HAL_GPIO_WritePin(LEDF2_GPIO_Port, LEDF2_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF2_GPIO_Port, LEDF2_Pin, 0);
	}
	if (G_Wall_data[3] == 1) {
		HAL_GPIO_WritePin(LEDF4_GPIO_Port, LEDF4_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF4_GPIO_Port, LEDF4_Pin, 0);
	}

}

/* 距離キャリブレーション(モード3-4)
 * 機体を壁から既知の距離に置き、表示された値を WallDistance.c の表に書き写す。
 * 約0.2秒ごとに100回平均したセンサ値と、現在の表で変換した距離[mm]をCSVで出す。
 * 手で機体を動かすとモード選択が変わるが、このループの中にいる間は影響しない。
 * 抜けるときはリセット。 */
void Wall_Distance_Calibration() {
	printf("CAL,FL,L,R,FR,FL_mm,L_mm,R_mm,FR_mm\n\r");
	while (1) {
		long sum[4] = { 0, 0, 0, 0 };
		for (int k = 0; k < 100; k++) {
			for (int i = 0; i < 4; i++) {
				sum[i] += g_sensor[i][0];
			}
			HAL_Delay(2);
		}
		int v[4];
		for (int i = 0; i < 4; i++) {
			v[i] = sum[i] / 100;
		}
		printf("CAL,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f\n\r", v[0], v[2], v[1],
				v[3], WallDist_mm(WALLDIST_FL, v[0]),
				WallDist_mm(WALLDIST_L, v[2]), WallDist_mm(WALLDIST_R, v[1]),
				WallDist_mm(WALLDIST_FR, v[3]));
	}
}

/* 会場ごとの感度合わせ(モード3 No.5)
 * 機体を区画中心に置き、前・左・右に壁がある状態で測る
 * (スタート区画なら、後ろの壁のほうを向けて置く)。
 * 約1秒ごとに500回平均した値と、手で書き写す行をシリアルに出す。抜けるときはリセット。
 * 倍率 = 測った値 / 表の区画中心(84mm)の値。閾値は今の値に倍率を掛けたもの
 * (書き写す前の値、つまり倍率1のときの値が基準。2回目以降も元の値に掛ける) */
void Wall_Center_Calibration() {
	/* 倍率1のときの閾値(書き換えたら、ここも基準として残す) */
	const int th_base[4] = { 60, 90, 90, 60 };	//Wall_threshold
	const float align_base = 200;	//Move.c ALIGN_FRONT_MIN
	const float cut_base = 80;	//motor.c Cut_L / Cut_R
	const float cut_na_base = 150;	//motor.c Cut_L_NA / Cut_R_NA

	while (1) {
		long sum[4] = { 0, 0, 0, 0 };
		for (int k = 0; k < 500; k++) {
			for (int i = 0; i < 4; i++) {
				sum[i] += g_sensor[i][0];
			}
			HAL_Delay(2);
		}
		float v[4], ref[4], k[4];
		for (int i = 0; i < 4; i++) {
			v[i] = sum[i] / 500.0f;
			ref[i] = WallDist_Value(i, WALLDIST_CENTER_MM);
			k[i] = v[i] / ref[i];
		}
		float kf = (k[0] + k[3]) / 2;
		printf("\r\n#BEGIN CENTER\r\n");	//tools/viewer.html で読む
		printf("---- CENTER  FL=%.0f L=%.0f R=%.0f FR=%.0f  (table FL=%.0f L=%.0f R=%.0f FR=%.0f)\n\r",
				v[0], v[2], v[1], v[3], ref[0], ref[2], ref[1], ref[3]);
		printf("WallDistance.c: float WallDist_Scale[4] = { %.2ff, %.2ff, %.2ff, %.2ff };\n\r",
				k[0], k[1], k[2], k[3]);
		printf("Wallsensor.c:   int Wall_L = %.0f;  int Wall_R = %.0f;  float FlontWall_Distance = %.0f;\n\r",
				v[2], v[1], (v[0] + v[3]) / 2);
		printf("Wallsensor.c:   int Wall_threshold[] = { %.0f, %.0f, %.0f, %.0f };\n\r",
				th_base[0] * k[0], th_base[1] * k[1], th_base[2] * k[2],
				th_base[3] * k[3]);
		printf("Move.c:         #define ALIGN_FRONT_MIN %.0f\n\r", align_base * kf);
		printf("motor.c:        float Cut_R = %.0f;  float Cut_L = %.0f;  float Cut_R_NA = %.0f;  float Cut_L_NA = %.0f;\n\r",
				cut_base * k[1], cut_base * k[2], cut_na_base * k[1],
				cut_na_base * k[2]);
		printf("#END CENTER\r\n");
	}
}

/* 横壁制御の出力。Wall_Control_Update() が1msに1回計算し、
 * calWallConrol() はその値を返す */
static float G_Wall_PID = 0;
/* 走行ログ用: P項・D項、使った壁(1 左、2 右、3 両方、0 なし)、横の壁までの距離[mm](3回の中央値から) */
float G_Wall_P = 0, G_Wall_D = 0, G_Wall_dL = 0, G_Wall_dR = 0;
int G_Wall_St = 0;

/* 直進中の横壁制御の値を返す(1msの中で何回呼んでも同じ値)。
 * 計算は Motor_Speed_PID_ST() の中の Wall_Control_Update() で1回だけ行う */
float calWallConrol() {
	return G_Wall_PID;
}

/* 横センサの直近3回の中央値(2026-10-10、距離版の横壁制御用)。
 * 左センサだけに4msおきの1回だけの落ち込みがあり、そのたびに「壁が遠い」と読んで
 * 誤差が -20〜-33 に飛んでいたので、1回だけの飛びを消す。
 * g_sensor[i][1] は更新直後のずらしで [0] と同じ値なので、[0](今回)・[2](1ms前)・[3](2ms前)を使う */
static int Median3(int a, int b, int c) {
	if (a > b) { int t = a; a = b; b = t; }
	if (b > c) { b = c; }
	return (a > b) ? a : b;
}

static int Wall_Sensor_Median3(int i) {
	return Median3(g_sensor[i][0], g_sensor[i][2], g_sensor[i][3]);
}

/* 3回の中央値(壁切れを遅れの少ない値で見るとき用。motor.c の G_WallCut_Fast) */
int Wall_Sensor_Med3(int i) {
	return Wall_Sensor_Median3(i);
}

/* ==== 柱の端(2026-10-12、設計は docs/pillar_cut_design.md) ====
 * センサ値ができるたびに(DMA割り込みの中、pl_callback_getSensor から)Wall_Edge_Sample() を呼ぶ。
 * 左右それぞれ PillarEdge で下がる端を見つけ、境目の推定 c = 端の位置 + Edge_Off + 横のずれの補正
 * を記録する。走行の関数(motor.c)は Edge_Cut_Find() で、決まった範囲にある推定を取り出す。 */
float Edge_Lag_S = 0.0012f;	//中央値(約1ms)とLPF(約0.2ms)の遅れ[s]。この分だけ距離を戻して記録する
float Edge_Max_W = 150;		//角速度がこれを超えたら(ターン中など)履歴を空にする[deg/s]
/* 端の位置から境目までの距離[mm]。[0: 右 / 1: 左][EDGE_KIND_WALL / EDGE_KIND_PILLAR]。
 * センサが区画中心で柱の真ん中を見る向きなので、下がる端は境目の約84mm手前で見える(PCのモデルで約84)。
 * モード3 No.6〜9 の c-border が0になるように実測で直す */
float Edge_Off[2][2] = { { 84, 84 }, { 84, 84 } };
float Edge_Tan = 1.07f;		//横のずれ1mmで、見える位置が何mm変わるか(90/84。横にずらして置いて実測で直す)
float Edge_Pair_Tol = 15;	//左右の推定がこれ以内なら平均する[mm]

#define EDGE_Q 4	//左右それぞれ、最後の何個の推定を残すか
typedef struct {
	PillarEdgeEvent ev;
	float d_eff;	//横のずれの補正に使った壁までの距離[mm](-1 なら補正なし)
	float c;		//境目の推定[mm](G_Odo_X)
} EdgeRec;
static PillarEdge Edge_Det[2];	//0: 右(g_sensor[1])、1: 左(g_sensor[2])
static volatile EdgeRec Edge_Rec[2][EDGE_Q];
static volatile unsigned Edge_Count[2] = { 0, 0 };
static volatile float Edge_Mask_X = -1e9f;	//最後に履歴を空にした(ターンなど)ときの距離

/* ---- 壁切れのログ(モード3 No.6〜9、最短走行のあと モード4 No.15 で出す) ---- */
#define EDGE_LOG_N 160
typedef struct {
	float x, a, b;
	short type;
	short info;
} EdgeLogRec;
static EdgeLogRec Edge_Log[EDGE_LOG_N];
static volatile int Edge_Log_Count = 0;
static volatile int Edge_Log_On = 0;
static int Edge_Log_Th = 0;	//閾値の判定(従来・中央値)も残すか
static float Edge_Log_X0 = 0;
static float Edge_Log_Border = -1;	//区画の境目(Edge_Log_Border + 180k)。-1 は最短走行のログ
static float Edge_Log_Speed = -1;	//テストの速度。-1 は最短走行のログ
static int Edge_Th_Arm[2][2];
/* 壁切れのテスト(モード3 No.6〜9)のときだけ、横センサの値(3回の中央値)を1mmごとに残す。
 * ビューアで「センサ値と距離」のグラフにし、柱のところで値がどう下がるかを見る */
#define EDGE_SIG_N 1000
static int16_t Edge_Sig[EDGE_SIG_N][2];	//[1mmごと][0: 右 / 1: 左]
static volatile int Edge_Sig_Count = 0;
#define EDGE_TH_REARM 20

static void Edge_Log_Put(int type, int info, float x, float a, float b) {
	if (!Edge_Log_On || Edge_Log_Count >= EDGE_LOG_N) return;
	EdgeLogRec *r = &Edge_Log[Edge_Log_Count];
	r->x = x;
	r->a = a;
	r->b = b;
	r->type = type;
	r->info = info;
	Edge_Log_Count++;
}

void Wall_Edge_Sample() {
	float v_robot = (G_Tire_Speed_L + G_Tire_Speed_R) / 2;
	float x = G_Odo_X - v_robot * Edge_Lag_S;
	if (fabsf(G_Gyro_Z) > Edge_Max_W) {
		PillarEdge_Reset(&Edge_Det[0]);
		PillarEdge_Reset(&Edge_Det[1]);
		Edge_Mask_X = x;
		return;
	}
	for (int s = 0; s < 2; s++) {
		int i = (s == 0) ? 1 : 2;
		float med = Wall_Sensor_Median3(i);
		float dist = WallDist_mm((s == 0) ? WALLDIST_R : WALLDIST_L, (int) med);
		if (PillarEdge_Update(&Edge_Det[s], x, med, dist)) {
			EdgeRec r;
			r.ev = Edge_Det[s].ev;
			/* 横のずれの補正: その側の壁までの距離。なければ反対側の壁から(168 - 反対側の距離) */
			r.d_eff = r.ev.d_before;
			float d_other;
			if (r.d_eff < 0 && DistHist_At(&Edge_Det[1 - s].d, r.ev.x - Edge_D_Back, &d_other)
					&& d_other <= 120) {
				r.d_eff = 2 * WALLDIST_CENTER_MM - d_other;
			}
			r.c = r.ev.x + Edge_Off[s][r.ev.kind];
			if (r.d_eff >= 0) {
				r.c += (r.d_eff - WALLDIST_CENTER_MM) * Edge_Tan;
			}
			Edge_Rec[s][Edge_Count[s] % EDGE_Q] = r;
			Edge_Count[s]++;
			Edge_Log_Put(EDGE_LOG_FALL, s * 2 + r.ev.kind, r.ev.x, r.c, r.ev.high);
		}
		if (Edge_Log_On && Edge_Log_Th && s == 1) {
			/* 1mm進むごとに左右の値を残す(左を処理したあと、左右そろったところで) */
			int k = (int) (x - Edge_Log_X0);
			float mr = Wall_Sensor_Median3(1);
			while (Edge_Sig_Count <= k && Edge_Sig_Count < EDGE_SIG_N) {
				Edge_Sig[Edge_Sig_Count][0] = (int16_t) mr;
				Edge_Sig[Edge_Sig_Count][1] = (int16_t) med;
				Edge_Sig_Count++;
			}
		}
		if (Edge_Log_On && Edge_Log_Th) {
			/* 比べるためだけ: 従来(g_sensor_av < Cut_*)と、中央値 < Cut_* で反応した位置 */
			float cut = (s == 0) ? Cut_R : Cut_L;
			float lv[2] = { g_sensor_av[i], med };
			for (int k = 0; k < 2; k++) {
				if (lv[k] > cut + EDGE_TH_REARM) {
					Edge_Th_Arm[s][k] = 1;
				} else if (Edge_Th_Arm[s][k] && lv[k] < cut) {
					Edge_Log_Put(EDGE_LOG_TH, s * 2 + k, G_Odo_X, 0, 0);
					Edge_Th_Arm[s][k] = 0;
				}
			}
		}
	}
}

int Edge_Cut_Find(float lo, float hi, float *c, int *src) {
	int have[2] = { 0, 0 };
	float cs[2] = { 0, 0 }, hs[2] = { 0, 0 };
	float mask_x = Edge_Mask_X;
	for (int s = 0; s < 2; s++) {
		/* 割り込み(Wall_Edge_Sample)が書き足すあいだに読んだら、読み直す(割り込みは止めない) */
		unsigned n;
		do {
			n = Edge_Count[s];
			have[s] = 0;
			for (unsigned k = 0; k < EDGE_Q && k < n; k++) {
				const volatile EdgeRec *r = &Edge_Rec[s][(n - 1 - k) % EDGE_Q];
				/* ターンをはさんだ推定は、まっすぐ走った距離の関係が成り立たないので使わない */
				if (r->ev.x < mask_x) break;
				if (r->c >= lo && r->c <= hi) {
					have[s] = 1;
					cs[s] = r->c;
					hs[s] = r->ev.high;
					break;
				}
			}
		} while (n != Edge_Count[s]);
	}
	int sr = PillarEdge_Fuse(have[0], cs[0], hs[0], have[1], cs[1], hs[1], Edge_Pair_Tol, c);
	if (src) *src = sr;
	return sr != 0;
}

/* with_thresholds: 閾値の判定と1mmごとのセンサ値も残す(テスト)。border / speed: テストの区画の境目と速度(最短走行は -1) */
void Edge_Log_Start(int with_thresholds, float border, float speed) {
	Edge_Log_On = 0;
	Edge_Log_Border = border;
	Edge_Log_Speed = speed;
	Edge_Log_Count = 0;
	Edge_Log_X0 = G_Odo_X;
	Edge_Log_Th = with_thresholds;
	Edge_Sig_Count = 0;
	for (int s = 0; s < 2; s++) {
		for (int k = 0; k < 2; k++) {
			Edge_Th_Arm[s][k] = 0;
		}
	}
	Edge_Log_On = 1;
}

void Edge_Log_Stop() {
	Edge_Log_On = 0;
}

/* 走行の関数(motor.c)から、壁切れの待ち始め・結果を残す */
void Edge_Log_Mark(int type, int info, float x, float a, float b) {
	__disable_irq();
	Edge_Log_Put(type, info, x, a, b);
	__enable_irq();
}

/* ログをUARTに出す。first_border >= 0 なら、区画の境目(first_border + 180k、ログ開始からの距離)からのずれも出す。
 * 1行ずつ、種類ごとに次の列:
 *   fall,側,種類,x(端),c(境目の推定),H                       下がる端
 *   th,側,av|med,x                                              閾値の判定(比べるためだけ)
 *   st,側,x(区画中心),X(前距離)                                 前距離の始め(柱を使うとき)
 *   st_cut,src,x(曲がり始め),c                                  前距離の終わり。src: 0 距離だけ、1 右、2 左、3 両方、+4 後ろの境目、+8 左右が合わない、
 *                                                               16 従来で壁切れした(c は壁切れした位置)、32 従来で壁がなく距離だけ
 *   (st / st_cut は柱でも従来でも残す)
 *   cut,側,x                                                    従来の壁切れで反応した位置
 *   end,-,x(始め),X(後距離)  /  end_cut,src,x(終わり)           後距離
 *   full,行数                                                   ログがいっぱいで、ここまでしか残っていない
 *   sig,x,右,左                                                 横センサの値(3回の中央値)。1mmごと。テストのときだけ
 * x はログ開始からの距離[mm]。全体を #BEGIN EDGE ... / #END EDGE ではさむ(tools/viewer.html で読む) */
int Edge_Log_Has_Data() {
	return Edge_Log_Count > 0 || Edge_Sig_Count > 0;
}

void Edge_Log_Print() {
	float first_border = Edge_Log_Border;
	float speed = Edge_Log_Speed;
	static const char *side_name[2] = { "R", "L" };
	static const char *kind_name[2] = { "wall", "pillar" };
	/* tools/viewer.html で読む。border は区画の境目(first_border + 180k)、v はテストの速度(-1 は最短走行のログ) */
	printf("\r\n#BEGIN EDGE border=%.0f v=%.0f off_r_wall=%.1f off_r_pillar=%.1f off_l_wall=%.1f off_l_pillar=%.1f tan=%.2f min_high=%.0f high_len=%.0f pillar=%d fast=%d cut_l=%.0f cut_r=%.0f\r\n",
			first_border, speed, Edge_Off[0][0], Edge_Off[0][1], Edge_Off[1][0], Edge_Off[1][1], Edge_Tan,
			Edge_Min_High, Edge_High_Len, G_WallCut_Pillar, G_WallCut_Fast, Cut_L, Cut_R);
	for (int n = 0; n < Edge_Log_Count; n++) {
		const EdgeLogRec *r = &Edge_Log[n];
		float x = r->x - Edge_Log_X0;
		switch (r->type) {
		case EDGE_LOG_FALL: {
			float c = r->a - Edge_Log_X0;
			printf("fall,%s,%s,%.1f,%.1f,%.0f", side_name[r->info / 2], kind_name[r->info % 2], x, c, r->b);
			if (first_border >= 0) {
				float k = floorf((c - first_border) / 180.0f + 0.5f);
				printf(",border=%.0f,c-border=%.1f", first_border + 180.0f * k, c - (first_border + 180.0f * k));
			}
			break;
		}
		case EDGE_LOG_TH:
			printf("th,%s,%s,%.1f", side_name[r->info / 2], (r->info % 2) ? "med" : "av", x);
			if (first_border >= 0) {
				float k = floorf((x + 84.0f - first_border) / 180.0f + 0.5f);
				printf(",x-border=%.1f", x - (first_border + 180.0f * k));
			}
			break;
		case EDGE_LOG_ST:
			printf("st,%s,%.1f,%.0f", side_name[r->info ? 1 : 0], x, r->a);
			break;
		case EDGE_LOG_ST_CUT:
			printf("st_cut,%d,%.1f,%.1f", r->info, x, (r->info & 0x13) ? r->a - Edge_Log_X0 : -1.0f);
			break;
		case EDGE_LOG_CUT:
			printf("cut,%s,%.1f", side_name[r->info ? 1 : 0], x);
			break;
		case EDGE_LOG_END:
			printf("end,-,%.1f,%.0f", x, r->a);
			break;
		case EDGE_LOG_END_CUT:
			printf("end_cut,%d,%.1f", r->info, x);
			break;
		}
		printf("\n\r");
	}
	if (Edge_Log_Count >= EDGE_LOG_N) {
		printf("full,%d\r\n", EDGE_LOG_N);	//ログがいっぱいで、ここまでしか残っていない
	}
	/* 横センサの値(テストのときだけ): sig,x(ログ開始からの距離[mm]),右,左 */
	for (int k = 0; k < Edge_Sig_Count; k++) {
		printf("sig,%d,%d,%d\r\n", k, Edge_Sig[k][0], Edge_Sig[k][1]);
	}
	printf("#END EDGE\r\n");
}

/* 横壁制御を計算する。1msに1回だけ呼ぶこと。
 * 以前は calWallConrol() の中で計算していて、1msに3回(左PWM・右PWM・ジャイロ目標)
 * 呼ばれていたため、2回目以降は Wall_old_error が更新済みでD項が0になり、
 * D項が左のPWMにしか効いていなかった */
float Wall_Control_Update() {

	int Sensor_diff_L = abs(g_sensor[2][0] - g_sensor[2][1]);
	int Sensor_diff_R = abs(g_sensor[1][0] - g_sensor[1][1]);
	float PID_Wall = 0;
	int Wall_st = 0;
	int Sensor_L_med = Wall_Sensor_Median3(2);
	int Sensor_R_med = Wall_Sensor_Median3(1);
	/* 距離版の切れ目判定用に、横壁までの距離を1msごとに残す。
	 * 上の Sensor_diff_L/R は g_sensor[i][1] がずらし後に [0] と同じ値なので常に0で、
	 * 切れ目判定が一度も効いていなかった(センサ値版はそのまま) */
	#define WALL_HIST_N 16
	static float hist_L[WALL_HIST_N], hist_R[WALL_HIST_N];
	static int hist_i = 0;
	float dL = WallDist_mm(WALLDIST_L, Sensor_L_med);
	float dR = WallDist_mm(WALLDIST_R, Sensor_R_med);
	hist_i = (hist_i + 1) % WALL_HIST_N;
	hist_L[hist_i] = dL;
	hist_R[hist_i] = dR;
	if (G_WallCtrl_Use_mm) {
		/* 距離版: 近い壁(WallCtrl_mm_MaxDist以内)で、WallCtrl_mm_StableLen 進む間に
		 * 距離があまり変わっていないものだけ使う。比べる相手は速度から何ms前かを決める */
		int back = WALL_HIST_N - 1;
		if (G_Motor_V_Target > 0) {
			back = (int) (WallCtrl_mm_StableLen * 1000 / G_Motor_V_Target + 0.5f);
		}
		if (back < 2) back = 2;
		if (back > WALL_HIST_N - 1) back = WALL_HIST_N - 1;
		int j = (hist_i - back + WALL_HIST_N) % WALL_HIST_N;
		int use_L = (dL <= WallCtrl_mm_MaxDist) && (fabsf(dL - hist_L[j]) < WallCtrl_mm_StableTH);
		int use_R = (dR <= WallCtrl_mm_MaxDist) && (fabsf(dR - hist_R[j]) < WallCtrl_mm_StableTH);
		Wall_st = (use_L ? 1 : 0) + (use_R ? 2 : 0);
	} else if ((g_sensor_av[2] > Wall_TH_L) && (Sensor_diff_L < Sensor_diff_TH)) { //左あり
		if ((g_sensor_av[1] > Wall_TH_R) && (Sensor_diff_R < Sensor_diff_TH)) { //右あり
			Wall_st = 3;
		} else { //右なし
			Wall_st = 1;
		}
	} else {
		if ((g_sensor_av[1] > Wall_TH_R) && (Sensor_diff_R < Sensor_diff_TH)) {
			Wall_st = 2;
		} else {
			Wall_st = 0;
		}
	}

	float Kp_base = G_WallCtrl_Use_mm ? Kp_mm : Kp;
	float Kd_base = G_WallCtrl_Use_mm ? Kd_mm : Kd;
	if (G_Motor_V_Target >= 2000) {
		KP = Kp_base * (2000 / 500);
		KD = Kd_base * (2000 / 500);
	} else {
		float gain = G_Motor_V_Target / 500;
		if (G_WallCtrl_Use_mm && (G_Motor_V_Target > 0) && (gain < WallCtrl_mm_GainMin)) {
			gain = WallCtrl_mm_GainMin;
		}
		KP = Kp_base * gain;
		KD = Kd_base * gain;
	}

	int Sensor_L = g_sensor[2][0];
	int Sensor_R = g_sensor[1][0];

	if (Sensor_L > 600) {
		Sensor_L = 600;
	}
	if (Sensor_R > 600) {
		Sensor_R = 600;
	}

	/* 距離モードでは「壁に近いほど正」になるよう (中心距離 - 実距離) を使う。
	 * 近づいたときの値の急増は変換で吸収されるので600での頭打ちは不要 */
	float Err_L, Err_R;
	if (G_WallCtrl_Use_mm) {
		Err_L = WALLDIST_CENTER_MM - dL;
		Err_R = WALLDIST_CENTER_MM - dR;
		if (Err_L > WallCtrl_mm_ErrMax) Err_L = WallCtrl_mm_ErrMax;
		if (Err_L < -WallCtrl_mm_ErrMax) Err_L = -WallCtrl_mm_ErrMax;
		if (Err_R > WallCtrl_mm_ErrMax) Err_R = WallCtrl_mm_ErrMax;
		if (Err_R < -WallCtrl_mm_ErrMax) Err_R = -WallCtrl_mm_ErrMax;
	} else {
		Err_L = Sensor_L - Wall_L;
		Err_R = Sensor_R - Wall_R;
	}

	LED_Reset();
	if (Wall_Flont_Av() > 800) {
		Wall_error = 0;
		Wall_old_error = 0;
	} else if (Wall_st == 3) {
		Wall_error = Err_L - Err_R;
		LED_ON_L();
		LED_ON_R();
	} else if (Wall_st == 2) {
		Wall_error = 2.0 * -Err_R;
		LED_ON_R();
	} else if (Wall_st == 1) {
		Wall_error = 2.0 * Err_L;
		LED_ON_L();
	} else {
		Wall_error = 0;
		Wall_old_error = 0;
	}

	/* 距離版: 壁の有無が切り替わった瞬間は誤差が段差で変わるので、D項を出さない */
	static int Wall_st_prev = 0;
	if (G_WallCtrl_Use_mm && Wall_st != Wall_st_prev) {
		Wall_old_error = Wall_error;
	}
	Wall_st_prev = Wall_st;

	Wall_Delta_error = Wall_error - Wall_old_error;
	Wall_old_error = Wall_error;

	PID_Wall = KP * Wall_error + KD * Wall_Delta_error;

	/* 走行ログ用(LOG.c) */
	G_Wall_P = KP * Wall_error;
	G_Wall_D = KD * Wall_Delta_error;
	G_Wall_St = (Wall_Flont_Av() > 800) ? 0 : Wall_st;
	G_Wall_dL = dL;
	G_Wall_dR = dR;

	G_Wall_PID = PID_Wall;
	return PID_Wall;
}

float calWallConrol_NANAME() {
	float PID_Wall = 0;

	KP = Kp_Na;
	KD = Kd_Na;

	KP = Kp_Na * (G_Motor_V_Target / 1000);
	KD = Kd_Na * (G_Motor_V_Target / 1000);
	LED_Reset();
	if (g_sensor[0][0] > 110) {
		Wall_error_NA = g_sensor[0][0] - 110;
		LED_ON_L();
	} else if (g_sensor[3][0] > 110) {
		Wall_error_NA = -(g_sensor[3][0] - 110);
		LED_ON_R();
	} else {
		Wall_error_NA = 0;
		Wall_old_error_NA=0;
	}

	Wall_Delta_error_NA = Wall_error_NA - Wall_old_error_NA;
	Wall_old_error_NA = Wall_error_NA;
	PID_Wall = KP * Wall_error_NA + KD * Wall_Delta_error_NA;

	return PID_Wall;
}

float calWallConrol_Flontwall_ST() {
	float PID_Wall = 0;

	if (Wall_Flont_Av() > 1000) {
		Wall_error_FW = FlontWall_Distance - 1000;
	} else {
		Wall_error_FW = FlontWall_Distance - Wall_Flont_Av();
	}

	Wall_Delta_error_FW = Wall_error_FW - Wall_old_error_FW;
	Wall_old_error_FW = Wall_error_FW;
	PID_Wall = Kp_FW * Wall_error_FW + Kd_FW * Wall_Delta_error_FW;

	return PID_Wall;
}

float calWallConrol_Flontwall_Turn() {
	float PID_Wall = 0;

	Wall_error_FW = (g_sensor[0][0]) - (g_sensor[3][0]);

	Wall_Delta_error_FW = Wall_error_FW - Wall_old_error_FW;
	Wall_old_error_FW = Wall_error_FW;
	PID_Wall = Kp_FW_dire * Wall_error_FW + Kd_FW_dire * Wall_Delta_error_FW;

	return PID_Wall;
}
