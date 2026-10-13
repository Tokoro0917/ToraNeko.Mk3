/*
 * DistHist.h
 *
 *  センサ値を「走った距離ごと」に残す履歴(2026-10-12、壁切れを柱で見るとき用)。
 *
 *  1msごとのセンサ値を、走行距離 DISTHIST_STEP_MM ごとの点に直して残す。
 *  「何mm前の値」が速度に関係なく取り出せるので、5mm進む間の変化量などが
 *  速度で変わらない。1msで STEP より多く進んだときは、間の点を前後の値から直線で埋める。
 *
 *  HALに依存しないので、ホスト(PC)のgccでもそのままテストできる。
 */

#ifndef INC_DISTHIST_H_
#define INC_DISTHIST_H_

/* 点の間隔[mm]。クラシック(区画180mm)の値。
 * ハーフ(区画90mm)では、柱・壁までの距離がおおよそ半分になるので 0.5 にすると、
 * 柱の端を見る長さ(PillarEdge.h の Edge_High_Len)に入る点の数がクラシックと同じになる(ハーフでは測り直す) */
#define DISTHIST_STEP_MM 1.0f

/* 残す点の数。DISTHIST_N * DISTHIST_STEP_MM [mm] 前まで取り出せる */
#define DISTHIST_N 32

typedef struct {
	float v[DISTHIST_N];	//点の値(リングバッファ)
	int head;		//いちばん新しい点の位置(v[] の添字)
	long grid;		//いちばん新しい点の番号(その点の距離 = grid * STEP)
	int count;		//残っている点の数(最大 DISTHIST_N)
	float last_x;	//最後に入れたサンプルの距離
	float last_v;	//最後に入れたサンプルの値
	int started;	//1回でも入れたら1
} DistHist;

/* 空にする */
void DistHist_Reset(DistHist *h);

/* 距離 x[mm](増える向きのみ。減ったときは値だけ入れ替える)のときの値 v を入れる */
void DistHist_Push(DistHist *h, float x, float v);

/* 距離 x[mm] のときの値を、前後の点(と最後のサンプル)から直線で求めて *out に入れる。
 * 履歴の範囲外なら 0 を返す(*out は変えない) */
int DistHist_At(const DistHist *h, float x, float *out);

/* 最後に入れたサンプルの距離 */
float DistHist_LastX(const DistHist *h);

/* 最後のサンプルから len[mm] 前までの点(と最後のサンプル)の最大値。空なら0 */
float DistHist_Max(const DistHist *h, float len);

/* 残っている長さ[mm](いちばん古い点から最後のサンプルまで) */
float DistHist_Length(const DistHist *h);

#endif /* INC_DISTHIST_H_ */
