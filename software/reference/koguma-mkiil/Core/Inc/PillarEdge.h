/*
 * PillarEdge.h
 *
 *  柱(区画の境目)の端を、横センサの値から見つける(2026-10-12)。設計は docs/pillar_cut_design.md。
 *
 *  左右それぞれに1つ持ち、センサ値ができるたびに PillarEdge_Update() を呼ぶ。
 *  値は走った距離ごとの履歴(DistHist)に残し、次の端を見つけたら位置をイベントとして出す。
 *
 *  - 下がる端(柱の後ろの端。壁の終わり #2 と、柱だけの山 #3): 直前 Edge_High_Len[mm] の最大値 H の
 *    半分を切った位置。会場の明るさや壁との距離で H が変わっても、半分を切る位置は変わりにくい
 *  - 上がる端(#4)は2段目(まだない)
 *
 *  HALに依存しないので、ホスト(PC)のgccでもそのままテストできる。
 */

#ifndef INC_PILLAREDGE_H_
#define INC_PILLAREDGE_H_

#include "DistHist.h"

#define EDGE_KIND_WALL 0	//壁の終わり(#2): 下がる前に、壁が Edge_Plateau_Wall 以上続いていた
#define EDGE_KIND_PILLAR 1	//柱だけの山(#3)

typedef struct {
	float x;		//半分を切った位置(走行距離[mm]、1mmより細かく補間)
	float high;		//その前の最大値 H(センサ値)
	float d_before;	//半分を切る Edge_D_Back[mm] 前の、その側の壁までの距離[mm]。壁でなければ -1
	int kind;		//EDGE_KIND_*
} PillarEdgeEvent;

typedef struct {
	DistHist v;		//センサ値
	DistHist d;		//その側の壁までの距離[mm]
	int armed;		//壁か柱が見えている(次に半分を切ったら端)
	float high_from;	//見え始めた位置(壁の終わりか柱だけかを分ける)
	float prev_x, prev_v;
	int has_prev;
	PillarEdgeEvent ev;	//最後に見つけた端
} PillarEdge;

/* パラメータ(クラシックの値。ハーフでは測り直す) */
extern float Edge_High_Len;		//最大値 H を見る長さ[mm]
extern float Edge_Min_High;		//H がこれ未満なら、壁も柱も見えていないとみなす(センサ値)
extern float Edge_Rearm_Ratio;	//値が H のこの割合以上になったら「見えている」
extern float Edge_Plateau_Wall;	//見えていた長さがこれ以上なら壁の終わり、未満なら柱だけ[mm]
extern float Edge_D_Back;		//壁までの距離を、半分を切る何mm前で取るか
extern float Edge_Min_Hist;		//履歴がこれより短いうちは端を出さない[mm](空にした直後)

void PillarEdge_Reset(PillarEdge *e);

/* x: 走行距離[mm](値を測った時点。遅れは呼ぶ側で直す)、v: センサ値、d: その側の壁までの距離[mm]。
 * 端を見つけたら1を返し、e->ev に入れる */
int PillarEdge_Update(PillarEdge *e, float x, float v, float d);

/* 左右の境目の推定をまとめる(向きのずれ・横のずれは左右で逆向きに出るので、平均で打ち消す)。
 * have_r / have_l: その側に推定があるか、c_*: 境目の推定[mm]、h_*: その端の信号の大きさ(H)。
 * 両方あって tol 以内なら平均、離れていたら H が大きいほう(壁に近く確か)を使う。
 * 戻り値: 0 なし。それ以外はビットの組み合わせ(EDGE_SRC_*) */
#define EDGE_SRC_R 1		//右を使った
#define EDGE_SRC_L 2		//左を使った
#define EDGE_SRC_DISAGREE 8	//両方あったが離れていたので片方だけ使った
int PillarEdge_Fuse(int have_r, float c_r, float h_r, int have_l, float c_l, float h_l,
		float tol, float *c);

#endif /* INC_PILLAREDGE_H_ */
