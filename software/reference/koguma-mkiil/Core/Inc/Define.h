/*
 * Define.h
 *
 *  Created on: Jun 14, 2025
 *      Author: akihi
 */

#ifndef INC_DEFINE_H_
#define INC_DEFINE_H_

/* ゴールは迷路中央の2x2区画(標準ルール)。MAZE_SIZEを変えたらここも中央に追従させる */
#define MAZE_GOOL_X (MAZE_SIZE / 2 - 1)
#define MAZE_GOOL_Y (MAZE_SIZE / 2 - 1)

/* 迷路の1辺のマス数。16 <-> 32 の切り替えはここ1箇所でOK
 * (壁ビット幅・ノード配列サイズ・キュー容量は全てここから導出される)
 * ビルド時に -DMAZE_SIZE=32 でも上書き可能 (PCシミュレータでの検証用) */
#ifndef MAZE_SIZE
#define MAZE_SIZE 16
#endif

/* 歩数マップの「未到達」センチネル、BFSの最大訪問数、圧縮パス配列長を兼ねる。
 * 迷路の全マス数以上であることが必須 (BFSがそれだけのマスを訪問し得るため) */
#define MAX_STEP (MAZE_SIZE * MAZE_SIZE)


#define TIREDIAMETER 23.5 //23.3mm
#define TIREBETWEEN 55
#define PI 3.1415
#define LIMITBATT 15.6
#define REFERENCE_V 16.0

#define INTERRUPTTIME 0.001//5kHz 0.0002 1kHz 0.001

#define ROBOT_M 0.078

#define MOTORR 5.4
#define KT 0.00263
#define KE 0.00209

#define GEARRATIO 1.7073

#define MAX_MAZE_STEP 255

/* セル単位BFS(Maze_Step_Calculate)用キュー。各セルは1回しかpushされないため
 * 全マス数で十分 */
#define MAX_QUEUE_NUM MAX_STEP

/* ダイクストラのノードグラフ(Row/Column半セルノード)用キュー。
 * inQueueフラグにより同一ノードは同時に1回しかキューに乗らないので、
 * ノード総数 2*(MAZE_SIZE+1)^2 が「絶対にオーバーフローしない」理論上限。
 * それをそのまま容量にする(オーバーフロー時に静かに壊れるのを避けるため) */
#define MAX_QUEUE_NODE_NUM (2 * (MAZE_SIZE + 1) * (MAZE_SIZE + 1))

#define ST_COST 5
#define DIAG_COST 3
#define CON_COST 2

/* ダイクストラの「未到達(=無限大)」センチネル。最短路はノードを再訪しないので、
 * 実際のコストは「ノード総数 × 最大辺コスト」を超えない。それより確実に大きい値にする。
 * 999固定だと32x32で実コストがこれを超えて「無限大のまま更新されない」バグになるため
 * MAZE_SIZEに追従させる (16x16なら2890、32x32なら10890) */
#define DIJK_MAXCOST (ST_COST * 2 * (MAZE_SIZE + 1) * (MAZE_SIZE + 1))
#define DIJK_WALLCOST (DIJK_MAXCOST + 1)

#endif /* INC_DEFINE_H_ */
