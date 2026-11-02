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


#define TIREDIAMETER 23.42 //2026-10-21 モード4 No.1(吸引35)で実測。1080mm 指示でエンコーダ 1065.1mm、実際 1080mm。前は 23.1(10-09、吸引70、指示の距離で割っていた)
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

/* ダイクストラのコストを最短走行の時間にしたとき(G_Dijk_Time_Cost)の単位: 1秒あたり(2000 なら 0.5ms) */
#define DIJK_COST_PER_S 2000

/* ダイクストラの「未到達(=無限大)」センチネル。これより大きくなる経路は「行けない」扱い。
 * 以前は ST_COST * 2 * (MAZE_SIZE+1)^2 (16x16なら2890)だったが、走行時間の表(0.5ms単位)では
 * 16x16の普通の経路でも 6000 前後になるので、コスト(uint16_t)の上限近くにした(32.5秒)。
 * 16x16では、1区画あたり多くても約210(1歩の区間の縦横)なので、256区画を通っても約54000で届かない。
 * 32x32(PCの検証用)は経路が長く届くことがあるので、コストを uint32_t にする(Maze.c) */
#if MAZE_SIZE > 16
#define DIJK_MAXCOST 2000000000
#else
#define DIJK_MAXCOST 65000
#endif
#define DIJK_WALLCOST (DIJK_MAXCOST + 1)

#endif /* INC_DEFINE_H_ */
