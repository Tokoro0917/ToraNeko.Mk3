/*
 * Maze.h
 *
 *  Created on: 2023/08/18
 *      Author: akihi
 */

#define INC_MAZE_H_
#define INC_MAZE_H_

#include <stdint.h>
#include"Define.h"
#include"ShortParam.h"

typedef struct {
	int head;
	int tail;
	uint16_t data[MAX_QUEUE_NUM];
} QUEUE_T;

typedef struct {
	int head;
	int tail;
	uint16_t data[MAX_QUEUE_NUM];
} STACK_T;

extern int G_Gool_X;
extern int G_Gool_Y;

extern uint8_t G_MAZE_Explored[MAZE_SIZE][MAZE_SIZE];

/* 行き止まりと判明して全面探索の対象から外したマス。壁マップとは別管理で、
 * 走行の通行判定には使わない(Maze.c冒頭のコメント参照) */
extern uint8_t G_MAZE_Closed[MAZE_SIZE][MAZE_SIZE];
extern int G_DeadEnd_Fill_Enable;	//0で従来動作(行き止まり潰しなし)に戻る

extern uint32_t G_Maze_Row[MAZE_SIZE+1];
extern uint32_t G_Maze_Column[MAZE_SIZE+1];

extern int G_Robot_Direction;
extern int G_Robot_Lastaction;
extern int G_Maze_Front_Override;	//迷路に書く前壁(-1 なら G_Wall_data[0] と [3] の両方)
extern int G_Just_UTurned;

extern uint16_t G_Maze_Flont;
extern uint16_t G_Maze_Back;
extern uint16_t G_Maze_Left;
extern uint16_t G_Maze_Right;

extern int G_Robot_MAZE_X;
extern int G_Robot_MAZE_Y;

extern uint32_t G_Maze_Row_Save[MAZE_SIZE+1];
extern uint32_t G_Maze_Column_Save[MAZE_SIZE+1];

extern uint16_t G_Step_Map[MAZE_SIZE][MAZE_SIZE];

extern int16_t G_Short_Pass[MAX_STEP];
extern int16_t G_Short_Pass_CP[MAX_STEP];
extern int16_t G_Short_Pass_NANAME[MAX_STEP];

extern int16_t G_Known_Pass[MAX_STEP];
extern int16_t Known_Pass_NANAME[MAX_STEP];

extern int16_t G_Dijk_Path_X[MAX_STEP];
extern int16_t G_Dijk_Path_Y[MAX_STEP];
extern int G_Dijk_Path_Len;

extern int G_Unknown_Target_Lost;	//未知壁探索で、見に行く途中で目標の区画に行けなくなったら1(2026-10-27)
extern int G_Unknown_Target_X;
extern int G_Unknown_Target_Y;

void pushQueue_walk(QUEUE_T*, unsigned short);
unsigned short popQueue_walk(QUEUE_T*);

void pushStack_walk(STACK_T*, unsigned short);
unsigned short popStack_walk(STACK_T*);

void Maze_Initialization();
void Maze_Wall_Search(int, int, int);
void Maze_Wall_Update();

void Maze_Unkown_ALL_ModeSet();
void Maze_Unkown_ALL_ModeOFF();
int Maze_All_MODE_Check();

void Maze_Gool_Setting(int );
void Maze_DeadEnd_Fill();
void Maze_Step_Calculate();

int Maze_Unknown_Wall_Scan(void);
int Maze_Unknown_Wall_Still_Unknown(void);
int Maze_Unknown_Wall_Is_Wall(void);
void Maze_Unknown_Target_ModeSet(int, int);

void Known_Pass_Generation();
void Known_Pass_Compression_NANAME();

void Maze_Mapping();
void Maze_Save();
void Maze_Road();
void Maze_Undo_Clear(void);
int Maze_Undo_Recent_Cells(void);
void Maze_Look_Mapping();
void Maze_Wall_fill();
void Maze_Shortest_Calculation();
void Shortest_Pass_Compression();
void Shortest_Pass_Compression_NANAME();
int Short_Pass_Ends_At_Boundary(void);	//命令列が区画の境目で終わるか(最後が直進なら1、ターンなら区画の中心で終わるので0)
int Short_Pass_End_Cell(int *x, int *y, int *dir);	//最短走行で止まる区画と向き(0 北、1 東、2 南、3 西)。1 求めた
void Pass_zero_act();


void  Maze_Dijkstra_Calculation();
/* ダイクストラのコスト(2026-10-27)。1 なら最短走行の時間(既定)、0 なら以前の ST_COST/DIAG_COST/CON_COST */
extern int G_Dijk_Time_Cost;
/* 最短走行の時間の表(0.5ms単位。Maze_Dijkstra_Set_Run で作る)。
 * 経路のコストは、命令列(G_Short_Pass_NANAME)の命令ごとの時間の合計(tools/route_time.py と同じ数え方) */
#define DIJK_TH_NUM (2 * MAZE_SIZE + 2)
#define DIJK_TD_NUM (2 * MAZE_SIZE + 2)
typedef struct {
	uint16_t th[DIJK_TH_NUM];	//縦横の直線 半区画 n 個
	uint16_t td[DIJK_TD_NUM];	//斜めの直線 n 区間(-50*n)
	uint16_t turn[T2000_NUM];	//ターン(並びは T2000_BIG90 など)。理想の始まりの点から終わりの点まで
	uint16_t start_s;	//走り出し(最初が直線): スタートの区画の中心から境目まで 0 から加速
	uint16_t start_t;	//走り出し(最初がターン)
	uint16_t stop_b;	//止まる(最後が直線。境目から270mm)
	uint16_t stop_c;	//止まる(最後がターン。区画の中心から180mm)
} DijkCost;
extern DijkCost G_Dijk_Cost;
extern int G_Dijk_Best_Cost;	//直近の Maze_Dijkstra_Calculation() の経路のコスト(行けなければ -1)
/* 最短走行の値からコストの表を作る(ac_naname が 0 なら ac-5000)。作るまでは G_Short_Run_Set[0] */
void Maze_Dijkstra_Set_Run(const ShortRunParam *p);
void Maze_Dijkstra_Mapping();
void Maze_Neighbor_Update(void);	//今いる区画の前後左右の歩数を G_Maze_Flont 等に入れる(移動しない)
void Maze_Debug_Dump(void);	//保存済みの迷路と最短経路の命令列をシリアルに出す(モード3 No.0)
