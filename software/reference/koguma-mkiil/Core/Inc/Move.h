/*
 * Move.h
 *
 *  Created on: 2023/08/29
 *      Author: akihi
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#include "ShortParam.h"

extern int G_Pass_before;
extern int G_Pass_after;

void Robot_adjustment();
void Robot_adjustment_180();
void Robot_adjustment_Back();
void Robot_adjustment_Back_Only();

void Robot_Maze_Sula_Action();
void Robot_Maze_Suction_Action();
void Robot_Maze_Pass_Action();

void Sula_Shortest_Move300(int, int);
void Sula_Shortest_Move500(int, int);

void Short_NANAME_Move1000(int, int);
extern int G_Short_AC_NANAME;
void Short_NANAME_Move2000(int, int);
void Short_NANAME_Move2400(int, int);
void Short_NANAME_Move2700(int, int);
void Short_NANAME_MoveTurnV(int, int);

void Short_Dijkstra_Move2000(int, int);
extern int G_Short_Reached;	//Short_Dijkstra_Move2000 がゴールまで走りきったら1
extern int G_Short_End_OK, G_Short_End_X, G_Short_End_Y, G_Short_End_Dir;	//最短走行で止まる区画と向き(走る前に求める)
void Short_Dijkstra_Move2400(int, int);
void Short_Dijkstra_MoveTurnV(int, int);
void TurnV_Test(int kind);	/* モード10 No.1〜7: ターン単体を TurnV_Table の速度で */

/* 最短走行の最初のターン(Move.c)。モード4 No.9〜No.11 の調整用にも使う */
#define FIRST_TURN_BIG90 0
#define FIRST_TURN_BIG180 1
#define FIRST_TURN_IN45 2
#define FIRST_TURN_IN135 3
#define FIRST_TURN_NUM 4
typedef struct {
	float pre;	//前距離[mm](区画中心から)
	float ang;	//角度[deg]
	float w;	//最大角速度[deg/s]
	float w_ac;	//角加速度[deg/s^2]
	float post;	//後距離[mm]
	float v;	//速度[mm/s](角速度・角加速度はこの速度のときの値)
} FirstTurnParam;
extern FirstTurnParam G_First_Turn[FIRST_TURN_NUM];
float Short_First_Turn_Run(int kind, int dir, float V);
extern int G_First_Turn_Accel;	/* 1: 最初のターンを加速しながら曲がる(Move.c) */

/* 2000・2400のターンの値(G_Turn2000 / G_Turn2400)と Turn2000_Param は ShortParam.h(2026-10-27) */
void Turn2000_Run(int kind, int dir);
void Turn2000_Test(int kind);	/* モード6 No.1〜7 */
void Turn2400_Run(int kind, int dir);
void Turn2400_Test(int kind);	/* モード7 No.1〜7 */
/* 1000のターンの値(Move.c の G_Turn1000、2026-10-21)。吸引探索の既知区間の斜め(モード0 No.12)・モード5で共通。形と並びは G_Turn2000 と同じ */
extern Turn2000_Param G_Turn1000[T2000_NUM];
void Turn1000_Run(int kind, int dir);
void Turn1000_Test(int kind);	/* モード5 No.1〜7 */
extern int G_Known_Naname;	/* 1: 吸引探索の既知区間を斜めで走る(Known_Pass_NANAME、G_Turn1000。モード0 No.12) */

/* 2200のターン(Move.c の G_Turn2200、Turn-Sim_KOGUMA の「Cの表をコピー」をそのまま貼る)。並びはシミュレーターと同じ */
enum {
	TS_IN45, TS_V90, TS_IN135, TS_OUT45, TS_OUT135, TS_BIG90, TS_BIG180, TS_NUM
};
typedef struct {
	float v;		/* 速度[mm/s] */
	float v_mid;	/* ターン中の最低速度(使わない) */
	float angle;	/* 角度[deg](左) */
	float w_max;	/* 最大角速度[deg/s] */
	float w_ac;		/* 角加速度[deg/s^2] */
	float st;		/* 前距離[mm](ターン区間の始まりから) */
	float end;		/* 後距離[mm] */
} TurnSim_Param;
extern TurnSim_Param G_Turn2200[TS_NUM];
void Turn2200_Test(int no);	/* モード11 No.1〜7(停止位置テスト) */

extern void (*G_Stop_Hook)(void);	//区画の中央で止まっているあいだに呼ぶ計算
void Robot_Maze_Go_From_Stop(float, float);
void Robot_Maze_Stop_And_Go(int);
extern int G_Log_Next_UTurn;	//1にすると次の吸引探索のUターンでログを取り始める

//void Sula_Shortest_Move800();
//void Sula_Shortest_Move1100();
//void Sula_Shortest_Move1000(int, int);
//void Sula_Shortest_Move1200();
//void Shot_NANAME_Move1000(int, int);
//void Adjust_Speed(int);
//void Shot_NANAME_Move1700_FUN(int, int);
//void Shot_NANAME_Move2000_FUN(int, int);

#endif /* INC_MOVE_H_ */
