/*
 * Move.h
 *
 *  Created on: 2023/08/29
 *      Author: akihi
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

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

/* 2000のターンの値(Move.c の G_Turn2000)。BFS・ダイクストラ・モード6で共通 */
#define WC2000_NORMAL 0
#define WC2000_NANAME 1
enum {
	T2000_BIG90, T2000_BIG180, T2000_IN45, T2000_IN135, T2000_OUT45, T2000_OUT135, T2000_V90, T2000_NUM
};
typedef struct {
	int st_kind;	/* 前距離の壁切れ: WC2000_NORMAL / WC2000_NANAME */
	float st;		/* 前距離[mm] */
	float ang;		/* 角度[deg](左が正) */
	float w;		/* 最大角速度[deg/s] */
	float w_ac;		/* 角加速度[deg/s^2] */
	int end_kind;	/* 後距離の壁切れ */
	float end;		/* 後距離[mm] */
} Turn2000_Param;
extern Turn2000_Param G_Turn2000[T2000_NUM];
void Turn2000_Run(int kind, int dir);
void Turn2000_Test(int kind);	/* モード6 No.1〜7 */

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
