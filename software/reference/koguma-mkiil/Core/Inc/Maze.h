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
void Maze_Unknown_Target_ModeSet(int, int);

void Known_Pass_Generation();
void Known_Pass_Compression_NANAME();

void Maze_Mapping();
void Maze_Save();
void Maze_Road();
void Maze_Look_Mapping();
void Maze_Wall_fill();
void Maze_Shortest_Calculation();
void Shortest_Pass_Compression();
void Shortest_Pass_Compression_NANAME();
void Pass_zero_act();


void  Maze_Dijkstra_Calculation();
void Maze_Dijkstra_Mapping();
