/*
 * Maze.c
 *
 *  Created on: 2023/08/18
 *      Author: akihi
 */

#include"Maze.h"
#include"stdio.h"
#include"Wallsensor.h"
#include"stdio.h"
#include "Define.h"
#include <math.h>
#include <string.h>

/* 計算時間の計測(実機のみ。DWTのサイクルカウンタ)。PCのテストでは0のまま。
 * 各関数の最大時間[us]を記録し、Maze_Debug_Dump()で表示する */
uint32_t G_Time_Max_WallUpdate_us = 0;	//Maze_Wall_Update(壁の記録+行き止まり潰し+歩数マップ)
uint32_t G_Time_Max_Known_us = 0;		//Known_Pass_Generation(既知区間の経路生成)
uint32_t G_Time_Max_Step_us = 0;		//Maze_Step_Calculate(歩数マップ)
uint32_t G_Time_Max_Dijk_us = 0;		//Maze_Dijkstra_Calculation
#if defined(STM32F446xx)
#include "stm32f4xx.h"
static uint32_t Maze_Time_Start(void) {
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
	return DWT->CYCCNT;
}
static void Maze_Time_End(uint32_t *max_us, uint32_t start) {
	uint32_t us = (DWT->CYCCNT - start) / (SystemCoreClock / 1000000u);
	if (us > *max_us) {
		*max_us = us;
	}
}
#else
static uint32_t Maze_Time_Start(void) {
	return 0;
}
static void Maze_Time_End(uint32_t *max_us, uint32_t start) {
	(void) max_us;
	(void) start;
}
#endif

int G_Gool_X; /////////////////////////////////////////
int G_Gool_Y; /////////////////////////////////////////

uint8_t G_MAZE_Explored[MAZE_SIZE][MAZE_SIZE];
int i, j;

uint16_t G_Step_Map[MAZE_SIZE][MAZE_SIZE];
int Step_N = 0;

int Maze_Wall_Left;
int Maze_Wall_Right;
int Maze_Wall_Flont;

uint32_t G_Maze_Row[MAZE_SIZE + 1];
uint32_t G_Maze_Column[MAZE_SIZE + 1];

uint32_t Maze_Row_Look[MAZE_SIZE + 1];
uint32_t Maze_Column_Look[MAZE_SIZE + 1];

uint32_t G_Maze_Row_Save[MAZE_SIZE + 1];
uint32_t G_Maze_Column_Save[MAZE_SIZE + 1];

uint32_t Maze_Row_Look_Save[MAZE_SIZE + 1];
uint32_t Maze_Column_Look_Save[MAZE_SIZE + 1];

int G_Robot_Direction = 4; //0~3　前　右　後ろ　左　%4
int G_Robot_Lastaction = 5; //0~3 直進　右　後ろ　左

/* Uターン(行き止まり)が実行された直後にMove.c側からセットされるフラグ。
 * Dijkstra誘導 未知壁探索(main.c)が「Uターンした時だけ経路を再計算する」
 * ための検出に使う。呼び出し側が使い終わったら自分で0に戻すこと */
int G_Just_UTurned = 0;
/* 迷路に書く前壁を、壁を読む側(motor.c の Motor_Sula_before)が決めて渡す。-1 なら今までどおり
 * G_Wall_data[0] と [3] の両方(Wallsensor.c の Wall_Front_For_Map、2026-10-19) */
int G_Maze_Front_Override = -1;
#define MAZE_FRONT_WALL() ((G_Maze_Front_Override >= 0) ? G_Maze_Front_Override : (G_Wall_data[0] == 1 && G_Wall_data[3] == 1))

uint16_t G_Maze_Flont;
uint16_t G_Maze_Back;
uint16_t G_Maze_Left;
uint16_t G_Maze_Right;

int G_Robot_MAZE_X = 0;
int G_Robot_MAZE_Y = 0;

int16_t G_Short_Pass[MAX_STEP];
int16_t G_Short_Pass_CP[MAX_STEP];
int16_t G_Short_Pass_NANAME[MAX_STEP];

int16_t G_Known_Pass[MAX_STEP];
int16_t Known_Pass_CP[MAX_STEP];
int16_t Known_Pass_NANAME[MAX_STEP];

int NANAME_Flag = 0;
int Known_Flag = 0;

int ALL_MODE = 0;

/* Dijkstraバックトレースが通過した経路上のセル列(Dijkstra誘導 未知壁探索用)。
 * インデックスkの区間はセル(G_Dijk_Path_X[k],G_Dijk_Path_Y[k])に入るために
 * 通過した壁(G_Dijk_Path_WallIsRow/I/J[k])に対応する。その壁が現時点で
 * 未知かどうかはMaze_Unknown_Wall_Scan()がその都度ライブ判定するので、
 * ここではスナップショットは持たない */
int16_t G_Dijk_Path_X[MAX_STEP];
int16_t G_Dijk_Path_Y[MAX_STEP];
uint8_t G_Dijk_Path_WallIsRow[MAX_STEP];
int16_t G_Dijk_Path_WallI[MAX_STEP];
int16_t G_Dijk_Path_WallJ[MAX_STEP];
int G_Dijk_Path_Len = 0;

/* Maze_Unknown_Wall_Scan()が見つけた「経路上で最初に見つかった未知壁」の
 * 手前側セル座標(Maze_Gool_Setting()のmode==2で目標地点として使う)と、
 * その壁自体の識別子(Maze_Unknown_Wall_Still_Unknown()での再確認用) */
/* 見に行く途中で目標の区画に行けなくなったら1(Maze_Step_Calculate、2026-10-27)。
 * Maze_Unknown_Target_ModeSet() で0に戻る */
int G_Unknown_Target_Lost = 0;
int G_Unknown_Target_X = 0;
int G_Unknown_Target_Y = 0;
uint8_t G_Unknown_Wall_IsRow = 0;
int16_t G_Unknown_Wall_I = 0;
int16_t G_Unknown_Wall_J = 0;

/* 行き止まり潰し(dead-end filling)。
 * 「確認済みの壁だけで出入口が1つ以下」のマスは、入った辺と同じ辺からしか
 * 出られないので、そのマス自身が始点/終点でない限り最短経路上に絶対に乗らない。
 * 全面探索でわざわざ寄り道する価値がないので、目標マスの候補から外す。
 *
 * 安全側に倒すための作り:
 *  - G_Maze_Row/G_Maze_Column(実測の壁マップ)は一切書き換えない。
 *    マップの保存/表示/ダイクストラは今まで通り真の壁データだけを見る。
 *  - この情報は歩数マップBFSの通行判定には使わない。使うのは
 *    Maze_Gool_Setting()の「未探索マスを目標にするか」の判断だけなので、
 *    誤って閉じても「そのマスを探索しない」だけで、経路が塞がったり
 *    スタートに帰れなくなったりはしない。
 *  - 毎回ゼロから作り直す(ステートレス)ので、誤った閉鎖が累積しないし、
 *    ゴール座標が探索用/帰還用に切り替わっても自動で追従する。
 *  - 未確認の壁は「開いている」とみなす(閉じる方向には倒さない)。*/
uint8_t G_MAZE_Closed[MAZE_SIZE][MAZE_SIZE];
/* 実機で挙動が疑わしいときに0にすれば即座に従来動作へ戻せる */
int G_DeadEnd_Fill_Enable = 1;
int g_deadend_closed_count = 0;

void pushQueue_walk(QUEUE_T *queue, unsigned short input) {
	/* データをデータの最後尾の１つ後ろに格納*/
	queue->data[queue->tail] = input;
	/* データの最後尾を１つ後ろに移動*/
	queue->tail = queue->tail + 1;
	/* 巡回シフト*/
	if (queue->tail == MAX_QUEUE_NUM)
		queue->tail = 0;
	/* スタックが満杯なら何もせず関数終了*/
	if (queue->tail == queue->head) {
		//printf("stack_full\n");return;
	}
}

unsigned short popQueue_walk(QUEUE_T *queue) {
	unsigned short ret = 0;
	/* スタックが空なら何もせずに関数終了*/
	if (queue->tail == queue->head) {
		//printf("stack_empty\n");
		return 65535;
	}
	/* データの最前列からデータを取得*/
	ret = queue->data[queue->head];
	/* データの最前列を１つ前にずらす*/
	queue->head = queue->head + 1;
	/* 巡回シフト*/
	if (queue->head == MAX_QUEUE_NUM)
		queue->head = 0;
	/* 取得したデータを返却*/
	return ret;
}

void pushStack_walk(STACK_T *stack, unsigned short input) {
	/* データをデータの最後尾の１つ後ろに格納*/
	stack->data[stack->tail] = input;
	/* データの最後尾を１つ後ろに移動*/
	stack->tail = stack->tail + 1;
	/* 巡回シフト*/
	if (stack->tail == MAX_QUEUE_NUM)
		stack->tail = 0;
	/* スタックが満杯なら何もせず関数終了*/
	if (stack->tail == stack->head) {
		//printf("stack_full\n");return;
	}
}

unsigned short popStack_walk(STACK_T *stack) {
	unsigned short ret = 0;
	/* スタックが空なら何もせずに関数終了*/
	if (stack->tail == stack->head) {
		//printf("stack_empty\n");
		return 65535;
	}
	/* データの最前列からデータを取得*/
	ret = stack->data[stack->head];
	/* データの最前列を１つ前にずらす*/
	stack->head = stack->head + 1;
	/* 巡回シフト*/
	if (stack->head == MAX_QUEUE_NUM)
		stack->head = 0;
	/* 取得したデータを返却*/
	return ret;
}

/* シミュレータ(sim/sim_main.c)が見る数 */
#ifdef SIM_DEBUG
int g_queue_node_push_count = 0;
int g_queue_node_max_occupancy = 0;
int g_queue_node_overflow_count = 0;
int g_dijkstra_backtrace_hops = 0;
int g_bfs_outer_hops = 0;
#endif

void Maze_Initialization() {
	for (i = 0; i < MAZE_SIZE; i++) {
		for (j = 0; j < MAZE_SIZE; j++) {
			G_MAZE_Explored[i][j] = 0;
			G_MAZE_Closed[i][j] = 0;
		}
	}
	g_deadend_closed_count = 0;
	G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
	for (i = 0; i < MAZE_SIZE; i++) {
		for (j = 0; j < MAZE_SIZE; j++) {
			G_Step_Map[i][j] = MAX_STEP;
		}
	}
	G_Step_Map[G_Gool_X][G_Gool_Y] = 0;
	for (i = 0; i < MAZE_SIZE + 1; i++) {
		G_Maze_Row[i] = 0;
		G_Maze_Column[i] = 0;
		Maze_Row_Look[i] = 0;
		Maze_Column_Look[i] = 0;
	}

	for (i = 0; i < MAX_STEP; i++) {
		G_Short_Pass[i] = 0;
	}

	/* 外周を全部壁にする。MAZE_SIZEビット分を1にしたマスク。
	 * MAZE_SIZE==32のとき (1u<<32) はUBになるので64bit側でシフトしてから落とす */
	uint32_t all_walls = (uint32_t) (((uint64_t) 1 << MAZE_SIZE) - 1);

	G_Maze_Row[0] = all_walls;
	G_Maze_Row[MAZE_SIZE] = all_walls;
	G_Maze_Column[0] = all_walls;
	G_Maze_Column[1] = 1;
	G_Maze_Column[MAZE_SIZE] = all_walls;

	Maze_Row_Look[0] = all_walls;
	Maze_Row_Look[1] = 1;
	Maze_Row_Look[MAZE_SIZE] = all_walls;
	Maze_Column_Look[0] = all_walls;
	Maze_Column_Look[1] = 1;
	Maze_Column_Look[MAZE_SIZE] = all_walls;
}

void Maze_Wall_Search(int X, int Y, int Direction) {
	Maze_Wall_Flont = 0;
	Maze_Wall_Left = 0;
	Maze_Wall_Right = 0;
	if (Direction == 0) {
		if ((G_Maze_Row[Y + 1] & (1u << X)) == (1u << X)) {
			Maze_Wall_Flont = 1;
		}
		if ((G_Maze_Column[X] & (1u << Y)) == (1u << Y)) {
			Maze_Wall_Left = 1;
		}
		if ((G_Maze_Column[X + 1] & (1u << Y)) == (1u << Y)) {
			Maze_Wall_Right = 1;
		}
	} else if (Direction == 1) {
		if ((G_Maze_Column[X + 1] & (1u << Y)) == (1u << Y)) {
			Maze_Wall_Flont = 1;
		}
		if ((G_Maze_Row[Y + 1] & (1u << X)) == (1u << X)) {
			Maze_Wall_Left = 1;
		}
		if ((G_Maze_Row[Y] & (1u << X)) == (1u << X)) {
			Maze_Wall_Right = 1;
		}
	} else if (Direction == 2) {
		if ((G_Maze_Row[Y] & (1u << X)) == (1u << X)) {
			Maze_Wall_Flont = 1;
		}
		if ((G_Maze_Column[X + 1] & (1u << Y)) == (1u << Y)) {
			Maze_Wall_Left = 1;
		}
		if ((G_Maze_Column[X] & (1u << Y)) == (1u << Y)) {
			Maze_Wall_Right = 1;
		}
	} else if (Direction == 3) {
		if ((G_Maze_Column[X] & (1u << Y)) == (1u << Y)) {
			Maze_Wall_Flont = 1;
		}
		if ((G_Maze_Row[Y] & (1u << X)) == (1u << X)) {
			Maze_Wall_Left = 1;
		}
		if ((G_Maze_Row[Y + 1] & (1u << X)) == (1u << X)) {
			Maze_Wall_Right = 1;
		}
	}

}

static void Maze_Wall_Update_Body(void) {
	if (G_Robot_Direction % 4 == 0) { //////////////////////////////////////////////////////////北に移動
		G_Robot_MAZE_X += 0;
		G_Robot_MAZE_Y += 1;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Column[G_Robot_MAZE_X] = G_Maze_Column[G_Robot_MAZE_X]
						| (1u << (G_Robot_MAZE_Y));
			}
			if (MAZE_FRONT_WALL()) {		//前壁がある//
				G_Maze_Row[G_Robot_MAZE_Y + 1] = G_Maze_Row[G_Robot_MAZE_Y + 1]
						| (1u << (G_Robot_MAZE_X));
			}
			if (G_Wall_data[2] == 1) {					//右壁がある
				G_Maze_Column[G_Robot_MAZE_X + 1] = G_Maze_Column[G_Robot_MAZE_X
						+ 1] | (1u << (G_Robot_MAZE_Y));
			}
			Maze_Column_Look[G_Robot_MAZE_X] = Maze_Column_Look[G_Robot_MAZE_X]
					| (1u << (G_Robot_MAZE_Y));
			Maze_Row_Look[G_Robot_MAZE_Y + 1] =
					Maze_Row_Look[G_Robot_MAZE_Y + 1] | (1u << (G_Robot_MAZE_X));
			Maze_Column_Look[G_Robot_MAZE_X + 1] =
					Maze_Column_Look[G_Robot_MAZE_X + 1]
							| (1u << (G_Robot_MAZE_Y));

			Maze_DeadEnd_Fill();		//行き止まりマスを目標候補から外す
			Maze_Step_Calculate();			//歩数マップ更新
		}
		G_Maze_Flont = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y + 1];
		G_Maze_Back = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y - 1];
		G_Maze_Left = G_Step_Map[G_Robot_MAZE_X - 1][G_Robot_MAZE_Y];
		G_Maze_Right = G_Step_Map[G_Robot_MAZE_X + 1][G_Robot_MAZE_Y];

		if ((G_Maze_Row[G_Robot_MAZE_Y + 1] & (1u << G_Robot_MAZE_X))
				== (1u << G_Robot_MAZE_X)) {
			G_Maze_Flont = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y] & (1u << G_Robot_MAZE_X))
				== (1u << G_Robot_MAZE_X)) {
			G_Maze_Back = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X] & (1u << G_Robot_MAZE_Y))
				== (1u << G_Robot_MAZE_Y)) {
			G_Maze_Left = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X + 1] & (1u << G_Robot_MAZE_Y))
				== (1u << G_Robot_MAZE_Y)) {
			G_Maze_Right = MAX_STEP;
		}

	} else if (G_Robot_Direction % 4 == 1) {//////////////////////////////////////////////////////////////東に移動
		G_Robot_MAZE_X += 1;
		G_Robot_MAZE_Y += 0;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Row[G_Robot_MAZE_Y + 1] = G_Maze_Row[G_Robot_MAZE_Y + 1]
						| (1u << (G_Robot_MAZE_X));
			}
			if (MAZE_FRONT_WALL()) {		//前壁がある
				G_Maze_Column[G_Robot_MAZE_X + 1] = G_Maze_Column[G_Robot_MAZE_X
						+ 1] | (1u << (G_Robot_MAZE_Y));
			}
			if (G_Wall_data[2] == 1) {					//右壁がある
				G_Maze_Row[G_Robot_MAZE_Y] = G_Maze_Row[G_Robot_MAZE_Y]
						| (1u << (G_Robot_MAZE_X));
			}
			Maze_Row_Look[G_Robot_MAZE_Y + 1] =
					Maze_Row_Look[G_Robot_MAZE_Y + 1] | (1u << (G_Robot_MAZE_X));
			Maze_Column_Look[G_Robot_MAZE_X + 1] =
					Maze_Column_Look[G_Robot_MAZE_X + 1]
							| (1u << (G_Robot_MAZE_Y));
			Maze_Row_Look[G_Robot_MAZE_Y] = Maze_Row_Look[G_Robot_MAZE_Y]
					| (1u << (G_Robot_MAZE_X));

			Maze_DeadEnd_Fill();		//行き止まりマスを目標候補から外す
			Maze_Step_Calculate();			//歩数マップ更新
		}
		G_Maze_Flont = G_Step_Map[G_Robot_MAZE_X + 1][G_Robot_MAZE_Y];
		G_Maze_Back = G_Step_Map[G_Robot_MAZE_X - 1][G_Robot_MAZE_Y];
		G_Maze_Left = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y + 1];
		G_Maze_Right = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y - 1];

		if ((G_Maze_Column[G_Robot_MAZE_X + 1] & (1u << G_Robot_MAZE_Y))
				== (1u << G_Robot_MAZE_Y)) {
			G_Maze_Flont = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X] & (1u << G_Robot_MAZE_Y))
				== (1u << G_Robot_MAZE_Y)) {
			G_Maze_Back = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y + 1] & (1u << G_Robot_MAZE_X))
				== (1u << G_Robot_MAZE_X)) {
			G_Maze_Left = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y] & (1u << G_Robot_MAZE_X))
				== (1u << G_Robot_MAZE_X)) {
			G_Maze_Right = MAX_STEP;
		}
	} else if (G_Robot_Direction % 4 == 2) {//////////////////////////////////////////////南に移動
		G_Robot_MAZE_X += 0;
		G_Robot_MAZE_Y += -1;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Column[G_Robot_MAZE_X + 1] = G_Maze_Column[G_Robot_MAZE_X
						+ 1] | (1u << (G_Robot_MAZE_Y));
			}
			if (MAZE_FRONT_WALL()) {		//前壁がある
				G_Maze_Row[G_Robot_MAZE_Y] = G_Maze_Row[G_Robot_MAZE_Y]
						| (1u << (G_Robot_MAZE_X));
			}
			if (G_Wall_data[2] == 1) {					//右壁がある
				G_Maze_Column[G_Robot_MAZE_X] = G_Maze_Column[G_Robot_MAZE_X]
						| (1u << (G_Robot_MAZE_Y));
			}
			Maze_Column_Look[G_Robot_MAZE_X + 1] =
					Maze_Column_Look[G_Robot_MAZE_X + 1]
							| (1u << (G_Robot_MAZE_Y));
			Maze_Row_Look[G_Robot_MAZE_Y] = Maze_Row_Look[G_Robot_MAZE_Y]
					| (1u << (G_Robot_MAZE_X));
			Maze_Column_Look[G_Robot_MAZE_X] = Maze_Column_Look[G_Robot_MAZE_X]
					| (1u << (G_Robot_MAZE_Y));

			Maze_DeadEnd_Fill();		//行き止まりマスを目標候補から外す
			Maze_Step_Calculate();			//歩数マップ更新

		}
		G_Maze_Flont = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y - 1];
		G_Maze_Back = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y + 1];
		G_Maze_Left = G_Step_Map[G_Robot_MAZE_X + 1][G_Robot_MAZE_Y];
		G_Maze_Right = G_Step_Map[G_Robot_MAZE_X - 1][G_Robot_MAZE_Y];

		if ((G_Maze_Row[G_Robot_MAZE_Y] & (1u << G_Robot_MAZE_X))
				== (1u << G_Robot_MAZE_X)) {
			G_Maze_Flont = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y + 1] & (1u << G_Robot_MAZE_X))
				== (1u << G_Robot_MAZE_X)) {
			G_Maze_Back = MAX_STEP;
		}

		if ((G_Maze_Column[G_Robot_MAZE_X + 1] & (1u << G_Robot_MAZE_Y))
				== (1u << G_Robot_MAZE_Y)) {
			G_Maze_Left = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X] & (1u << G_Robot_MAZE_Y))
				== (1u << G_Robot_MAZE_Y)) {
			G_Maze_Right = MAX_STEP;
		}

	} else {			///////////////////////////////////////////////////西に移動
		G_Robot_MAZE_X += -1;
		G_Robot_MAZE_Y += 0;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Row[G_Robot_MAZE_Y] = G_Maze_Row[G_Robot_MAZE_Y]
						| (1u << (G_Robot_MAZE_X));
			}
			if (MAZE_FRONT_WALL()) {		//前壁がある
				G_Maze_Column[G_Robot_MAZE_X] = G_Maze_Column[G_Robot_MAZE_X]
						| (1u << (G_Robot_MAZE_Y));
			}
			if (G_Wall_data[2] == 1) {					//右壁がある
				G_Maze_Row[G_Robot_MAZE_Y + 1] = G_Maze_Row[G_Robot_MAZE_Y + 1]
						| (1u << (G_Robot_MAZE_X));
			}
			Maze_Row_Look[G_Robot_MAZE_Y] = Maze_Row_Look[G_Robot_MAZE_Y]
					| (1u << (G_Robot_MAZE_X));
			Maze_Column_Look[G_Robot_MAZE_X] = Maze_Column_Look[G_Robot_MAZE_X]
					| (1u << (G_Robot_MAZE_Y));
			Maze_Row_Look[G_Robot_MAZE_Y + 1] =
					Maze_Row_Look[G_Robot_MAZE_Y + 1] | (1u << (G_Robot_MAZE_X));

			Maze_DeadEnd_Fill();		//行き止まりマスを目標候補から外す
			Maze_Step_Calculate();			//歩数マップ更新
		}
		G_Maze_Flont = G_Step_Map[G_Robot_MAZE_X - 1][G_Robot_MAZE_Y];
		G_Maze_Back = G_Step_Map[G_Robot_MAZE_X + 1][G_Robot_MAZE_Y];
		G_Maze_Left = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y - 1];
		G_Maze_Right = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y + 1];

		if ((G_Maze_Column[G_Robot_MAZE_X + 1] & (1u << G_Robot_MAZE_Y))
				== (1u << G_Robot_MAZE_Y)) {
			G_Maze_Back = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X] & (1u << G_Robot_MAZE_Y))
				== (1u << G_Robot_MAZE_Y)) {
			G_Maze_Flont = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y + 1] & (1u << G_Robot_MAZE_X))
				== (1u << G_Robot_MAZE_X)) {
			G_Maze_Right = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y] & (1u << G_Robot_MAZE_X))
				== (1u << G_Robot_MAZE_X)) {
			G_Maze_Left = MAX_STEP;
		}
	}
}

/* 1: 既知区間加速で、直線にはさまれた小回りを大回り(90/180)に置き換える。0: 置き換えない(2026-10-09〜) */
#define KNOWN_PASS_BIG_TURN 0

static void Known_Pass_Generation_Body(void) {
	int N = 0;

	Known_Flag = 1;
	//Maze_Step_Calculate();
	for (int i = 0; i < MAX_STEP; i++) {
		G_Known_Pass[i] = 0;
		Known_Pass_CP[i] = 0;
	}
	G_Known_Pass[0] = -1;
	while (1) {
		if (N >= MAX_STEP - 2) {	//配列オーバーフロー防止(異常系フェイルセーフ)
			break;
		}
		int Known_X = G_Robot_MAZE_X;
		int Known_Y = G_Robot_MAZE_Y;
		if (G_Robot_Direction % 4 == 0) {
			Known_X += 0;
			Known_Y += 1;
		} else if (G_Robot_Direction % 4 == 1) {
			Known_X += 1;
			Known_Y += 0;
		} else if (G_Robot_Direction % 4 == 2) {
			Known_X += 0;
			Known_Y += -1;
		} else {
			Known_X += -1;
			Known_Y += 0;
		}
		if (G_MAZE_Explored[Known_X][Known_Y] == 0) {
			break;
		}
		Maze_Wall_Update();
		if ((G_Maze_Flont <= G_Maze_Left) && (G_Maze_Flont <= G_Maze_Right)	//前進
				&& (G_Maze_Flont <= G_Maze_Back)) {
			if (G_Known_Pass[N] < 0) {
				N++;
			}
			G_Known_Pass[N] += 2;
			G_Robot_Direction += 0;
		} else if ((G_Maze_Left <= G_Maze_Right)	//左折
		&& (G_Maze_Left <= G_Maze_Back)) {
			N++;
			G_Known_Pass[N] = -2;
			G_Robot_Direction += 3;
		} else if (G_Maze_Right <= G_Maze_Back) {	//	右折
			N++;
			G_Known_Pass[N] = -3;
			G_Robot_Direction += 1;
		} else {			//Uターン(行き止まり) ※-1は先頭センチネルと衝突するため-8を使用
			N++;
			G_Known_Pass[N] = -8;
			G_Robot_Direction += 2;
		}

	}

	for (i = 0; G_Known_Pass[i] != 0; i++) {
		Known_Pass_CP[i] = G_Known_Pass[i];
	}
	/* ループは実際の経路長term_idxで打ち切る。終端より先(先読み)は関数の最初で
	 * 0にしてあるので、最後の曲がりは小回りのまま残る。
	 * 最短走行(Shortest_Pass_Compression)と違い、ここでは先頭を1にしたり
	 * 終端に1を置いたりしてはいけない。既知区間は区画の境界の手前から始まり、
	 * 未探索の区画の手前で終わるので、先頭に1(半区画の直進)を置くと毎回
	 * 90mm余分に進み、終端に1を置くと最後の小回りが大回りになって半区画先で
	 * 終わってしまう(どちらも実機で直線が伸びる症状になった) */
	int term_idx = i;
	/* 既知区間の大回り(-4〜-7)への置き換え。2026-10-09 から使わない(KNOWN_PASS_BIG_TURN 0):
	 * 曲がりはすべて探索と同じ小回りスラローム(-2/-3)で走る */
	for (i = 0; KNOWN_PASS_BIG_TURN && i < term_idx; i++) {
		if (Known_Pass_CP[i] == -2) {
			if (Known_Pass_CP[i - 1] > 0) {
				if (Known_Pass_CP[i + 1] > 0) { //左９０おおまわり
					Known_Pass_CP[i - 1] -= 1;
					Known_Pass_CP[i] = -4;
					Known_Pass_CP[i + 1] -= 1;
				} else if ((Known_Pass_CP[i + 1] == -2)
						&& (Known_Pass_CP[i + 2] > 0)) { //左１８０おおまわり
					Known_Pass_CP[i - 1] -= 1;
					Known_Pass_CP[i] = -5;
					Known_Pass_CP[i + 1] = -1;
					Known_Pass_CP[i + 2] -= 1;
				}
				if (Known_Pass_CP[i - 1] == 0) {
					Known_Pass_CP[i - 1] = -1;
				}
				if (Known_Pass_CP[i + 1] == 0) {
					Known_Pass_CP[i + 1] = -1;
				}
			}
		} else if (Known_Pass_CP[i] == -3) {
			if (Known_Pass_CP[i - 1] > 0) {
				if (Known_Pass_CP[i + 1] > 0) { //右９０おおまわり
					Known_Pass_CP[i - 1] -= 1;
					Known_Pass_CP[i] = -6;
					Known_Pass_CP[i + 1] -= 1;
				} else if ((Known_Pass_CP[i + 1] == -3)
						&& (Known_Pass_CP[i + 2] > 0)) { //右１８０おおまわり
					Known_Pass_CP[i - 1] -= 1;
					Known_Pass_CP[i] = -7;
					Known_Pass_CP[i + 1] = -1;
					Known_Pass_CP[i + 2] -= 1;
				}
				if (Known_Pass_CP[i - 1] == 0) {
					Known_Pass_CP[i - 1] = -1;
				}
				if (Known_Pass_CP[i + 1] == 0) {
					Known_Pass_CP[i + 1] = -1;
				}
			}
		}

	}

	G_Known_Pass[term_idx] = 0;
	Known_Pass_CP[term_idx] = 0;

	Known_Pass_Compression_NANAME();

	for (i = 0; G_Known_Pass[i] != 0; i++) {
		G_Known_Pass[i] = Known_Pass_CP[i];
	}
	Known_Flag = 0;
}

/* 既知区間加速バーストの斜め(NANAME)圧縮パス。Known_Pass_CP[]の大廻り圧縮結果
 * (-4/-5/-6/-7)を Known_Pass_NANAME[]へコピーし、さらに斜め入り(-51〜-54)・
 * 斜め出(-61〜-64)・V90(-65/-66)・斜め直線(-50連結)へ圧縮する。
 * Shortest_Pass_Compression_NANAME()と同一ロジックの既知区間版。
 * ※Move.c側のモーター実行(Robot_Maze_Pass_Action())はまだ実機未調整のため
 *   Known_Pass_NANAME[]を消費しない(有効化するにはMove.cの対応ブロックの
 *   コメントアウトを外し、実行ループの参照先をKnown_Pass_NANAMEへ切り替える必要がある)。
 */
/* 斜めの区間を、斜めに入る前の形に戻す。from: 斜め入りで1つ減らした直進の位置、from_val: 減らす前のその値、
 * to: 戻す範囲の終わり(含まない)。直進は、前の斜め出で減らしてあることがあるので Known_Pass_CP ではなく減らす前の値に戻す */
static void Known_Naname_Revert(int from, int from_val, int to) {
	Known_Pass_NANAME[from] = from_val;
	for (int k = from + 1; k < to; k++) {
		Known_Pass_NANAME[k] = Known_Pass_CP[k];
	}
}

void Known_Pass_Compression_NANAME() {
	NANAME_Flag = 0;
	int naname_from = 0;	//今の斜めの区間を戻すときの始まり(斜め入りの前の直進)
	int naname_from_val = 0;	//その直進の、斜め入りで減らす前の値

	for (i = 0; Known_Pass_CP[i] != 0; i++) {
		Known_Pass_NANAME[i] = Known_Pass_CP[i];
	}
	/* 終端の先を0にしておく(下の先読み [i+1]・[i+2] が前回の残りを読まないように)。
	 * 2026-10-21: 以前は終端に1を置いて「直進が続く」ことにしていたが、既知区間は未探索の区画の手前の
	 * 境目で終わり、その先に直進はない。最後の曲がりが斜め出になると、その半区画を先読みの1から取ってしまい、
	 * 半区画手前で終わっていた。斜め出は、後ろに本当の直進(> 0)があるときだけにする */
	int naname_term_idx = i;
	for (int k = naname_term_idx; k < naname_term_idx + 3 && k < MAX_STEP; k++) {
		Known_Pass_NANAME[k] = 0;
	}
	for (i = 0; i < naname_term_idx; i++) {
		if ((Known_Pass_NANAME[i] == -8) && (NANAME_Flag == 1)) {
			/* 斜めのまま行き止まりのUターンには入れない(Uターンは直進の向きで、区画の境目から)。
			 * 斜め出できなかった斜めの区間は、小回りに戻す */
			Known_Naname_Revert(naname_from, naname_from_val, i);
			NANAME_Flag = 0;
			continue;
		}
		if ((Known_Pass_NANAME[i] == -2) || (Known_Pass_NANAME[i] == -3)) {
			if (Known_Pass_NANAME[i - 1] > 0) { //斜め入り
				/* 既知区間は大回りにしない(KNOWN_PASS_BIG_TURN 0)ので、直進 → 同じ向きに2回 → 直進(大回り180の形)が残っている。
				 * 135 は、そのあとも曲がり続けるときだけ(最短では先に大回り180になるので、この形は来ない) */
				int turn2 = (Known_Pass_NANAME[i + 2] == -2) || (Known_Pass_NANAME[i + 2] == -3);
				if (Known_Pass_NANAME[i] == -2) { //左

					if (Known_Pass_NANAME[i + 1] == -3) {
						//入り４５
						naname_from = i - 1;
						naname_from_val = Known_Pass_NANAME[i - 1];
						Known_Pass_NANAME[i - 1] -= 1;
						Known_Pass_NANAME[i] = -51;
						NANAME_Flag = 1;
						Pass_zero_act();
					} else if ((Known_Pass_NANAME[i + 1] == -2) && turn2) {
						//入り135
						naname_from = i - 1;
						naname_from_val = Known_Pass_NANAME[i - 1];
						Known_Pass_NANAME[i - 1] -= 1;
						Known_Pass_NANAME[i] = -52;
						Known_Pass_NANAME[i + 1] = -1;
						NANAME_Flag = 1;
						Pass_zero_act();
					}
				} else if (Known_Pass_NANAME[i] == -3) { //右

					if (Known_Pass_NANAME[i + 1] == -2) {
						//入り４５
						naname_from = i - 1;
						naname_from_val = Known_Pass_NANAME[i - 1];
						Known_Pass_NANAME[i - 1] -= 1;
						Known_Pass_NANAME[i] = -53;
						NANAME_Flag = 1;
						Pass_zero_act();
					} else if ((Known_Pass_NANAME[i + 1] == -3) && turn2) {
						//入り135
						naname_from = i - 1;
						naname_from_val = Known_Pass_NANAME[i - 1];
						Known_Pass_NANAME[i - 1] -= 1;
						Known_Pass_NANAME[i] = -54;
						Known_Pass_NANAME[i + 1] = -1;
						NANAME_Flag = 1;
						Pass_zero_act();
					}
				}
			} else if ((Known_Pass_NANAME[i + 1] > 0) && (NANAME_Flag == 1)) { //斜め出 45
				if (Known_Pass_NANAME[i] == -3) {
					//右４５
					Known_Pass_NANAME[i] = -63;
					Known_Pass_NANAME[i + 1] -= 1;
					Pass_zero_act();
				} else if (Known_Pass_NANAME[i] == -2) {
					//左４５
					Known_Pass_NANAME[i] = -61;
					Known_Pass_NANAME[i + 1] -= 1;
					Pass_zero_act();
				}
				NANAME_Flag = 0;
			} else if ((Known_Pass_NANAME[i + 2] > 0) && (NANAME_Flag == 1)) {
				if (Known_Pass_NANAME[i] == Known_Pass_NANAME[i + 1]) {
					if (Known_Pass_NANAME[i] == -3) {
						//右135
						Known_Pass_NANAME[i] = -64;
						Known_Pass_NANAME[i + 1] = -1;
						Known_Pass_NANAME[i + 2] -= 1;
						Pass_zero_act();
					} else if (Known_Pass_NANAME[i] == -2) {
						//左135
						Known_Pass_NANAME[i] = -62;
						Known_Pass_NANAME[i + 1] = -1;
						Known_Pass_NANAME[i + 2] -= 1;
						Pass_zero_act();
					}
					NANAME_Flag = 0;
				} else {
					if ((Known_Pass_NANAME[i] == -2) || (Known_Pass_NANAME[i] == -3)) {
						if (Known_Pass_NANAME[i] == Known_Pass_NANAME[i + 1]) {
							if (Known_Pass_NANAME[i] == -2) {
								Known_Pass_NANAME[i] = -65;
								Known_Pass_NANAME[i + 1] = -1;
							} else {
								Known_Pass_NANAME[i] = -66;
								Known_Pass_NANAME[i + 1] = -1;
							}

						} else {
							Known_Pass_NANAME[i] = -50;
						}
					}
				}
			} else if (NANAME_Flag == 1) {
				if ((Known_Pass_NANAME[i] == -2) || (Known_Pass_NANAME[i] == -3)) {
					if (Known_Pass_NANAME[i] == Known_Pass_NANAME[i + 1]) {
						if (Known_Pass_NANAME[i] == -2) {
							Known_Pass_NANAME[i] = -65;
							Known_Pass_NANAME[i + 1] = -1;
						} else {
							Known_Pass_NANAME[i] = -66;
							Known_Pass_NANAME[i + 1] = -1;
						}

					} else {
						Known_Pass_NANAME[i] = -50;
					}
				}
			}
		}
	}
	if (NANAME_Flag == 1) {
		/* 斜めのまま既知区間が終わった: 未探索の区画の手前の境目で、直進の向きで終わらないといけないので、小回りに戻す */
		Known_Naname_Revert(naname_from, naname_from_val, naname_term_idx);
		NANAME_Flag = 0;
	}
	for (i = 0; i < naname_term_idx; i++) {
		if (Known_Pass_NANAME[i] == -50) {
			for (int j = 1; Known_Pass_NANAME[i + j] == -50; j++) {
				Known_Pass_NANAME[i + j] = -1;
				Known_Pass_NANAME[i] -= 50;
			}
		}
	}

	/* 正式な0終端(Pass_zero_act が終端の0を-1にしていることがある) */
	Known_Pass_NANAME[naname_term_idx] = 0;
}

void Maze_Unkown_ALL_ModeSet() {
	ALL_MODE = 1;
}

int Maze_All_MODE_Check() {
	return ALL_MODE;
}

/* 連鎖で潰してはいけないマス。
 * ここを守っている限り、閉じたマスが最短経路に乗ることはない
 * (経路の途中のマスは必ず入口と出口の2辺が要るので、出入口1つのマスは
 *  始点/終点にしかなり得ない)。*/
static int Maze_Cell_IsProtected(int x, int y) {
	if ((x == 0) && (y == 0)) {
		return 1;			//スタートは初期状態で3枚壁。潰すと帰還できない
	}
	if ((x == G_Gool_X) && (y == G_Gool_Y)) {
		return 1;			//現在の目標(探索時は中央、帰還時はスタート)
	}
	/* 本番ゴールは2×2区画。Maze_Shortest_Calculation()が
	 * (MAZE_GOOL_X..+1, MAZE_GOOL_Y..+1)の4マスを終点として扱うので、
	 * 区画全体を守る。帰還モード中(G_Gool=スタート)も守っておく */
	if ((x >= MAZE_GOOL_X) && (x <= MAZE_GOOL_X + 1) && (y >= MAZE_GOOL_Y)
			&& (y <= MAZE_GOOL_Y + 1)) {
		return 1;
	}
	return 0;
}

/* (x,y)から出られる辺の数。
 * 「壁が確認済み」または「その先が閉鎖済み(=経路に乗り得ない)」の辺は数えない。
 * 未確認の壁は開いているとみなすので、判定は必ず安全側に倒れる。*/
static int Maze_Cell_OpenDegree(int x, int y) {
	int deg = 0;
	if (((G_Maze_Row[y + 1] & (1u << x)) == 0) && (y + 1 < MAZE_SIZE)
			&& (G_MAZE_Closed[x][y + 1] == 0)) {
		deg++;						//北
	}
	if (((G_Maze_Column[x + 1] & (1u << y)) == 0) && (x + 1 < MAZE_SIZE)
			&& (G_MAZE_Closed[x + 1][y] == 0)) {
		deg++;						//東
	}
	if (((G_Maze_Row[y] & (1u << x)) == 0) && (y - 1 >= 0)
			&& (G_MAZE_Closed[x][y - 1] == 0)) {
		deg++;						//南
	}
	if (((G_Maze_Column[x] & (1u << y)) == 0) && (x - 1 >= 0)
			&& (G_MAZE_Closed[x - 1][y] == 0)) {
		deg++;						//西
	}
	return deg;
}

/* 行き止まりマスを連鎖的に閉じる。毎回ゼロから作り直す。
 * 必ず「壁を記録した後」かつ「Maze_Wall_fill()より前」に呼ぶこと。
 * Maze_Wall_fill()は未確認の壁を全部1に潰すので、その後に呼ぶと
 * ほぼ全マスが行き止まり判定になってしまう。*/
void Maze_DeadEnd_Fill() {
	/* 1マスにつき高々1回しか積まないのでMAX_STEP(=総マス数)で足りる。
	 * スタックオーバーフロー回避のためstatic(.bss)に置く */
	static uint8_t stack_x[MAX_STEP];
	static uint8_t stack_y[MAX_STEP];
	static const int dx[4] = { 0, 1, 0, -1 };
	static const int dy[4] = { 1, 0, -1, 0 };
	int sp = 0;

	for (int x = 0; x < MAZE_SIZE; x++) {
		for (int y = 0; y < MAZE_SIZE; y++) {
			G_MAZE_Closed[x][y] = 0;
		}
	}
	g_deadend_closed_count = 0;

	if (G_DeadEnd_Fill_Enable == 0) {
		return;
	}

	/* 第1段: 確認済みの壁だけで既に行き止まりになっているマスを閉じる */
	for (int x = 0; x < MAZE_SIZE; x++) {
		for (int y = 0; y < MAZE_SIZE; y++) {
			if (Maze_Cell_IsProtected(x, y)) {
				continue;
			}
			if (Maze_Cell_OpenDegree(x, y) <= 1) {
				G_MAZE_Closed[x][y] = 1;
				g_deadend_closed_count++;
				stack_x[sp] = (uint8_t) x;
				stack_y[sp] = (uint8_t) y;
				sp++;
			}
		}
	}

	/* 第2段: 閉じたマスの隣を再評価して連鎖させる。
	 * 閉じたマスは通路として数えなくなるので、袋小路の通路が根元まで潰れる */
	while (sp > 0) {
		sp--;
		int cx = stack_x[sp];
		int cy = stack_y[sp];
		for (int d = 0; d < 4; d++) {
			int nx = cx + dx[d];
			int ny = cy + dy[d];
			if ((nx < 0) || (nx >= MAZE_SIZE) || (ny < 0) || (ny >= MAZE_SIZE)) {
				continue;
			}
			if (G_MAZE_Closed[nx][ny] != 0) {
				continue;
			}
			if (Maze_Cell_IsProtected(nx, ny)) {
				continue;
			}
			if (Maze_Cell_OpenDegree(nx, ny) <= 1) {
				G_MAZE_Closed[nx][ny] = 1;
				g_deadend_closed_count++;
				stack_x[sp] = (uint8_t) nx;
				stack_y[sp] = (uint8_t) ny;
				sp++;
			}
		}
	}
}

void Maze_Gool_Setting(int mode) {
	int N = 0;

	if (mode == 1) {
		for (int x = 0; x < MAZE_SIZE; x++) {
			for (int y = 0; y < MAZE_SIZE; y++) {
				/* 行き止まりと判明したマスは全面探索の目標にしない。
				 * 最短経路に乗り得ないので、探索を打ち切っても損しない */
				if ((G_MAZE_Explored[x][y] == 0) && (G_MAZE_Closed[x][y] == 0)) {

					if ((x == 0) && (y == 0)) {
						G_Step_Map[x][y] = 100;
						N = 0;
					} else {
						G_Step_Map[x][y] = 0;
						N++;
					}
				}
			}
		}
		if (N == 0) {
			ALL_MODE = 0;
		}
	} else if (mode == 2) {
		G_Step_Map[G_Unknown_Target_X][G_Unknown_Target_Y] = 0;
	} else {
		G_Step_Map[G_Gool_X][G_Gool_Y] = 0;
	}
}

void Maze_Unkown_ALL_ModeOFF() {
	ALL_MODE = 0;
}

/* 歩数マップ(G_Step_Map)で歩数0になっているマス全部を起点にBFSで歩数を広げる。
 * 呼ぶ前に、起点以外のマスをMAX_STEPにしておくこと */
static void Maze_Step_Propagate(void) {
	/* MAZE_SIZE=32だとdata[]がMAX_QUEUE_NUM(=マス総数)分あり、スタックに置くと
	 * スタックオーバーフローの危険があるためstatic(.bss)に置く */
	static QUEUE_T queue_x;
	static QUEUE_T queue_y;
	queue_x.head = 0;
	queue_x.tail = 0;
	queue_y.head = 0;
	queue_y.tail = 0;
	unsigned short X;
	unsigned short Y;
	Step_N = 0;
	for (i = 0; i < MAZE_SIZE; i++) {
		for (j = 0; j < MAZE_SIZE; j++) {
			if (G_Step_Map[i][j] == 0) {
				pushQueue_walk(&queue_x, i);
				pushQueue_walk(&queue_y, j);
			}
		}
	}
	while (Step_N < MAX_STEP) {
		X = popQueue_walk(&queue_x);
		Y = popQueue_walk(&queue_y);
		if ((X == 65535) || (Y == 65535)) {
			break;
		}
		if (((G_Maze_Column[X + 1] & (1u << Y)) == 0)
				&& (G_Step_Map[X + 1][Y] == MAX_STEP)) { //右
			G_Step_Map[X + 1][Y] = G_Step_Map[X][Y] + 1;
			pushQueue_walk(&queue_x, X + 1);
			pushQueue_walk(&queue_y, Y);
		}
		if (((G_Maze_Column[X] & (1u << Y)) == 0)
				&& (G_Step_Map[X - 1][Y] == MAX_STEP)) { //左
			G_Step_Map[X - 1][Y] = G_Step_Map[X][Y] + 1;
			pushQueue_walk(&queue_x, X - 1);
			pushQueue_walk(&queue_y, Y);
		}
		if (((G_Maze_Row[Y + 1] & (1u << X)) == 0)
				&& (G_Step_Map[X][Y + 1] == MAX_STEP)) { //上
			G_Step_Map[X][Y + 1] = G_Step_Map[X][Y] + 1;
			pushQueue_walk(&queue_x, X);
			pushQueue_walk(&queue_y, Y + 1);
		}
		if (((G_Maze_Row[Y] & (1u << X)) == 0)
				&& (G_Step_Map[X][Y - 1] == MAX_STEP)) { //下
			G_Step_Map[X][Y - 1] = G_Step_Map[X][Y] + 1;
			pushQueue_walk(&queue_x, X);
			pushQueue_walk(&queue_y, Y - 1);
		}
		Step_N++;
	}
}

static void Maze_Step_Calculate_Body(void) {
	for (i = 0; i < MAZE_SIZE; i++) {
		for (j = 0; j < MAZE_SIZE; j++) {
			G_Step_Map[i][j] = MAX_STEP;
		}
	}
	Maze_Gool_Setting(ALL_MODE);
	/* ALL_MODE中に未探索マスが尽きるとMaze_Gool_Setting()内でALL_MODEが
	 * OFFになるが、そのタイミングではまだ「通常モードの目標セル
	 * (G_Gool_X,Y)を歩数0にする」処理(elseブランチ)が走っていない
	 * (呼び出し時点のmode==1のまま実行されているため)。ここで補う */
	if (ALL_MODE == 0 && G_Step_Map[G_Gool_X][G_Gool_Y] != 0) {
		G_Step_Map[G_Gool_X][G_Gool_Y] = 0;
	}
	/* ALL_MODE時はMaze_Gool_Setting()が未探索マスを全部歩数0にするが、
	 * BFSはキューに積んだマスからしか伝播しないので、歩数0のマス全部を
	 * 起点としてキューに積まないと多点始点BFSにならない。
	 * G_Gool_X/Yだけを特別扱いして先に積むと、ALL_MODE中はスタート
	 * (歩数100、本来は起点ではない)がFIFOの先頭に来て先に展開されてしまい、
	 * 近くのマスを誤って「スタートからの距離」で埋めてしまう。
	 * 歩数が実際に0のマスだけを均等に積むことでこれを避ける(mode==2の
	 * G_Unknown_Target_X/Yもこのスキャンで自動的に拾われる) */
	Maze_Step_Propagate();

	/* 大会ルール上、壁で完全に閉じられていて到達不可能なマスが
	 * 存在し得る。ALL_MODE中にそういうマスだけが未探索として残ると
	 * Maze_Gool_Setting()のN(未探索マス数)が0にならずALL_MODEが
	 * 永遠にOFFにならない。その場合、今いる場所からはどの未探索マスにも
	 * 到達できず歩数マップが更新されない(=自分のマスがMAX_STEPのまま)
	 * ので、それを検知したら全面探索を打ち切ってスタートへ戻るモードに
	 * 切り替え、歩数マップを作り直す */
	if (ALL_MODE == 1
			&& G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y] == MAX_STEP) {
		ALL_MODE = 0;
		Maze_Step_Calculate();
	}
	/* ダイクストラ誘導の未知壁探索(ALL_MODE == 2)で、見に行く途中でほかの壁が見つかり、
	 * 目標の区画に行けなくなった(今いる区画が MAX_STEP。2026-10-27)。このままだと前後左右が
	 * 全部 MAX_STEP になり、前に壁があっても前進を選んでしまう(シミュレータでは迷路の外へ出た)。
	 * いったんゴール(G_Gool_X,Y)を目標にして動けるようにし、G_Unknown_Target_Lost を立てる
	 * (main.c がその区画で止まって経路を計算し直す) */
	if (ALL_MODE == 2
			&& G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y] == MAX_STEP) {
		G_Unknown_Target_Lost = 1;
		ALL_MODE = 0;
		Maze_Step_Calculate();
	}
}

void Maze_Shortest_Calculation() {
	int Short_MAZE_X = 0;
	int Short_MAZE_Y = 0;
	int N = 0;

	/* 歩数マップをゴール2x2区画の4マス全部を起点に作り直す。
	 * Maze_Step_Calculate()はゴール(G_Gool_X,Y)の1マスだけを起点にするので
	 * (探索はそのマスに着くまで走る作りのため)、そのままたどると
	 * 「そのマスへの最短」になり、2x2区画のほかのマスから入る方が近い迷路で
	 * 1〜2マス遠回りになる(test/test_shortpath.c で検出)。
	 * 下のループは隣の歩数が小さい方へ進み、歩数0のマスに入ったら止まる */
	for (int x = 0; x < MAZE_SIZE; x++) {
		for (int y = 0; y < MAZE_SIZE; y++) {
			G_Step_Map[x][y] = MAX_STEP;
		}
	}
	G_Step_Map[G_Gool_X][G_Gool_Y] = 0;
	G_Step_Map[G_Gool_X + 1][G_Gool_Y] = 0;
	G_Step_Map[G_Gool_X][G_Gool_Y + 1] = 0;
	G_Step_Map[G_Gool_X + 1][G_Gool_Y + 1] = 0;
	Maze_Step_Propagate();

	for (i = 0; i < MAX_STEP; i++) {
		G_Short_Pass[i] = 0;
	}
	/* スタートからゴールへ行けない(未確認の壁を壁とみなすと塞がる、探索が
	 * 途中で止まった等)と、下のループは歩数の小さい隣が見つからず
	 * 永久に回り続けて固まる。その場合は経路を空にして返す */
	if (G_Step_Map[0][0] == MAX_STEP) {
		return;
	}
	G_Short_Pass[0] = -1;
#ifdef SIM_DEBUG
	g_bfs_outer_hops = 0;
#endif
	while ((G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0)
			|| (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0)
			|| (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0)
			|| (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0)) {
#ifdef SIM_DEBUG
		g_bfs_outer_hops++;
		if (g_bfs_outer_hops > 2000) {
			printf("[dbg] Maze_Shortest_Calculation aborting after 2000 iterations\n");
			break;
		}
#endif
		if (G_Robot_Direction % 4 == 0) { //北向き
			Short_MAZE_X += 0;
			Short_MAZE_Y += 1;

			if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y] == 0) {
				break;
			}
			Maze_Wall_Search(Short_MAZE_X, Short_MAZE_Y, 0);
			while (1) {
				if (Maze_Wall_Flont == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y + 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //前
						if (G_Short_Pass[N] < 0) {
							N++;
						}
						G_Short_Pass[N] += 2;
						G_Robot_Direction += 0;
						break;
					}
				}
				if (Maze_Wall_Left == 0) {
					if (G_Step_Map[Short_MAZE_X - 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //左
						N++;
						G_Short_Pass[N] = -2;
						G_Robot_Direction += 3;
						break;
					}
				}
				if (Maze_Wall_Right == 0) {
					if (G_Step_Map[Short_MAZE_X + 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //右
						N++;
						G_Short_Pass[N] = -3;
						G_Robot_Direction += 1;
						break;
					}
				}
			}

		} else if (G_Robot_Direction % 4 == 1) { //東向き
			Short_MAZE_X += 1;
			Short_MAZE_Y += 0;
			if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y] == 0) {
				break;
			}
			Maze_Wall_Search(Short_MAZE_X, Short_MAZE_Y, 1);
			while (1) {
				if (Maze_Wall_Flont == 0) {
					if (G_Step_Map[Short_MAZE_X + 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //前
						if (G_Short_Pass[N] < 0) {
							N++;
						}
						G_Short_Pass[N] += 2;
						G_Robot_Direction += 0;
						break;
					}
				}
				if (Maze_Wall_Left == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y + 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //左
						N++;
						G_Short_Pass[N] = -2;
						G_Robot_Direction += 3;
						break;
					}
				}
				if (Maze_Wall_Right == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y - 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //右
						N++;
						G_Short_Pass[N] = -3;
						G_Robot_Direction += 1;
						break;
					}
				}

			}
		} else if (G_Robot_Direction % 4 == 2) { //南
			Short_MAZE_X += 0;
			Short_MAZE_Y += -1;
			if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y] == 0) {
				break;
			}
			Maze_Wall_Search(Short_MAZE_X, Short_MAZE_Y, 2);
			while (1) {
				if (Maze_Wall_Flont == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y - 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //前
						if (G_Short_Pass[N] < 0) {
							N++;
						}
						G_Short_Pass[N] += 2;
						G_Robot_Direction += 0;
						break;
					}
				}
				if (Maze_Wall_Left == 0) {
					if (G_Step_Map[Short_MAZE_X + 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //左
						N++;
						G_Short_Pass[N] = -2;
						G_Robot_Direction += 3;
						break;
					}
				}
				if (Maze_Wall_Right == 0) {
					if (G_Step_Map[Short_MAZE_X - 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //右
						N++;
						G_Short_Pass[N] = -3;
						G_Robot_Direction += 1;
						break;
					}
				}
			}
		} else if (G_Robot_Direction % 4 == 3) { //西
			Short_MAZE_X += -1;
			Short_MAZE_Y += 0;
			if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y] == 0) {
				break;
			}
			Maze_Wall_Search(Short_MAZE_X, Short_MAZE_Y, 3);
			while (1) {
				if (Maze_Wall_Flont == 0) {
					if (G_Step_Map[Short_MAZE_X - 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //前
						if (G_Short_Pass[N] < 0) {
							N++;
						}
						G_Short_Pass[N] += 2;
						G_Robot_Direction += 0;
						break;
					}
				}
				if (Maze_Wall_Left == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y - 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //左
						N++;
						G_Short_Pass[N] = -2;
						G_Robot_Direction += 3;
						break;
					}
				}
				if (Maze_Wall_Right == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y + 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //右
						N++;
						G_Short_Pass[N] = -3;
						G_Robot_Direction += 1;
						break;
					}
				}
			}
		}
		G_MAZE_Explored[Short_MAZE_X][Short_MAZE_Y] = 1;
	}
}

/* ---- ダイクストラ(2026-10-27: 最短走行の時間) ----------------------------------
 * ノードは壁の位置(境界)。Row型 = 横壁 G_Maze_Row[y] の bit x、Column型 = 縦壁 G_Maze_Column[x] の bit y。
 * 境界は2つのマスに挟まれているので、ノードを「どちら向きに通過しているか」(sense)で2つに分ける。
 * sense=1 は Row型なら北向き、Column型なら東向き。
 * ノード番号 = ((isRow * (MAZE_SIZE+1) + x) * (MAZE_SIZE+1) + y) * 2 + sense
 * ノードから進める「ホップ」は、今の向きの先のマスを通る3本(直進1本 + 斜め2本、dijk_edges)。
 * 直進のホップはマスをまっすぐ、斜めのホップはマスの中で小回り1回(G_Short_Pass の -2/-3)。
 *
 * 状態 = ノード × ここから進む直線の向き(dijk_edges の3本のどれか)。
 *   縦横の状態: 大回り・斜め出が終わったところ。ここから縦横の直線(k マス)とターンを1つ進む
 *   斜めの状態: 斜め入り・V90 が終わったところ。ここから斜めの直線(m ホップ)とターンを1つ進む
 * 1回の移動のコストは、Shortest_Pass_Compression(_NANAME) がそこに作る命令(直線・ターン)の時間の合計
 * (tools/route_time.py と同じ数え方)。なので、経路のコストは命令列 G_Short_Pass_NANAME の時間になる。
 * ターンとホップの並び(Shortest_Pass_Compression(_NANAME) と同じ):
 *   大回り90    縦横k, 斜め1(45°)                   → 縦横(90°)
 *   大回り180   縦横k, 斜め1(45°), 斜め1(さらに90°)   → 縦横(180°)
 *   斜め入り45  縦横k, 斜め(45°)                    → 斜め
 *   斜め入り135 縦横k, 斜め1(45°), 斜め(さらに90°)    → 斜め
 *   斜め出45    斜めm                               → 縦横(45°)
 *   斜め出135   斜めm, 斜め1(90°)                   → 縦横(135°)
 *   V90         斜めm, 斜め(90°)                    → 斜め
 * 縦横の直線は前後のターンが半区画ずつ使うので 2k-2 半区画、斜めの直線は、斜めに入ったターン(の最後の
 * ホップ)と出るターン(の最初のホップ)が1ホップずつ使うので m-1 区間。
 * ゴールの2x2区画の外側の境界を内向きに通ったら経路の終わり。最後が直線なら境目から、
 * ターンならゴールの区画の中心から止まる。
 * 以前(2026-10-14〜26)は状態をノードだけにして、ゴールから同じ向きの区間ごとに広げていた。
 * ターンごとの時間を数えるには、ここから進む向きを状態に持たないといけないので、状態を3倍にした。
 * 1つ前の状態は持たず、経路をたどるときに「コストが合う1つ前の状態」を探す(RAM を減らすため) */
#define DIJK_W (MAZE_SIZE + 1)
#define DIJK_NODE_NUM (2 * DIJK_W * DIJK_W * 2)
#define DIJK_STATE_NUM (DIJK_NODE_NUM * 3)
#define DIJK_HOP_MAX (MAZE_SIZE * MAZE_SIZE + 4)	//経路のホップの数の上限(1ホップで1マス)

#if DIJK_MAXCOST > 65535
typedef uint32_t dijk_cost_t;
#else
typedef uint16_t dijk_cost_t;
#endif

static dijk_cost_t dijk_cost[DIJK_STATE_NUM];

/* SPFAのキュー。dijk_inq で同じ状態は同時に1回しか積まれないので状態数で足りる */
static uint16_t dijk_queue[DIJK_STATE_NUM];
static uint8_t dijk_inq[(DIJK_STATE_NUM + 7) / 8];
static int dijk_q_head, dijk_q_count;

static void dijk_push(int s) {
	if (dijk_q_count >= DIJK_STATE_NUM) {	//dijk_inq がある限り起きない
#ifdef SIM_DEBUG
		g_queue_node_overflow_count++;
#endif
		return;
	}
	dijk_queue[(dijk_q_head + dijk_q_count) % DIJK_STATE_NUM] = (uint16_t) s;
	dijk_q_count++;
	dijk_inq[s >> 3] |= (uint8_t) (1u << (s & 7));
#ifdef SIM_DEBUG
	g_queue_node_push_count++;
	if (dijk_q_count > g_queue_node_max_occupancy)
		g_queue_node_max_occupancy = dijk_q_count;
#endif
}

static int dijk_pop(void) {
	if (dijk_q_count == 0) {
		return -1;
	}
	int s = dijk_queue[dijk_q_head];
	dijk_q_head = (dijk_q_head + 1) % DIJK_STATE_NUM;
	dijk_q_count--;
	dijk_inq[s >> 3] &= (uint8_t) ~(1u << (s & 7));
	return s;
}

/* ノードから進める3本の辺。進む先は今の向きの先にあるマスを通るものだけ(直進1本+斜め2本)。
 * [0] がいつも直進。{dir, isRow, dx, dy, sense}。dir は 0=N,1=NE,...,7=NW(時計回り) */
typedef struct {
	int8_t dir, isRow, dx, dy, sense;
} DIJK_EDGE_T;

static const DIJK_EDGE_T dijk_edges[2][2][3] = {
	{ /* Column型 */
		{ /* 西向き: マス(x-1,y)を通る */
			{ 6, 0, -1, 0, 0 },		//W  縦壁(x-1,y)
			{ 7, 1, -1, 1, 1 },		//NW 横壁(x-1,y+1)
			{ 5, 1, -1, 0, 0 },		//SW 横壁(x-1,y)
		},
		{ /* 東向き: マス(x,y)を通る */
			{ 2, 0, 1, 0, 1 },		//E  縦壁(x+1,y)
			{ 1, 1, 0, 1, 1 },		//NE 横壁(x,y+1)
			{ 3, 1, 0, 0, 0 },		//SE 横壁(x,y)
		},
	},
	{ /* Row型 */
		{ /* 南向き: マス(x,y-1)を通る */
			{ 4, 1, 0, -1, 0 },		//S  横壁(x,y-1)
			{ 5, 0, 0, -1, 0 },		//SW 縦壁(x,y-1)
			{ 3, 0, 1, -1, 1 },		//SE 縦壁(x+1,y-1)
		},
		{ /* 北向き: マス(x,y)を通る */
			{ 0, 1, 0, 1, 1 },		//N  横壁(x,y+1)
			{ 7, 0, 0, 0, 0 },		//NW 縦壁(x,y)
			{ 1, 0, 1, 0, 1 },		//NE 縦壁(x+1,y)
		},
	},
};

typedef struct {
	int8_t isRow, x, y, sense;
} DIJK_NODE_T;

static int dijk_is_wall(int isRow, int x, int y) {
	if (isRow) {
		return (G_Maze_Row[y] & (1u << x)) != 0;
	}
	return (G_Maze_Column[x] & (1u << y)) != 0;
}

static int dijk_state_index(const DIJK_NODE_T *n, int k) {
	return (((n->isRow * DIJK_W + n->x) * DIJK_W + n->y) * 2 + n->sense) * 3 + k;
}

static void dijk_state_node(int s, DIJK_NODE_T *n, int *k) {
	*k = s % 3;
	s /= 3;
	n->sense = (int8_t) (s & 1);
	s >>= 1;
	n->y = (int8_t) (s % DIJK_W);
	s /= DIJK_W;
	n->x = (int8_t) (s % DIJK_W);
	n->isRow = (int8_t) (s / DIJK_W);
}

/* ノード n の3本の辺のうち向き dir のもの(0〜2)。なければ -1 */
static int dijk_edge_k(const DIJK_NODE_T *n, int dir) {
	for (int k = 0; k < 3; k++) {
		if (dijk_edges[n->isRow][n->sense][k].dir == dir) {
			return k;
		}
	}
	return -1;
}

/* ノード n から向き dir に1ホップ。進めれば(その向きの辺があって、行き先が迷路の中で壁でなければ)1 */
static int dijk_hop(const DIJK_NODE_T *n, int dir, DIJK_NODE_T *out) {
	int k = dijk_edge_k(n, dir);
	if (k < 0) {
		return 0;
	}
	const DIJK_EDGE_T *e = &dijk_edges[n->isRow][n->sense][k];
	int nx = n->x + e->dx, ny = n->y + e->dy;
	/* 外周は全部壁なので、外周より外へ出るホップは壁の手前で止まる。念のため範囲も確認する */
	if (nx < 0 || nx > MAZE_SIZE || ny < 0 || ny > MAZE_SIZE || dijk_is_wall(e->isRow, nx, ny)) {
		return 0;
	}
	out->isRow = e->isRow;
	out->x = (int8_t) nx;
	out->y = (int8_t) ny;
	out->sense = e->sense;
	return 1;
}

/* ノード n に向き dir のホップで来たときの、1つ前のノード(壁は見ない)。なければ 0 */
static int dijk_hop_back(const DIJK_NODE_T *n, int dir, DIJK_NODE_T *out) {
	DIJK_NODE_T r = *n;
	r.sense = (int8_t) (1 - n->sense);
	int k = dijk_edge_k(&r, (dir + 4) & 7);
	if (k < 0) {
		return 0;
	}
	const DIJK_EDGE_T *e = &dijk_edges[r.isRow][r.sense][k];
	int nx = n->x + e->dx, ny = n->y + e->dy;
	if (nx < 0 || nx > MAZE_SIZE || ny < 0 || ny > MAZE_SIZE) {
		return 0;
	}
	out->isRow = e->isRow;
	out->x = (int8_t) nx;
	out->y = (int8_t) ny;
	out->sense = (int8_t) (1 - e->sense);
	return 1;
}

/* ノード n を通るとゴールの2x2区画に入るか(外側の境界を内向きに通る) */
static int dijk_is_goal_entry(const DIJK_NODE_T *n) {
	int gx = G_Gool_X, gy = G_Gool_Y;
	if (n->isRow) {	//横壁 y: マス(x,y-1)とマス(x,y)のあいだ
		if (n->x < gx || n->x > gx + 1) {
			return 0;
		}
		return (n->sense == 1 && n->y == gy) || (n->sense == 0 && n->y == gy + 2);
	}
	if (n->y < gy || n->y > gy + 1) {	//縦壁 x: マス(x-1,y)とマス(x,y)のあいだ
		return 0;
	}
	return (n->sense == 1 && n->x == gx) || (n->sense == 0 && n->x == gx + 2);
}

/* 表示用: そのノードの状態のうち一番小さいコスト。
 * 最短走行の時間(G_Dijk_Time_Cost)のときは3桁に収まるように10ms単位にする */
static int dijk_node_cost(int isRow, int x, int y) {
	DIJK_NODE_T n = { (int8_t) isRow, (int8_t) x, (int8_t) y, 0 };
	int c = DIJK_MAXCOST;
	for (int sense = 0; sense < 2; sense++) {
		n.sense = (int8_t) sense;
		for (int k = 0; k < 3; k++) {
			int v = dijk_cost[dijk_state_index(&n, k)];
			if (v < c) {
				c = v;
			}
		}
	}
	if (G_Dijk_Time_Cost) {
		c = (c >= DIJK_MAXCOST) ? 999 : c / (DIJK_COST_PER_S / 100);
	}
	return c;
}

/* ---- コスト(最短走行の時間) ---- */
int G_Dijk_Time_Cost = 1;
DijkCost G_Dijk_Cost;
int G_Dijk_Best_Cost = -1;
static int dijk_cost_ready = 0;

#define DIJK_PI 3.14159265f
#define DIJK_D2R (DIJK_PI / 180.0f)
#define DIJK_DIAG 127.3f	//斜め1区間[mm]

/* cos型の加減速で、距離 L[mm] を速度 v → (最高 vmax) → v で走る時間[s]。加速 ac、減速 dc */
static float dijk_run_time(float L, float v, float vmax, float ac, float dc) {
	if (L <= 0) {
		return 0;
	}
	float r = DIJK_PI / 4.0f * (1.0f / ac + 1.0f / dc);	//v から vp まで上げて戻る距離 = r*(vp^2 - v^2)
	float vp = vmax;
	if (r * (vp * vp - v * v) > L) {	//最高速度まで届かない
		vp = sqrtf(v * v + L / r);
	}
	float rest = L - r * (vp * vp - v * v);
	if (rest < 0) {
		rest = 0;
	}
	return DIJK_PI / 2.0f * (vp - v) * (1.0f / ac + 1.0f / dc) + rest / vp;
}

/* ターンの理想の始まりの点から終わりの点までの時間[s](tools/route_time.py の turn_time と同じ)。
 * 速度 v で、角速度は cos 型の加速 → 等角速度 → cos 型の減速。前後の直進は理想の線に乗る長さ */
static float dijk_turn_time(int kind, float v, const Turn2000_Param *t) {
	/* 理想の始まりの点と向き、終わりの点、曲がる角度(左ターン。区画 180mm、中心が (0,0)) */
	static const float geom[T2000_NUM][6] = {
		[T2000_BIG90]  = { 0, 0, 90, -180, 180, 90 },
		[T2000_BIG180] = { 0, 0, 90, -180, 0, 180 },
		[T2000_IN45]   = { 0, 0, 90, -90, 180, 45 },
		[T2000_IN135]  = { 0, 0, 90, -180, 90, 135 },
		[T2000_OUT45]  = { 90, 180, 225, 0, 0, 45 },
		[T2000_OUT135] = { 180, 90, 135, 0, 0, 135 },
		[T2000_V90]    = { 90, 0, 45, 90, 180, 90 },
	};
	const float *g = geom[kind];
	float ang = fabsf(t->ang), wmax = t->w, wac = t->w_ac;
	float x13 = DIJK_PI / (2 * wac) * wmax * wmax;	//加速と減速で回る角度
	if (x13 > ang) {
		wmax = sqrtf(2 * wac * ang / DIJK_PI);
		x13 = ang;
	}
	float t1 = DIJK_PI * wmax / (2 * wac);
	float tc = (ang - x13) / wmax;
	float T = 2 * t1 + tc;
	/* 回るあいだに進む位置(始まりの向きで前・左)。64 段で、0.5ms 単位に丸めた時間は 4000 段(route_time.py)と同じ */
	const int n = 64;
	float h = T / n, ax = 0, ay = 0, th = 0;
	for (int i = 0; i < n; i++) {
		float tt = (i + 0.5f) * h, w;
		if (tt < t1) {
			w = wmax / 2 * (1 - cosf(DIJK_PI * tt / t1));
		} else if (tt < t1 + tc) {
			w = wmax;
		} else {
			w = wmax / 2 * (1 + cosf(DIJK_PI * (tt - t1 - tc) / t1));
		}
		float thm = th + w * DIJK_D2R * h / 2;
		ax += v * cosf(thm) * h;
		ay += v * sinf(thm) * h;
		th += w * DIJK_D2R * h;
	}
	float a = g[2] * DIJK_D2R, dx = g[3] - g[0], dy = g[4] - g[1];
	float lx = dx * cosf(a) + dy * sinf(a), ly = -dx * sinf(a) + dy * cosf(a);	//始まりの向きで見た終わりの点
	float ae = g[5] * DIJK_D2R, s = sinf(ae), st, end;
	if (fabsf(s) < 1e-6f) {	//180°: 前後は平行なので、前の直進を0にして後ろで合わせる
		st = 0;
		end = (ax - lx > 0) ? ax - lx : 0;
	} else {
		end = (ly - ay) / s;
		st = (lx - ax) - end * cosf(ae);
	}
	return ((st > 0 ? st : 0) + (end > 0 ? end : 0)) / v + T;
}

static uint16_t dijk_units(float sec) {
	float c = sec * DIJK_COST_PER_S + 0.5f;
	return (uint16_t) ((c < 0) ? 0 : (c > 60000) ? 60000 : c);
}

/* 作った表を覚えておく(探索は止まるたびに G_Short_Run_Set の組を順に使うので、同じ値なら作り直さない) */
#define DIJK_CACHE_NUM 2
typedef struct {
	int v, vmax, ac, ac_naname;
	float turn[T2000_NUM][3];
} DijkRunKey;
static DijkRunKey dijk_cache_key[DIJK_CACHE_NUM];
static DijkCost dijk_cache_cost[DIJK_CACHE_NUM];
static int dijk_cache_n = 0, dijk_cache_next = 0;

static void dijk_run_key(const ShortRunParam *p, DijkRunKey *k) {
	memset(k, 0, sizeof(*k));
	k->v = p->v;
	k->vmax = p->vmax;
	k->ac = p->ac;
	k->ac_naname = p->ac_naname;
	for (int t = 0; t < T2000_NUM; t++) {
		k->turn[t][0] = p->turn[t].ang;
		k->turn[t][1] = p->turn[t].w;
		k->turn[t][2] = p->turn[t].w_ac;
	}
}

void Maze_Dijkstra_Set_Run(const ShortRunParam *p) {
	DijkRunKey key;
	dijk_run_key(p, &key);
	for (int k = 0; k < dijk_cache_n; k++) {
		if (memcmp(&key, &dijk_cache_key[k], sizeof(key)) == 0) {
			G_Dijk_Cost = dijk_cache_cost[k];
			dijk_cost_ready = 1;
			return;
		}
	}
	float v = p->v, vmax = (p->vmax > p->v) ? p->vmax : p->v, ac = p->ac;
	float acn = (p->ac_naname > 0) ? p->ac_naname : p->ac - 5000;
	DijkCost *c = &G_Dijk_Cost;
	/* 直線: 縦横は加速 ac・減速 2*ac(Motor_trapezoid_Asymmetric_PID)、斜めは加速・減速とも acn(Motor_NANAME_PID) */
	for (int k = 0; k < DIJK_TH_NUM; k++) {
		c->th[k] = dijk_units(dijk_run_time(90.0f * k, v, vmax, ac, 2 * ac));
	}
	for (int k = 0; k < DIJK_TD_NUM; k++) {
		c->td[k] = dijk_units(dijk_run_time(DIJK_DIAG * k, v, vmax, acn, acn));
	}
	for (int k = 0; k < T2000_NUM; k++) {
		c->turn[k] = dijk_units(dijk_turn_time(k, v, &p->turn[k]));
	}
	/* 走り出し: 最初が直線なら区画の境目までの114mmで 0 から v、最初がターンなら 10mm で v(Short_Start) */
	c->start_s = dijk_units(2 * 114.0f / v);
	c->start_t = dijk_units(DIJK_PI * v / (2 * 70000.0f));
	/* 止まる: v から 30000 で減速して、境目から270mm、区画の中心から180mm */
	float dec = DIJK_PI * v * v / (4 * 30000.0f), tdec = DIJK_PI * v / (2 * 30000.0f);
	c->stop_b = dijk_units((270.0f - dec) / v + tdec);
	c->stop_c = dijk_units((180.0f - dec) / v + tdec);
	dijk_cost_ready = 1;
	dijk_cache_key[dijk_cache_next] = key;
	dijk_cache_cost[dijk_cache_next] = *c;
	dijk_cache_next = (dijk_cache_next + 1) % DIJK_CACHE_NUM;
	if (dijk_cache_n < DIJK_CACHE_NUM) {
		dijk_cache_n++;
	}
}

#define DIJK_GOAL T2000_NUM	//縦横の直線のままゴールに入る
/* 以前のコスト(G_Dijk_Time_Cost = 0)で、ターンが使う斜めのホップのうち新しく向きを変えるもの(DIAG_COST)の数 */
static const uint8_t dijk_old_turn[T2000_NUM] = { 1, 2, 1, 2, 0, 1, 1 };

/* 縦横の状態から、縦横 k マス + ターン t(DIJK_GOAL なら直線のままゴール)。start: スタートの状態から */
static int dijk_cost_o(int start, int k, int t) {
	const DijkCost *c = &G_Dijk_Cost;
	if (G_Dijk_Time_Cost) {
		if (t == DIJK_GOAL) {
			return (start ? c->start_s + c->th[2 * k] : c->th[2 * k - 1]) + c->stop_b;
		}
		if (start) {
			return ((k == 0) ? c->start_t : c->start_s + c->th[2 * k - 1]) + c->turn[t];
		}
		return c->th[2 * k - 2] + c->turn[t];
	}
	/* 以前のコスト: 同じ向きに続く区間は 最初の1歩(縦横 ST_COST・斜め DIAG_COST) + CON_COST*(k-1)。
	 * ゴールに入る縦横の直線は全部 CON_COST(以前と同じ) */
	if (t == DIJK_GOAL) {
		return CON_COST * k;
	}
	return ((k == 0) ? 0 : ST_COST + CON_COST * (k - 1)) + DIAG_COST * dijk_old_turn[t];
}

/* 斜めの状態から、斜め m ホップ + ターン t(斜め出45・斜め出135・V90) */
static int dijk_cost_d(int m, int t) {
	if (G_Dijk_Time_Cost) {
		return G_Dijk_Cost.td[m - 1] + G_Dijk_Cost.turn[t];
	}
	return CON_COST * m + DIAG_COST * dijk_old_turn[t];
}

/* ターンでゴールに入ったときに足す時間(区画の中心から止まる) */
static int dijk_cost_stop_c(void) {
	return G_Dijk_Time_Cost ? G_Dijk_Cost.stop_c : 0;
}

/* 一番速いゴールまでの経路の、最後の移動(状態 s から直線 len + ターン t、t の最初の斜めのホップの向き dir1) */
static int dijk_best, dijk_best_s, dijk_best_len, dijk_best_t, dijk_best_dir1;

static void dijk_goal(int cost, int s, int len, int t, int dir1) {
	if (cost < dijk_best) {
		dijk_best = cost;
		dijk_best_s = s;
		dijk_best_len = len;
		dijk_best_t = t;
		dijk_best_dir1 = dir1;
	}
}

static void dijk_relax(const DIJK_NODE_T *n, int k, int cost) {
	if (k < 0 || cost >= DIJK_MAXCOST) {
		return;
	}
	int s = dijk_state_index(n, k);
	if (dijk_cost[s] > cost) {
		dijk_cost[s] = (dijk_cost_t) cost;
		if (!(dijk_inq[s >> 3] & (1u << (s & 7)))) {
			dijk_push(s);
		}
	}
}

/* 縦横の状態 s(ノード p、向き d)から、縦横の直線 k マス + 入りのターン(大回り90・180、斜め入り45・135) */
static void dijk_expand_o(int s, const DIJK_NODE_T *p, int d, int c, int start) {
	DIJK_NODE_T r = *p, a, b, nx;
	for (int k = 0;; k++) {
		if (k >= 1 || start) {	//スタート以外は、前のターンが半区画使うので1マス以上
			for (int side = 0; side < 2; side++) {
				int d1 = (d + (side ? 1 : 7)) & 7;	//45°(右 / 左)
				if (!dijk_hop(&r, d1, &a)) {
					continue;
				}
				if (dijk_is_goal_entry(&a)) {
					dijk_goal(c + dijk_cost_o(start, k, T2000_BIG90) + dijk_cost_stop_c(), s, k, T2000_BIG90, d1);
					continue;
				}
				dijk_relax(&a, 0, c + dijk_cost_o(start, k, T2000_BIG90));
				dijk_relax(&a, dijk_edge_k(&a, d1), c + dijk_cost_o(start, k, T2000_IN45));
				int d2 = (d1 + (side ? 2 : 6)) & 7;	//さらに90°
				if (!dijk_hop(&a, d2, &b)) {
					continue;
				}
				if (dijk_is_goal_entry(&b)) {
					dijk_goal(c + dijk_cost_o(start, k, T2000_BIG180) + dijk_cost_stop_c(), s, k, T2000_BIG180, d1);
					continue;
				}
				dijk_relax(&b, 0, c + dijk_cost_o(start, k, T2000_BIG180));
				dijk_relax(&b, dijk_edge_k(&b, d2), c + dijk_cost_o(start, k, T2000_IN135));
			}
		}
		if (!dijk_hop(&r, d, &nx)) {
			break;
		}
		if (dijk_is_goal_entry(&nx)) {
			dijk_goal(c + dijk_cost_o(start, k + 1, DIJK_GOAL), s, k + 1, DIJK_GOAL, -1);
			break;
		}
		r = nx;
	}
}

/* ノード n で、向き d でないほうの斜め */
static int dijk_other_diag(const DIJK_NODE_T *n, int d) {
	int d1 = dijk_edges[n->isRow][n->sense][1].dir;
	return (d1 == d) ? dijk_edges[n->isRow][n->sense][2].dir : d1;
}

/* 斜めの状態 s(ノード p、向き d)から、斜めの直線 m ホップ + 出るターン(斜め出45・135、V90) */
static void dijk_expand_d(int s, const DIJK_NODE_T *p, int d, int c) {
	DIJK_NODE_T r = *p, b, nx;
	for (int m = 1;; m++) {
		if (!dijk_hop(&r, d, &nx)) {
			break;
		}
		if (dijk_is_goal_entry(&nx)) {
			dijk_goal(c + dijk_cost_d(m, T2000_OUT45) + dijk_cost_stop_c(), s, m, T2000_OUT45, -1);
			break;
		}
		r = nx;
		dijk_relax(&r, 0, c + dijk_cost_d(m, T2000_OUT45));
		int d2 = dijk_other_diag(&r, d);	//90°
		if (dijk_hop(&r, d2, &b)) {
			if (dijk_is_goal_entry(&b)) {
				dijk_goal(c + dijk_cost_d(m, T2000_OUT135) + dijk_cost_stop_c(), s, m, T2000_OUT135, d2);
			} else {
				dijk_relax(&b, 0, c + dijk_cost_d(m, T2000_OUT135));
				dijk_relax(&b, dijk_edge_k(&b, d2), c + dijk_cost_d(m, T2000_V90));
			}
		}
	}
}

/* 経路のホップの向き(ゴール側から積む) */
static uint8_t dijk_hops[DIJK_HOP_MAX];
static int dijk_hop_n;

static void dijk_hop_put(int dir) {
	if (dijk_hop_n < DIJK_HOP_MAX) {
		dijk_hops[dijk_hop_n++] = (uint8_t) dir;
	}
}

/* 1つ前の状態を探す: x は直線の終わり(ターンの始まり)のノード。そこから直線の向き run_dir に戻りながら、
 * 「1つ前の状態のコスト + この移動のコスト + extra」が ct になる状態を探す。見つかれば状態と直線の長さ */
static int dijk_find_pred(DIJK_NODE_T x, int run_dir, int orth, int t, int ct,
		int extra, int start_s, int *pred, int *len) {
	for (int j = 0; j <= 2 * MAZE_SIZE + 2; j++) {
		int k = dijk_edge_k(&x, run_dir);
		if (k >= 0 && (j >= 1 || (orth && dijk_state_index(&x, k) == start_s))) {
			int ps = dijk_state_index(&x, k);
			if (dijk_cost[ps] < DIJK_MAXCOST) {
				int w = orth ? dijk_cost_o(ps == start_s, j, t) : dijk_cost_d(j, t);
				if ((int) dijk_cost[ps] + w + extra == ct) {
					*pred = ps;
					*len = j;
					return 1;
				}
			}
		}
		/* もう1ホップ戻る。x は直線のホップの行き先なので、壁でなく、ゴールの入口でもない */
		if (dijk_is_wall(x.isRow, x.x, x.y) || dijk_is_goal_entry(&x)) {
			return 0;
		}
		if (!dijk_hop_back(&x, run_dir, &x)) {
			return 0;
		}
	}
	return 0;
}

/* ノード n にホップの向き dir で戻る(n はホップの行き先なので、壁でなくゴールの入口でもないこと) */
static int dijk_back_mid(DIJK_NODE_T *n, int dir) {
	if (dijk_is_wall(n->isRow, n->x, n->y) || dijk_is_goal_entry(n)) {
		return 0;
	}
	return dijk_hop_back(n, dir, n);
}

/* 状態 s(コスト ct)に来た1つ前の移動を探し、そのホップを積む。戻り値: 1つ前の状態(なければ -1) */
static int dijk_back_step(int s, int start_s) {
	DIJK_NODE_T q, x;
	int cls, pred, len;
	dijk_state_node(s, &q, &cls);
	int f = dijk_edges[q.isRow][q.sense][cls].dir;
	int ct = dijk_cost[s];
	for (int side = 1; side >= -1; side -= 2) {
		if (cls == 0) {	//縦横の状態: 大回り90・180、斜め出45・135 で来た
			int h = (f - side + 8) & 7;
			/* 大回り90: 縦横(f-2s), 斜め1(f-s) */
			x = q;
			if (dijk_hop_back(&x, h, &x)
					&& dijk_find_pred(x, (f - 2 * side + 8) & 7, 1, T2000_BIG90, ct, 0, start_s, &pred, &len)) {
				dijk_hop_put(h);
				for (int j = 0; j < len; j++) dijk_hop_put((f - 2 * side + 8) & 7);
				return pred;
			}
			/* 大回り180: 縦横(f+4), 斜め1(f-3s), 斜め1(f-s) */
			x = q;
			if (dijk_hop_back(&x, h, &x) && dijk_back_mid(&x, (f - 3 * side + 8) & 7)
					&& dijk_find_pred(x, (f + 4) & 7, 1, T2000_BIG180, ct, 0, start_s, &pred, &len)) {
				dijk_hop_put(h);
				dijk_hop_put((f - 3 * side + 8) & 7);
				for (int j = 0; j < len; j++) dijk_hop_put((f + 4) & 7);
				return pred;
			}
			/* 斜め出45: 斜め(f-s) */
			if (dijk_find_pred(q, h, 0, T2000_OUT45, ct, 0, start_s, &pred, &len)) {
				for (int j = 0; j < len; j++) dijk_hop_put(h);
				return pred;
			}
			/* 斜め出135: 斜め(f-3s), 斜め1(f-s) */
			x = q;
			if (dijk_hop_back(&x, h, &x)
					&& dijk_find_pred(x, (f - 3 * side + 8) & 7, 0, T2000_OUT135, ct, 0, start_s, &pred, &len)) {
				dijk_hop_put(h);
				for (int j = 0; j < len; j++) dijk_hop_put((f - 3 * side + 8) & 7);
				return pred;
			}
		} else {	//斜めの状態: 斜め入り45・135、V90 で来た
			/* 斜め入り45: 縦横(f-s), 斜め(f) */
			x = q;
			if (dijk_hop_back(&x, f, &x)
					&& dijk_find_pred(x, (f - side + 8) & 7, 1, T2000_IN45, ct, 0, start_s, &pred, &len)) {
				dijk_hop_put(f);
				for (int j = 0; j < len; j++) dijk_hop_put((f - side + 8) & 7);
				return pred;
			}
			/* 斜め入り135: 縦横(f-3s), 斜め1(f-2s), 斜め(f) */
			x = q;
			if (dijk_hop_back(&x, f, &x) && dijk_back_mid(&x, (f - 2 * side + 8) & 7)
					&& dijk_find_pred(x, (f - 3 * side + 8) & 7, 1, T2000_IN135, ct, 0, start_s, &pred, &len)) {
				dijk_hop_put(f);
				dijk_hop_put((f - 2 * side + 8) & 7);
				for (int j = 0; j < len; j++) dijk_hop_put((f - 3 * side + 8) & 7);
				return pred;
			}
			/* V90: 斜め(f-2s), 斜め(f) */
			x = q;
			if (dijk_hop_back(&x, f, &x)
					&& dijk_find_pred(x, (f - 2 * side + 8) & 7, 0, T2000_V90, ct, 0, start_s, &pred, &len)) {
				dijk_hop_put(f);
				for (int j = 0; j < len; j++) dijk_hop_put((f - 2 * side + 8) & 7);
				return pred;
			}
		}
	}
	return -1;
}

/* 壁のノード n をホップの向き dir で通るときに入るマス */
static void dijk_cell_after(const DIJK_NODE_T *n, int dir, int16_t *cx, int16_t *cy) {
	*cx = n->x;
	*cy = n->y;
	if (n->isRow) {
		if (!(dir == 0 || dir == 1 || dir == 7)) {	//南へ
			*cy = n->y - 1;
		}
	} else if (!(dir == 1 || dir == 2 || dir == 3)) {	//西へ
		*cx = n->x - 1;
	}
}

static void dijk_path_put(const DIJK_NODE_T *n, int dir) {
	if (G_Dijk_Path_Len < MAX_STEP - 1) {
		/* 壁が今わかっているかどうかは、Maze_Unknown_Wall_Scan() が呼ばれるたびに
		 * Maze_Row_Look / Maze_Column_Look を見て決める(同じ経路で何回も探せるように) */
		dijk_cell_after(n, dir, &G_Dijk_Path_X[G_Dijk_Path_Len], &G_Dijk_Path_Y[G_Dijk_Path_Len]);
		G_Dijk_Path_WallIsRow[G_Dijk_Path_Len] = (uint8_t) n->isRow;
		G_Dijk_Path_WallI[G_Dijk_Path_Len] = n->x;
		G_Dijk_Path_WallJ[G_Dijk_Path_Len] = n->y;
		G_Dijk_Path_Len++;
	}
}

static void Maze_Dijkstra_Calculation_Body(void) {
	if (!dijk_cost_ready) {	//まだ作っていなければモード1 No.5 と同じ
		Maze_Dijkstra_Set_Run(&G_Short_Run_Set[0]);
	}
	for (int s = 0; s < DIJK_STATE_NUM; s++) {	//初期化
		dijk_cost[s] = DIJK_MAXCOST;
	}
	for (int k = 0; k < (int) sizeof(dijk_inq); k++) {
		dijk_inq[k] = 0;
	}
	dijk_q_head = 0;
	dijk_q_count = 0;
	dijk_best = DIJK_MAXCOST;
	dijk_best_s = -1;

	for (int k = 0; k < MAX_STEP; k++) {
		G_Short_Pass[k] = 0;
	}
	G_Dijk_Path_Len = 0;
	G_Dijk_Best_Cost = -1;

	/* スタート: スタートの区画の北の壁を北向きに通ったところ。区画の中心からここまでは走り出し */
	const DIJK_NODE_T start = { 1, 0, 1, 1 };
	if (dijk_is_wall(start.isRow, start.x, start.y)) {
		return;
	}
	int start_s = dijk_state_index(&start, 0);
	dijk_cost[start_s] = 0;
	dijk_push(start_s);

	while (1) {
		int s = dijk_pop();
		if (s < 0) { //END
			break;
		}
		DIJK_NODE_T p;
		int cls;
		dijk_state_node(s, &p, &cls);
		int c = dijk_cost[s];
		if (c >= dijk_best) {	//ゴールまでの一番速い経路より遅い(コストは進むほど増える)
			continue;
		}
		int d = dijk_edges[p.isRow][p.sense][cls].dir;
		if (cls == 0) {
			dijk_expand_o(s, &p, d, c, s == start_s);
		} else {
			dijk_expand_d(s, &p, d, c);
		}
	}
	/* スタートからゴールへ行けないときは経路を空にして返す(Maze_Shortest_Calculation()と同じ) */
	if (dijk_best_s < 0) {
		return;
	}
	G_Dijk_Best_Cost = dijk_best;

	/* ゴールからたどって、ホップの向きをゴール側から積む。最後の移動は覚えてある */
	dijk_hop_n = 0;
	{
		DIJK_NODE_T p;
		int cls;
		dijk_state_node(dijk_best_s, &p, &cls);
		int d = dijk_edges[p.isRow][p.sense][cls].dir, d1 = dijk_best_dir1;
		if (dijk_best_t == T2000_BIG180) {	//大回り180: 斜め1, さらに90°の斜め1
			dijk_hop_put((d1 + (((d1 - d) & 7) == 1 ? 2 : 6)) & 7);
		}
		if (d1 >= 0) {
			dijk_hop_put(d1);
		}
		for (int j = 0; j < dijk_best_len; j++) {
			dijk_hop_put(d);
		}
	}
	int s = dijk_best_s, guard = 0;
	while (s != start_s && s >= 0 && ++guard <= DIJK_HOP_MAX) {
		s = dijk_back_step(s, start_s);
	}
	if (s != start_s) {	//異常系(たどれなかった)
		G_Dijk_Best_Cost = -1;
		return;
	}
#ifdef SIM_DEBUG
	g_dijkstra_backtrace_hops = dijk_hop_n;
#endif

	/* スタートから、ホップごとに命令(直進は半区画2つ、斜めは小回り)と、通る壁を書く */
	DIJK_NODE_T n = start;
	int np = 0;
	G_Short_Pass[0] = -1;
	for (int k = dijk_hop_n - 1; k >= 0; k--) {
		int h = dijk_hops[k];
		dijk_path_put(&n, h);
		if (np >= MAX_STEP - 2) {	//配列の終端(0)を必ず残す
			break;
		}
		if ((h & 1) == 0) {	//直進(半区画2つ=1区画)
			if (G_Short_Pass[np] < 0) {
				np++;
			}
			G_Short_Pass[np] += 2;
		} else {	//小回り: 進む向きとノードの種類で左右が決まる
			int left = n.isRow ? (h == 3 || h == 7) : (h == 1 || h == 5);
			np++;
			G_Short_Pass[np] = left ? -2 : -3;
		}
		if (!dijk_hop(&n, h, &n)) {	//異常系(ホップの並びがおかしい)
			break;
		}
	}
	if (dijk_hop_n > 0) {	//ゴールの入口の壁も、通る壁として書く
		dijk_path_put(&n, dijk_hops[0]);
	}
}

/* 直近のMaze_Dijkstra_Calculation()が記録した経路(G_Dijk_Path_*)を先頭から
 * 走査し、まだ未確認の壁をG_Unknown_Target_X/Yとその壁自体の識別子
 * (G_Unknown_Wall_*)に記録する。戻り値1=見つかった、0=経路上の壁は全て確認済み、
 * 2=経路の壁のどれかが「壁あり」とわかった(見に行く途中で見つかった。経路は通れないので計算し直す。2026-10-27。
 *   以前はこれを見ずに 0 を返して探索を終え、見た範囲の最短が本当の最短より遅いことがあった)
 * (Dijkstra再計算タイミングをゴール到達直後とUターン発生時に絞るため、
 * 各壁が現時点で未知かどうかはMaze_Row_Look/Maze_Column_Lookをその都度
 * ライブで見て判定する。これにより、同じ経路データに対してこの関数を
 * 複数回呼び、既に確認済みになった壁を自然に読み飛ばして次の未知壁を
 * 見つけられる — Dijkstraを毎回再計算し直す必要がない) */
int Maze_Unknown_Wall_Scan(void) {
	for (int k = 0; k < G_Dijk_Path_Len; k++) {
		int known, wall;
		if (G_Dijk_Path_WallIsRow[k]) {
			known = (Maze_Row_Look[G_Dijk_Path_WallJ[k]] >> G_Dijk_Path_WallI[k]) & 1u;
			wall = (G_Maze_Row[G_Dijk_Path_WallJ[k]] >> G_Dijk_Path_WallI[k]) & 1u;
		} else {
			known = (Maze_Column_Look[G_Dijk_Path_WallI[k]] >> G_Dijk_Path_WallJ[k]) & 1u;
			wall = (G_Maze_Column[G_Dijk_Path_WallI[k]] >> G_Dijk_Path_WallJ[k]) & 1u;
		}
		if (known && wall) {
			return 2;
		}
	}
	for (int k = 0; k < G_Dijk_Path_Len; k++) {
		int wallUnknown;
		if (G_Dijk_Path_WallIsRow[k]) {
			wallUnknown = ((Maze_Row_Look[G_Dijk_Path_WallJ[k]]
					>> G_Dijk_Path_WallI[k]) & 1u) == 0;
		} else {
			wallUnknown = ((Maze_Column_Look[G_Dijk_Path_WallI[k]]
					>> G_Dijk_Path_WallJ[k]) & 1u) == 0;
		}
		if (wallUnknown) {
			G_Unknown_Target_X = G_Dijk_Path_X[k];
			G_Unknown_Target_Y = G_Dijk_Path_Y[k];
			G_Unknown_Wall_IsRow = G_Dijk_Path_WallIsRow[k];
			G_Unknown_Wall_I = G_Dijk_Path_WallI[k];
			G_Unknown_Wall_J = G_Dijk_Path_WallJ[k];
			return 1;
		}
	}
	return 0;
}

/* Maze_Unknown_Wall_Scan()が最後に見つけた壁が、今もまだ未確認かどうかを
 * O(1)で再確認する。目標セルへ到達済みかどうか(G_MAZE_Explored)ではなく
 * 壁のLookビットそのものを見るのは、既に立ったことのあるセルでも
 * (その壁面を通らずに入った場合は)壁が未確認のままのことがあるため */
int Maze_Unknown_Wall_Still_Unknown(void) {
	if (G_Unknown_Wall_IsRow) {
		return ((Maze_Row_Look[G_Unknown_Wall_J] >> G_Unknown_Wall_I) & 1u) == 0;
	} else {
		return ((Maze_Column_Look[G_Unknown_Wall_I] >> G_Unknown_Wall_J) & 1u)
				== 0;
	}
}

/* Maze_Unknown_Wall_Scan()が最後に見つけた壁を確認した結果、壁があったら1
 * (ダイクストラ経路がその壁でふさがれていた)。確認済みになってから呼ぶこと */
int Maze_Unknown_Wall_Is_Wall(void) {
	if (G_Unknown_Wall_IsRow) {
		return (G_Maze_Row[G_Unknown_Wall_J] >> G_Unknown_Wall_I) & 1u;
	} else {
		return (G_Maze_Column[G_Unknown_Wall_I] >> G_Unknown_Wall_J) & 1u;
	}
}

/* Maze_Unknown_Wall_Scan()が見つけた目標セルを、Maze_Gool_Setting()のmode==2で
 * BFSの目標地点として使うようセットする。Maze_Unkown_ALL_ModeSet()の兄弟関数 */
void Maze_Unknown_Target_ModeSet(int x, int y) {
	G_Unknown_Target_X = x;
	G_Unknown_Target_Y = y;
	G_Unknown_Target_Lost = 0;
	ALL_MODE = 2;
}

void Shortest_Pass_Compression() {
	for (i = 0; G_Short_Pass[i] != 0; i++) {
		G_Short_Pass_CP[i] = G_Short_Pass[i];
	}
	/* iは実経路長。末尾コーナーの先読み判定用に一時的に1を置くが、
	 * これを0終端の代わりに使ってしまうと下流(この関数の2つ目のループや
	 * Shortest_Pass_Compression_NANAME)が「!= 0の間スキャン」で終端を見失い、
	 * 前回実行時にG_Short_Pass_CP/NANAMEへ残っていた古いデータをそのまま
	 * 読み進めてしまう(短い経路の直後に前回の残骸が繋がって実行される)。
	 * ループ自体は実経路長term_idxで打ち切り、終端は最後にきちんと0へ戻す */
	int term_idx = i;
	G_Short_Pass[i] = 1;
	G_Short_Pass_CP[i] = 1;
	G_Short_Pass_CP[0] = 1;
	for (i = 0; i < term_idx; i++) {
		if (G_Short_Pass_CP[i] == -2) {
			if (G_Short_Pass_CP[i - 1] > 0) {
				if (G_Short_Pass[i + 1] > 0) { //左９０おおまわり
					G_Short_Pass_CP[i - 1] -= 1;
					G_Short_Pass_CP[i] = -4;
					G_Short_Pass_CP[i + 1] -= 1;
				} else if ((G_Short_Pass[i + 1] == -2)
						&& (G_Short_Pass[i + 2] > 0)) { //左１８０おおまわり
					G_Short_Pass_CP[i - 1] -= 1;
					G_Short_Pass_CP[i] = -5;
					G_Short_Pass_CP[i + 1] = -1;
					G_Short_Pass_CP[i + 2] -= 1;
				}
				if (G_Short_Pass_CP[i - 1] == 0) {
					G_Short_Pass_CP[i - 1] = -1;
				}
				if (G_Short_Pass_CP[i + 1] == 0) {
					G_Short_Pass_CP[i + 1] = -1;
				}
			}
		} else if (G_Short_Pass_CP[i] == -3) {
			if (G_Short_Pass_CP[i - 1] > 0) {
				if (G_Short_Pass_CP[i + 1] > 0) { //右９０おおまわり
					G_Short_Pass_CP[i - 1] -= 1;
					G_Short_Pass_CP[i] = -6;
					G_Short_Pass_CP[i + 1] -= 1;
				} else if ((G_Short_Pass[i + 1] == -3)
						&& (G_Short_Pass[i + 2] > 0)) { //右１８０おおまわり
					G_Short_Pass_CP[i - 1] -= 1;
					G_Short_Pass_CP[i] = -7;
					G_Short_Pass_CP[i + 1] = -1;
					G_Short_Pass_CP[i + 2] -= 1;
				}
				if (G_Short_Pass_CP[i - 1] == 0) {
					G_Short_Pass_CP[i - 1] = -1;
				}
				if (G_Short_Pass_CP[i + 1] == 0) {
					G_Short_Pass_CP[i + 1] = -1;
				}
			}
		}

	}

	/* 先読み用に1を置いていた終端を正式に0へ戻す。これをやらないと
	 * 次にこの配列を読む側(このすぐ下のNANAME圧縮や、次回このループを
	 * 呼んだ時)が終端を見失い、前回の残骸を読み進めてしまう */
	G_Short_Pass[term_idx] = 0;
	G_Short_Pass_CP[term_idx] = 0;
}

/* 最短走行の命令列(G_Short_Pass_NANAME)が、区画の境目で終わるか(2026-10-20)。
 * 経路はゴールに入ったところで終わる。最後が直進(-1 を除いた最後の命令が正)なら、ゴールに入る境目で終わる。
 * 最後がターン(大回り・斜め出)なら、そのターンがゴールの区画の中心で終わる。
 * 最短走行の最後に止まるまでの距離を決めるのに使う(Move.c の Short_Goal_X)。test/test_shortpath.c で確かめている。
 * 命令列は走っているあいだに書き換わる(Move.c の Short_Start)ので、走る前に呼ぶ(Move.c の Short_Pass_Check) */
int Short_Pass_Ends_At_Boundary(void) {
	int16_t last = 0;
	for (int k = 0; k < MAX_STEP && G_Short_Pass_NANAME[k] != 0; k++) {
		if (G_Short_Pass_NANAME[k] != -1) {
			last = G_Short_Pass_NANAME[k];
		}
	}
	return last > 0;
}

/* 最短走行で止まる区画と向き(2026-10-20)。命令列(G_Short_Pass_NANAME)を半区画の格子でたどり
 * (test/test_shortpath.c の run_tokens と同じ動き。スタートは (0,0) の中心で北向き)、最後に止まるまでの距離
 * (境目で終わるなら1.5区画、区画の中心で終わるなら1区画。Move.c の Short_Goal_X)を足した位置を出す。
 * x, y: 区画、dir: 止まったときの向き(0 北、1 東、2 南、3 西。最後の Robot_adjustment で向きを変える前)。
 * 戻り値: 1 求めた、0 命令列が読めない(斜めで終わる、知らない命令など)。
 * 命令列は走っているあいだに書き換わる(Move.c の Short_Start が最初の直線を半区画減らす)ので、走る前に呼ぶ
 * (Move.c の Short_Pass_Check。走ったあとに呼ぶと半区画ずれる) */
int Short_Pass_End_Cell(int *x, int *y, int *dir) {
	static const int8_t hx[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
	static const int8_t hy[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
	int px = 1, py = 1, ph = 0;	//半区画の格子(区画(x,y)の中心が(2x+1, 2y+1))と、45°ごとの向き(0 北、時計回り)
/* 今の向きで、前に f、左に s 半区画動く */
#define END_GO(f, s) do { px += (f) * hx[ph] + (s) * hx[(ph + 6) % 8]; \
		py += (f) * hy[ph] + (s) * hy[(ph + 6) % 8]; } while (0)
	for (int k = 0; k < MAX_STEP && G_Short_Pass_NANAME[k] != 0; k++) {
		int t = G_Short_Pass_NANAME[k];
		int diag = ph & 1;
		int sgn;
		if (t == -1) {
			continue;
		}
		if (t > 0) {	//直進(半区画 × t)
			if (diag) return 0;
			END_GO(t, 0);
		} else if (t <= -50 && t % 50 == 0) {	//斜めの直進
			if (!diag) return 0;
			px += (-t / 50) * hx[ph];
			py += (-t / 50) * hy[ph];
		} else if (t == -4 || t == -6) {	//大回り90(-4 左)
			if (diag) return 0;
			sgn = (t == -4) ? 1 : -1;
			END_GO(2, 2 * sgn);
			ph = (ph + 8 - 2 * sgn) % 8;
		} else if (t == -5 || t == -7) {	//大回り180
			if (diag) return 0;
			sgn = (t == -5) ? 1 : -1;
			END_GO(0, 2 * sgn);
			ph = (ph + 4) % 8;
		} else if (t == -51 || t == -53) {	//斜め入り45
			if (diag) return 0;
			sgn = (t == -51) ? 1 : -1;
			END_GO(2, sgn);
			ph = (ph + 8 - sgn) % 8;
		} else if (t == -52 || t == -54) {	//斜め入り135
			if (diag) return 0;
			sgn = (t == -52) ? 1 : -1;
			END_GO(1, 2 * sgn);
			ph = (ph + 8 - 3 * sgn) % 8;
		} else if (t == -61 || t == -63) {	//斜め出45
			if (!diag) return 0;
			sgn = (t == -61) ? 1 : -1;
			px += hx[ph];
			py += hy[ph];
			ph = (ph + 8 - sgn) % 8;
			END_GO(1, 0);
		} else if (t == -62 || t == -64) {	//斜め出135
			if (!diag) return 0;
			sgn = (t == -62) ? 1 : -1;
			px += hx[ph];
			py += hy[ph];
			ph = (ph + 8 - sgn) % 8;
			END_GO(1, sgn);
			ph = (ph + 8 - 2 * sgn) % 8;
			END_GO(1, 0);
		} else if (t == -65 || t == -66) {	//V90
			if (!diag) return 0;
			sgn = (t == -65) ? 1 : -1;
			px += hx[ph];
			py += hy[ph];
			ph = (ph + 8 - 2 * sgn) % 8;
			px += hx[ph];
			py += hy[ph];
		} else {
			return 0;
		}
	}
	if (ph & 1) {
		return 0;
	}
	END_GO(((px & 1) && (py & 1)) ? 2 : 3, 0);	//最後に止まるまで(区画の中心からなら1区画、境目からなら1.5区画)
#undef END_GO
	if (!((px & 1) && (py & 1)) || px < 0 || py < 0) {
		return 0;
	}
	*x = px / 2;
	*y = py / 2;
	*dir = ph / 2;
	return 1;
}

void Shortest_Pass_Compression_NANAME() {
	for (i = 0; G_Short_Pass_CP[i] != 0; i++) {
		G_Short_Pass_NANAME[i] = G_Short_Pass_CP[i];
	}
	/* G_Short_Pass_CPの終端(0)自体はコピーされないので、この位置の
	 * G_Short_Pass_NANAMEは前回実行時の値が残ったまま。末尾コーナー判定の
	 * 先読み用に1を置き(CP側と同じ手法)、処理後に必ず0へ戻す。
	 * これをしないと下のループが終端を見失い前回の残骸を読み進めてしまう */
	int naname_term_idx = i;
	G_Short_Pass_NANAME[i] = 1;
	for (i = 0; i < naname_term_idx; i++) {
		if ((G_Short_Pass_NANAME[i] == -2) || (G_Short_Pass_NANAME[i] == -3)) {
			if (G_Short_Pass_NANAME[i - 1] > 0) { //斜め入り
				if (G_Short_Pass_NANAME[i] == -2) { //左

					if (G_Short_Pass_NANAME[i + 1] == -3) {
						//入り４５
						G_Short_Pass_NANAME[i - 1] -= 1;
						G_Short_Pass_NANAME[i] = -51;
						NANAME_Flag = 1;
						Pass_zero_act();
					} else if (G_Short_Pass_NANAME[i + 1] == -2) {
						//入り135
						G_Short_Pass_NANAME[i - 1] -= 1;
						G_Short_Pass_NANAME[i] = -52;
						G_Short_Pass_NANAME[i + 1] = -1;
						//G_Short_Pass_NANAME[i + 2] = -1;
						NANAME_Flag = 1;
						Pass_zero_act();
					}
				} else if (G_Short_Pass_NANAME[i] == -3) { //右

					if (G_Short_Pass_NANAME[i + 1] == -2) {
						//入り４５
						G_Short_Pass_NANAME[i - 1] -= 1;
						G_Short_Pass_NANAME[i] = -53;
						NANAME_Flag = 1;
						Pass_zero_act();
					} else if (G_Short_Pass_NANAME[i + 1] == -3) {
						//入り135
						G_Short_Pass_NANAME[i - 1] -= 1;
						G_Short_Pass_NANAME[i] = -54;
						G_Short_Pass_NANAME[i + 1] = -1;
						//G_Short_Pass_NANAME[i + 2] = -1;
						NANAME_Flag = 1;
						Pass_zero_act();
					}
				}
			} else if ((G_Short_Pass_NANAME[i + 1] >= 0)
					&& (NANAME_Flag == 1)) { //斜め出 45
				if (G_Short_Pass_NANAME[i] == -3) {
					//右４５
					//G_Short_Pass_NANAME[i - 1] = -1;
					G_Short_Pass_NANAME[i] = -63;
					G_Short_Pass_NANAME[i + 1] -= 1;
					Pass_zero_act();
				} else if (G_Short_Pass_NANAME[i] == -2) {
					//左４５
					//G_Short_Pass_NANAME[i - 1] = -1;
					G_Short_Pass_NANAME[i] = -61;
					G_Short_Pass_NANAME[i + 1] -= 1;
					Pass_zero_act();
				}
				NANAME_Flag = 0;
			} else if ((G_Short_Pass_NANAME[i + 2] >= 0)
					&& (NANAME_Flag == 1)) {
				if (G_Short_Pass_NANAME[i] == G_Short_Pass_NANAME[i + 1]) {
					if (G_Short_Pass_NANAME[i] == -3) {
						//右135
						G_Short_Pass_NANAME[i] = -64;
						G_Short_Pass_NANAME[i + 1] = -1;
						G_Short_Pass_NANAME[i + 2] -= 1;
						Pass_zero_act();
					} else if (G_Short_Pass_NANAME[i] == -2) {
						//左135
						G_Short_Pass_NANAME[i] = -62;
						G_Short_Pass_NANAME[i + 1] = -1;
						G_Short_Pass_NANAME[i + 2] -= 1;
						Pass_zero_act();
					}
					NANAME_Flag = 0;
				} else {
					if ((G_Short_Pass_NANAME[i] == -2)
							|| (G_Short_Pass_NANAME[i] == -3)) {
						if (G_Short_Pass_NANAME[i]
								== G_Short_Pass_NANAME[i + 1]) {
							if (G_Short_Pass_NANAME[i] == -2) {
								G_Short_Pass_NANAME[i] = -65;
								G_Short_Pass_NANAME[i + 1] = -1;
							} else {
								G_Short_Pass_NANAME[i] = -66;
								G_Short_Pass_NANAME[i + 1] = -1;
							}

						} else {
							G_Short_Pass_NANAME[i] = -50;
							//G_Short_Pass_NANAME[i + 1] = -1;
						}
					}
				}
			} else if (NANAME_Flag == 1) {
				if ((G_Short_Pass_NANAME[i] == -2)
						|| (G_Short_Pass_NANAME[i] == -3)) {
					if (G_Short_Pass_NANAME[i] == G_Short_Pass_NANAME[i + 1]) {
						if (G_Short_Pass_NANAME[i] == -2) {
							G_Short_Pass_NANAME[i] = -65;
							G_Short_Pass_NANAME[i + 1] = -1;
						} else {
							G_Short_Pass_NANAME[i] = -66;
							G_Short_Pass_NANAME[i + 1] = -1;
						}

					} else {
						G_Short_Pass_NANAME[i] = -50;
						//G_Short_Pass_NANAME[i + 1] = -1;
					}
				}
			}
		}
	}
	for (i = 0; i < naname_term_idx; i++) {
		if (G_Short_Pass_NANAME[i] == -50) {
			for (int j = 1; G_Short_Pass_NANAME[i + j] == -50; j++) {
				G_Short_Pass_NANAME[i + j] = -1;
				G_Short_Pass_NANAME[i] -= 50;
			}
		}
	}

	/* 先読み用の1を正式な0終端に戻す */
	G_Short_Pass_NANAME[naname_term_idx] = 0;
}

void Maze_Wall_fill() {
	uint32_t Maze_Row_NoLook[MAZE_SIZE + 1];
	uint32_t Maze_Column_NoLook[MAZE_SIZE + 1];
	for (i = 0; i < MAZE_SIZE; i++) {
		Maze_Row_NoLook[i] = ~Maze_Row_Look[i];
		G_Maze_Row[i] = G_Maze_Row[i] | Maze_Row_NoLook[i];

		Maze_Column_NoLook[i] = ~Maze_Column_Look[i];
		G_Maze_Column[i] = G_Maze_Column[i] | Maze_Column_NoLook[i];
	}
}

void Maze_Save() {
	for (i = 0; i < MAZE_SIZE + 1; i++) {
		G_Maze_Column_Save[i] = G_Maze_Column[i];
		G_Maze_Row_Save[i] = G_Maze_Row[i];
		Maze_Row_Look_Save[i] = Maze_Row_Look[i];
		Maze_Column_Look_Save[i] = Maze_Column_Look[i];
	}
}

void Maze_Road() {
	for (i = 0; i < MAZE_SIZE + 1; i++) {
		G_Maze_Column[i] = G_Maze_Column_Save[i];
		G_Maze_Row[i] = G_Maze_Row_Save[i];
		Maze_Row_Look[i] = Maze_Row_Look_Save[i];
		Maze_Column_Look[i] = Maze_Column_Look_Save[i];
	}
}
void Maze_Mapping() {
	for (j = MAZE_SIZE; j > 0; j--) {
		for (i = 0; i < MAZE_SIZE; i++) {
			printf("+");
			if ((G_Maze_Row_Save[j] & (1u << i)) == (1u << i)) {
				printf("---");
			} else {
				printf("   ");
			}
		}
		printf("+\n\r");
		for (i = 0; i < MAZE_SIZE + 1; i++) {
			if ((G_Maze_Column_Save[i] & (1u << (j - 1))) == (1u << (j - 1))) {
				printf("|");
			} else {
				printf(" ");
			}
			printf("%3d", G_Step_Map[i][j - 1]);
			//printf("%3d", G_MAZE_Explored[i][j - 1]);
			//printf("%3d", G_MAZE_Closed[i][j - 1]);	//行き止まり潰しの確認用
		}
		printf("\n\r");
	}
	for (i = 0; i < MAZE_SIZE; i++) {
		printf("+");
		if ((G_Maze_Row[0] & (1u << i)) == (1u << i)) {
			printf("---");
		} else {
			printf("   ");
		}
	}
	printf("＋\n\r");
}

void Maze_Look_Mapping() {
	for (j = MAZE_SIZE; j > 0; j--) {
		for (i = 0; i < MAZE_SIZE; i++) {
			printf("+");
			if ((Maze_Row_Look[j] & (1u << i)) == (1u << i)) {
				printf("---");
			} else {
				printf("   ");
			}
		}
		printf("+\n\r");
		for (i = 0; i < MAZE_SIZE + 1; i++) {
			if ((Maze_Column_Look[i] & (1u << (j - 1))) == (1u << (j - 1))) {
				printf("|");
			} else {
				printf(" ");
			}
			printf("   ");
		}
		printf("\n\r");
	}
	for (i = 0; i < MAZE_SIZE; i++) {
		printf("+");
		if ((Maze_Row_Look[0] & (1u << i)) == (1u << i)) {
			printf("---");
		} else {
			printf("   ");
		}
	}
	printf("＋\n\r");					////かによし
}

void Maze_Dijkstra_Mapping() {
	for (int j = MAZE_SIZE; j > 0; j--) {
		printf("   ");
		for (int i = 0; i < MAZE_SIZE; i++) {
			printf("+");
			if ((G_Maze_Row[j] & (1u << i)) == (1u << i)) {
				printf("---");
			} else {
				printf("%3d", dijk_node_cost(1, i, j));
			}
		}
		printf("+\n\r");
		for (int i = 0; i < MAZE_SIZE + 1; i++) {
			if ((G_Maze_Column[i] & (1u << (j - 1))) == (1u << (j - 1))) {
				printf("   |");
			} else {
				printf(" %3d", dijk_node_cost(0, i, j - 1));
			}
		}
		printf("\n\r");
	}
	printf("   ");
	for (int i = 0; i < MAZE_SIZE; i++) {
		printf("+");
		if ((G_Maze_Row[0] & (1u << i)) == (1u << i)) {
			printf("---");
		} else {
			printf("   ");
		}
	}
	printf("＋\n\r");
}

void Pass_zero_act() {
	if (G_Short_Pass_NANAME[i - 1] == 0) {
		G_Short_Pass_NANAME[i - 1] = -1;
	}
	if (G_Short_Pass_NANAME[i + 1] == 0) {
		G_Short_Pass_NANAME[i + 1] = -1;
	}
	if (G_Short_Pass_NANAME[i + 2] == 0) {
		G_Short_Pass_NANAME[i + 2] = -1;
	}

	if (Known_Pass_NANAME[i - 1] == 0) {
		Known_Pass_NANAME[i - 1] = -1;
	}
	if (Known_Pass_NANAME[i + 1] == 0) {
		Known_Pass_NANAME[i + 1] = -1;
	}
	if (Known_Pass_NANAME[i + 2] == 0) {
		Known_Pass_NANAME[i + 2] = -1;
	}
}


/* ---- 迷路のデバッグ出力(main.c モード3 No.0) -------------------------------
 * 保存済みの迷路(Maze_Save()した壁・見た壁)をシリアルに出す。
 *   1. PCで読み込む用の16進の行(MAZE_DUMP_BEGIN 〜 MAZE_DUMP_END)
 *   2. 人が見る用の地図。"---"/"|"=壁、空白=壁なし(確認済み)、
 *      " . "/":"=未確認(最短走行では壁として扱う)、S=スタート、G=ゴール
 *   3. 最短走行と同じ手順で作った命令列(BFS版とダイクストラ版)
 * 出力を丸ごとファイルに保存して
 *   ./test_shortpath --dump そのファイル
 * にかけると、PCで同じ命令列が作られるか・壁に突っ込まないかを確認できる。
 * 走行はしない(モーターは動かさない)。作業用の壁情報・歩数マップ・経路配列は
 * 書き換わるが、保存済みの迷路(_Save)は変えない */
static void Maze_Debug_Print_Pass(const char *name, const int16_t *pass) {
	printf("%s:", name);
	for (int k = 0; k < MAX_STEP && pass[k] != 0; k++) {
		printf(" %d", pass[k]);
	}
	printf("\r\n");
}

static void Maze_Debug_Plan(int dijkstra) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	if (dijkstra) {
		Maze_Dijkstra_Set_Run(&G_Short_Run_Set[0]);	//モード1 No.5 と同じ値で(2026-10-27)
		Maze_Dijkstra_Calculation();
	} else {
		Maze_Shortest_Calculation();
	}
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
}

void Maze_Debug_Dump(void) {
	int x, y;

	printf("\r\n#BEGIN MAZE size=%d goal_x=%d goal_y=%d\r\n", MAZE_SIZE, MAZE_GOOL_X, MAZE_GOOL_Y);	//tools/viewer.html で読む
	printf("MAZE_DUMP_BEGIN %d\r\n", MAZE_SIZE);
	for (int k = 0; k < MAZE_SIZE + 1; k++) {
		printf("ROW %d %08lX\r\n", k, (unsigned long) G_Maze_Row_Save[k]);
	}
	for (int k = 0; k < MAZE_SIZE + 1; k++) {
		printf("COL %d %08lX\r\n", k, (unsigned long) G_Maze_Column_Save[k]);
	}
	for (int k = 0; k < MAZE_SIZE + 1; k++) {
		printf("LROW %d %08lX\r\n", k, (unsigned long) Maze_Row_Look_Save[k]);
	}
	for (int k = 0; k < MAZE_SIZE + 1; k++) {
		printf("LCOL %d %08lX\r\n", k, (unsigned long) Maze_Column_Look_Save[k]);
	}
	printf("MAZE_DUMP_END\r\n\r\n");

	/* 人が見る用の地図(上が北) */
	int unknown = 0;
	for (y = MAZE_SIZE; y >= 0; y--) {
		for (x = 0; x < MAZE_SIZE; x++) {
			int seen = (Maze_Row_Look_Save[y] >> x) & 1u;
			int wall = (G_Maze_Row_Save[y] >> x) & 1u;
			printf("+%s", !seen ? " . " : (wall ? "---" : "   "));
			unknown += !seen;
		}
		printf("+\r\n");
		if (y == 0) {
			break;
		}
		for (x = 0; x < MAZE_SIZE + 1; x++) {
			int seen = (Maze_Column_Look_Save[x] >> (y - 1)) & 1u;
			int wall = (G_Maze_Column_Save[x] >> (y - 1)) & 1u;
			printf("%s", !seen ? ":" : (wall ? "|" : " "));
			unknown += !seen;
			if (x < MAZE_SIZE) {
				int cy = y - 1;
				char c = ' ';
				if (x == 0 && cy == 0) {
					c = 'S';
				} else if (x >= MAZE_GOOL_X && x <= MAZE_GOOL_X + 1
						&& cy >= MAZE_GOOL_Y && cy <= MAZE_GOOL_Y + 1) {
					c = 'G';
				}
				printf(" %c ", c);
			}
		}
		printf("\r\n");
	}
	printf("unknown walls: %d\r\n\r\n", unknown);

	/* 電源を入れてからの各計算の最大時間(この下の経路計算の分は含まない) */
	printf("TIME_MAX_US wall_update=%lu known_pass=%lu step=%lu dijkstra=%lu\r\n\r\n",
			(unsigned long) G_Time_Max_WallUpdate_us,
			(unsigned long) G_Time_Max_Known_us,
			(unsigned long) G_Time_Max_Step_us,
			(unsigned long) G_Time_Max_Dijk_us);

	/* 最短走行と同じ手順の命令列 */
	Maze_Debug_Plan(0);
	if (G_Short_Pass[0] == 0) {	//空の経路(最短走行ではエラーになって走らない)
		printf("NO PATH: start cannot reach goal (unknown walls count as walls)\r\n");
	}
	Maze_Debug_Print_Pass("BFS_PASS", G_Short_Pass);
	Maze_Debug_Print_Pass("BFS_NANAME", G_Short_Pass_NANAME);
	Maze_Debug_Plan(1);
	Maze_Debug_Print_Pass("DIJK_PASS", G_Short_Pass);
	Maze_Debug_Print_Pass("DIJK_NANAME", G_Short_Pass_NANAME);
	printf("#END MAZE\r\n\r\n");

	Maze_Road();	//作業用の壁情報を保存済みの状態に戻しておく
}

/* ---- 計算時間を測るための入口 -----------------------------------------------
 * 探索中は計算のあいだも機体は前の速度のまま進むので、計算が長いとその分だけ
 * 直線が伸びる。最大時間はファイル先頭の G_Time_Max_* に記録する */

/* ---- フェイルセーフで止まったときに、最後に壁を書いた区画を戻す(2026-10-27、ToraNeko.Mk3 と同じ) ----
 * 壁を書く前に「その区画と、書く前の4枚の壁・見たかどうか・G_MAZE_Explored」を覚えておく(新しい MAZE_UNDO_CELLS 回ぶん)。
 * ぶつかる直前の区画は、ずれた位置・向きで壁を読んだかもしれないので、フェイルセーフで止まったら
 * 新しいほうから順に、書く前の状態に戻す(その走行で初めて入った区画は「まだ見ていない」に戻る)。
 * Uターンで同じ区画に2回入ると、その区画は2回と数える */
#define MAZE_UNDO_CELLS 5
typedef struct {
	uint8_t x, y;
	uint8_t wall;		//bit0 北 bit1 東 bit2 南 bit3 西(1 = 壁あり)
	uint8_t look;		//同じ並び(1 = 見た)
	uint8_t explored;
} Maze_Undo_Cell;
static Maze_Undo_Cell s_undo[MAZE_UNDO_CELLS];
static int s_undo_n = 0;

/* 区画 (x, y) の北・東・南・西の壁を、row・column の並びから4ビットにする */
static uint8_t Maze_Undo_Bits(const uint32_t *row, const uint32_t *col, int x, int y) {
	uint8_t b = 0;
	if (row[y + 1] & (1u << x)) {
		b |= 1;
	}
	if (col[x + 1] & (1u << y)) {
		b |= 2;
	}
	if (row[y] & (1u << x)) {
		b |= 4;
	}
	if (col[x] & (1u << y)) {
		b |= 8;
	}
	return b;
}

/* 4ビットの壁を、row・column の並びの区画 (x, y) に書き戻す */
static void Maze_Undo_Put(uint32_t *row, uint32_t *col, int x, int y, uint8_t b) {
	row[y + 1] = (b & 1) ? (row[y + 1] | (1u << x)) : (row[y + 1] & ~(1u << x));
	col[x + 1] = (b & 2) ? (col[x + 1] | (1u << y)) : (col[x + 1] & ~(1u << y));
	row[y] = (b & 4) ? (row[y] | (1u << x)) : (row[y] & ~(1u << x));
	col[x] = (b & 8) ? (col[x] | (1u << y)) : (col[x] & ~(1u << y));
}

/* 覚えている区画を忘れる。探索を始めるときに呼ぶ(その走行で書いた壁だけを戻すため) */
void Maze_Undo_Clear(void) {
	s_undo_n = 0;
}

/* 壁を書く区画 (x, y) の、書く前の状態を覚える。いっぱいなら、いちばん古いものを捨てる */
static void Maze_Undo_Record(int x, int y) {
	if ((x < 0) || (x >= MAZE_SIZE) || (y < 0) || (y >= MAZE_SIZE)) {
		return;
	}
	if (s_undo_n == MAZE_UNDO_CELLS) {
		for (int k = 0; k + 1 < MAZE_UNDO_CELLS; k++) {
			s_undo[k] = s_undo[k + 1];
		}
		s_undo_n = MAZE_UNDO_CELLS - 1;
	}
	Maze_Undo_Cell *c = &s_undo[s_undo_n];
	c->x = x;
	c->y = y;
	c->wall = Maze_Undo_Bits(G_Maze_Row, G_Maze_Column, x, y);
	c->look = Maze_Undo_Bits(Maze_Row_Look, Maze_Column_Look, x, y);
	c->explored = G_MAZE_Explored[x][y];
	s_undo_n++;
}

/* 最後に壁を書いた区画(MAZE_UNDO_CELLS 回まで)を、新しいほうから順に書く前の状態に戻す。
 * 戻したら忘れる(2回呼んでも1回ぶんしか戻さない)。戻した回数を返す。
 * 保存した迷路(Maze_Save)は変えないので、戻したあとに Maze_Save() する */
int Maze_Undo_Recent_Cells(void) {
	int n = s_undo_n;
	for (int k = s_undo_n - 1; k >= 0; k--) {
		Maze_Undo_Cell *c = &s_undo[k];
		Maze_Undo_Put(G_Maze_Row, G_Maze_Column, c->x, c->y, c->wall);
		Maze_Undo_Put(Maze_Row_Look, Maze_Column_Look, c->x, c->y, c->look);
		G_MAZE_Explored[c->x][c->y] = c->explored;
	}
	s_undo_n = 0;
	return n;
}

void Maze_Wall_Update() {
	uint32_t t = Maze_Time_Start();
	if (Known_Flag == 0) {
		static const int dx[4] = { 0, 1, 0, -1 };
		static const int dy[4] = { 1, 0, -1, 0 };
		int d = G_Robot_Direction % 4;	//Maze_Wall_Update_Body と同じ: 0 北、1 東、2 南、それ以外は西
		if ((d < 0) || (d > 2)) {
			d = 3;
		}
		Maze_Undo_Record(G_Robot_MAZE_X + dx[d], G_Robot_MAZE_Y + dy[d]);
	}
	Maze_Wall_Update_Body();
	Maze_Time_End(&G_Time_Max_WallUpdate_us, t);
}

void Known_Pass_Generation() {
	uint32_t t = Maze_Time_Start();
	Known_Pass_Generation_Body();
	Maze_Time_End(&G_Time_Max_Known_us, t);
}

void Maze_Step_Calculate() {
	uint32_t t = Maze_Time_Start();
	Maze_Step_Calculate_Body();
	Maze_Time_End(&G_Time_Max_Step_us, t);
}

void Maze_Dijkstra_Calculation() {
	uint32_t t = Maze_Time_Start();
	Maze_Dijkstra_Calculation_Body();
	Maze_Time_End(&G_Time_Max_Dijk_us, t);
}

/* 今いる区画(G_Robot_MAZE_X/Y)と向き(G_Robot_Direction)から、前後左右の
 * 区画の歩数を G_Maze_Flont/Back/Left/Right に入れる(壁があれば MAX_STEP)。
 * Maze_Wall_Update() の後半と同じ内容を、区画を移動せずに行う。
 * 止まっているあいだに歩数マップを作り直したあと、進む向きを決め直すのに使う */
void Maze_Neighbor_Update(void) {
	static const int dx[4] = { 0, 1, 0, -1 };
	static const int dy[4] = { 1, 0, -1, 0 };
	int x = G_Robot_MAZE_X;
	int y = G_Robot_MAZE_Y;
	int h = G_Robot_Direction % 4;
	uint16_t v[4];	//向き 0=前 1=右 2=後 3=左
	for (int k = 0; k < 4; k++) {
		int d = (h + k) % 4;	//絶対方位 0=北 1=東 2=南 3=西
		int nx = x + dx[d];
		int ny = y + dy[d];
		int wall;
		if (d == 0) {
			wall = (G_Maze_Row[y + 1] >> x) & 1u;
		} else if (d == 1) {
			wall = (G_Maze_Column[x + 1] >> y) & 1u;
		} else if (d == 2) {
			wall = (G_Maze_Row[y] >> x) & 1u;
		} else {
			wall = (G_Maze_Column[x] >> y) & 1u;
		}
		if (wall || nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) {
			v[k] = MAX_STEP;
		} else {
			v[k] = G_Step_Map[nx][ny];
		}
	}
	G_Maze_Flont = v[0];
	G_Maze_Right = v[1];
	G_Maze_Back = v[2];
	G_Maze_Left = v[3];
}

