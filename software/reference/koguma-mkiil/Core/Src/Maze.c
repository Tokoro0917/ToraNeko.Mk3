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

typedef struct {
	uint16_t cost;
	uint8_t x;
	uint8_t y;
	uint8_t direction;
	uint8_t isRow;
	uint8_t inQueue; //SPFA: このノードが現在キューに積まれているか(確定済みフラグではない)
} NODE_T;

typedef struct {
	int head;
	int tail;
	NODE_T *data[MAX_QUEUE_NODE_NUM];
} Queue_T;

NODE_T node_Row[MAZE_SIZE + 1][MAZE_SIZE + 1];
NODE_T node_Column[MAZE_SIZE + 1][MAZE_SIZE + 1];

#ifdef SIM_DEBUG
int g_queue_node_push_count = 0;
int g_queue_node_max_occupancy = 0;
int g_queue_node_overflow_count = 0;
int g_dijkstra_backtrace_hops = 0;
int g_bfs_outer_hops = 0;
#endif

void pushQueue_walk_node(Queue_T *queue, NODE_T *input) {
	/* データをデータの最後尾の１つ後ろに格納*/
	queue->data[queue->tail] = input;
	/* データの最後尾を１つ後ろに移動*/
	queue->tail = queue->tail + 1;
	/* 巡回シフト*/
	if (queue->tail == MAX_QUEUE_NODE_NUM)
		queue->tail = 0;
	/* スタックが満杯なら何もせず関数終了*/
	if (queue->tail == queue->head) {
		//printf("queue_full\n");return;
#ifdef SIM_DEBUG
		g_queue_node_overflow_count++;
#endif
	}
#ifdef SIM_DEBUG
	g_queue_node_push_count++;
	int occ = queue->tail - queue->head;
	if (occ < 0)
		occ += MAX_QUEUE_NODE_NUM;
	if (occ > g_queue_node_max_occupancy)
		g_queue_node_max_occupancy = occ;
#endif
}

NODE_T* popqueue_walk_node(Queue_T *queue) {
	NODE_T *ret = NULL;
	/* スタックが空なら何もせずに関数終了*/
	if (queue->tail == queue->head) {
		//printf("queue_empty\n");
		// ret->cost=65535;
		return ret;
	}
	/* データの最前列からデータを取得*/
	ret = queue->data[queue->head];
	/* データの最前列を１つ前にずらす*/
	queue->head = queue->head + 1;
	/* 巡回シフト*/
	if (queue->head == MAX_QUEUE_NODE_NUM)
		queue->head = 0;
	/* 取得したデータを返却*/
	return ret;
}

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

void Maze_Wall_Update() {
	if (G_Robot_Direction % 4 == 0) { //////////////////////////////////////////////////////////北に移動
		G_Robot_MAZE_X += 0;
		G_Robot_MAZE_Y += 1;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Column[G_Robot_MAZE_X] = G_Maze_Column[G_Robot_MAZE_X]
						| (1u << (G_Robot_MAZE_Y));
			}
			if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {		//前壁がある//
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
			if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {		//前壁がある
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
			if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {		//前壁がある
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
			if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {		//前壁がある
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

void Known_Pass_Generation() {
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
	/* G_Known_Passの終端(0)自体はコピーされないので、この位置のKnown_Pass_CPは
	 * 前回実行時の値が残ったまま。末尾コーナー判定の先読み用に1を置き、
	 * 処理後に必ず0へ戻す(Shortest_Pass_Compression()と同じ手法)。
	 * これをしないと下のループが終端を見失い前回の残骸を読み進めてしまう */
	int term_idx = i;
	G_Known_Pass[i] = 1;
	Known_Pass_CP[i] = 1;
	Known_Pass_CP[0] = 1;
	for (i = 0; i < term_idx; i++) {
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

	/* 先読み用に置いていた終端を正式な0へ戻す */
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
void Known_Pass_Compression_NANAME() {
	NANAME_Flag = 0;

	for (i = 0; Known_Pass_CP[i] != 0; i++) {
		Known_Pass_NANAME[i] = Known_Pass_CP[i];
	}
	/* Known_Pass_CPの終端(0)自体はコピーされないので、この位置の
	 * Known_Pass_NANAMEは前回実行時の値が残ったまま。末尾コーナー判定の
	 * 先読み用に1を置き(CP側と同じ手法)、処理後に必ず0へ戻す。
	 * これをしないと下のループが終端を見失い前回の残骸を読み進めてしまう */
	int naname_term_idx = i;
	Known_Pass_NANAME[i] = 1;
	for (i = 0; i < naname_term_idx; i++) {
		if ((Known_Pass_NANAME[i] == -2) || (Known_Pass_NANAME[i] == -3)) {
			if (Known_Pass_NANAME[i - 1] > 0) { //斜め入り
				if (Known_Pass_NANAME[i] == -2) { //左

					if (Known_Pass_NANAME[i + 1] == -3) {
						//入り４５
						Known_Pass_NANAME[i - 1] -= 1;
						Known_Pass_NANAME[i] = -51;
						NANAME_Flag = 1;
						Pass_zero_act();
					} else if (Known_Pass_NANAME[i + 1] == -2) {
						//入り135
						Known_Pass_NANAME[i - 1] -= 1;
						Known_Pass_NANAME[i] = -52;
						Known_Pass_NANAME[i + 1] = -1;
						NANAME_Flag = 1;
						Pass_zero_act();
					}
				} else if (Known_Pass_NANAME[i] == -3) { //右

					if (Known_Pass_NANAME[i + 1] == -2) {
						//入り４５
						Known_Pass_NANAME[i - 1] -= 1;
						Known_Pass_NANAME[i] = -53;
						NANAME_Flag = 1;
						Pass_zero_act();
					} else if (Known_Pass_NANAME[i + 1] == -3) {
						//入り135
						Known_Pass_NANAME[i - 1] -= 1;
						Known_Pass_NANAME[i] = -54;
						Known_Pass_NANAME[i + 1] = -1;
						NANAME_Flag = 1;
						Pass_zero_act();
					}
				}
			} else if ((Known_Pass_NANAME[i + 1] >= 0) && (NANAME_Flag == 1)) { //斜め出 45
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
			} else if ((Known_Pass_NANAME[i + 2] >= 0) && (NANAME_Flag == 1)) {
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
	for (i = 0; i < naname_term_idx; i++) {
		if (Known_Pass_NANAME[i] == -50) {
			for (int j = 1; Known_Pass_NANAME[i + j] == -50; j++) {
				Known_Pass_NANAME[i + j] = -1;
				Known_Pass_NANAME[i] -= 50;
			}
		}
	}

	/* 先読み用の1を正式な0終端に戻す */
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

void Maze_Step_Calculate() {
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
}

void Maze_Shortest_Calculation() {
	int Short_MAZE_X = 0;
	int Short_MAZE_Y = 0;
	int N = 0;

	G_Step_Map[G_Gool_X][G_Gool_Y] = 0;
	G_Step_Map[G_Gool_X + 1][G_Gool_Y] = 0;
	G_Step_Map[G_Gool_X][G_Gool_Y + 1] = 0;
	G_Step_Map[G_Gool_X + 1][G_Gool_Y + 1] = 0;

	for (i = 0; i < MAX_STEP; i++) {
		G_Short_Pass[i] = 0;
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

void Maze_Dijkstra_Calculation() {
	/* MAZE_SIZE=32だとdata[]がMAX_QUEUE_NODE_NUM(=ノード総数)分あり、スタックに
	 * 置くとスタックオーバーフローの危険があるためstatic(.bss)に置く */
	static Queue_T queue_node;
	queue_node.head = 0;
	queue_node.tail = 0;

	for (int i = 0; i < MAZE_SIZE + 1; i++) { //dijk 初期化
		for (int j = 0; j < MAZE_SIZE + 1; j++) {
			node_Row[i][j].cost = DIJK_MAXCOST;
			node_Row[i][j].isRow = 1;
			node_Row[i][j].inQueue = 0;
			node_Row[i][j].x = i;
			node_Row[i][j].y = j;
			node_Column[i][j].cost = DIJK_MAXCOST;
			node_Column[i][j].isRow = 0;
			node_Column[i][j].inQueue = 0;
			node_Column[i][j].x = i;
			node_Column[i][j].y = j;
		}
	}

	for (int i = 0; i < MAZE_SIZE + 1; i++) { //dijk 壁入れ
		for (int j = 0; j < MAZE_SIZE + 1; j++) {
			if ((G_Maze_Row[j] & (1u << i)) == (1u << i)) {
				node_Row[i][j].cost = DIJK_WALLCOST;
			}
			if ((G_Maze_Column[i] & (1u << j)) == (1u << j)) {
				node_Column[i][j].cost = DIJK_WALLCOST;
			}
		}
	}

	/* ゴールは2x2区画。Maze_Shortest_Calculation(普通の最短)と同じく、
	 * どの面から入ってもゴール区画に入った時点で経路生成を止めたい。
	 * 大会ルール上どの面が開口になっているか事前には分からないため、
	 * 2x2区画を囲む8本の「外から侵入する境界」全てをコスト0の
	 * マルチソースにする(壁で塞がれている境界は起点にしない) */
	{
		int gx = G_Gool_X;
		int gy = G_Gool_Y;
		NODE_T *goal_entries[8] = { &node_Row[gx][gy], //(gx,gy)   南から
				&node_Row[gx + 1][gy], //(gx+1,gy) 南から
				&node_Row[gx][gy + 2], //(gx,gy+1)   北から
				&node_Row[gx + 1][gy + 2], //(gx+1,gy+1) 北から
				&node_Column[gx][gy], //(gx,gy)   西から
				&node_Column[gx][gy + 1], //(gx,gy+1) 西から
				&node_Column[gx + 2][gy], //(gx+1,gy)   東から
				&node_Column[gx + 2][gy + 1], //(gx+1,gy+1) 東から
				};
		for (int k = 0; k < 8; k++) {
			if (goal_entries[k]->cost != DIJK_WALLCOST) {
				goal_entries[k]->cost = 0;
				goal_entries[k]->inQueue = 1;
				pushQueue_walk_node(&queue_node, goal_entries[k]);
			}
		}
	}

	while (1) {
		NODE_T *popNode;
		popNode = popqueue_walk_node(&queue_node);

		if (popNode == NULL) { //END
			break;
		}

		popNode->inQueue = 0; //SPFA: キューから出た。まだ確定ではないので後で再度緩和され得る

		if (popNode->isRow == 1) { //Row
			if (node_Row[popNode->x][popNode->y + 1].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 0) {
					if (node_Row[popNode->x][popNode->y + 1].cost
							> popNode->cost + CON_COST) {
						node_Row[popNode->x][popNode->y + 1].cost =
								popNode->cost + CON_COST;
						node_Row[popNode->x][popNode->y + 1].direction = 0; //N
						if (node_Row[popNode->x][popNode->y + 1].inQueue != 1) {
							node_Row[popNode->x][popNode->y + 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x][popNode->y + 1]);
						}
					}
				} else {
					if (node_Row[popNode->x][popNode->y + 1].cost
							> popNode->cost + ST_COST) {
						node_Row[popNode->x][popNode->y + 1].cost =
								popNode->cost + ST_COST;
						node_Row[popNode->x][popNode->y + 1].direction = 0; //N
						if (node_Row[popNode->x][popNode->y + 1].inQueue != 1) {
							node_Row[popNode->x][popNode->y + 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x][popNode->y + 1]);
						}
					}
				}
			}
			if (node_Row[popNode->x][popNode->y - 1].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 4) {
					if (node_Row[popNode->x][popNode->y - 1].cost
							> popNode->cost + CON_COST) {
						node_Row[popNode->x][popNode->y - 1].cost =
								popNode->cost + CON_COST;
						node_Row[popNode->x][popNode->y - 1].direction = 4; //S
						if (node_Row[popNode->x][popNode->y - 1].inQueue != 1) {
							node_Row[popNode->x][popNode->y - 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x][popNode->y - 1]);
						}
					}
				} else {
					if (node_Row[popNode->x][popNode->y - 1].cost
							> popNode->cost + ST_COST) {
						node_Row[popNode->x][popNode->y - 1].cost =
								popNode->cost + ST_COST;
						node_Row[popNode->x][popNode->y - 1].direction = 4; //S
						if (node_Row[popNode->x][popNode->y - 1].inQueue != 1) {
							node_Row[popNode->x][popNode->y - 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x][popNode->y - 1]);
						}
					}
				}
			}
			if (node_Column[popNode->x][popNode->y].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 7) {
					if (node_Column[popNode->x][popNode->y].cost
							> popNode->cost + CON_COST) {
						node_Column[popNode->x][popNode->y].cost =
								popNode->cost + CON_COST;
						node_Column[popNode->x][popNode->y].direction = 7; //NW
						if (node_Column[popNode->x][popNode->y].inQueue != 1) {
							node_Column[popNode->x][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x][popNode->y]);
						}
					}
				} else {
					if (node_Column[popNode->x][popNode->y].cost
							> popNode->cost + DIAG_COST) {
						node_Column[popNode->x][popNode->y].cost =
								popNode->cost + DIAG_COST;
						node_Column[popNode->x][popNode->y].direction = 7; //NW
						if (node_Column[popNode->x][popNode->y].inQueue != 1) {
							node_Column[popNode->x][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x][popNode->y]);
						}
					}
				}
			}
			if (node_Column[popNode->x + 1][popNode->y].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 1) {
					if (node_Column[popNode->x + 1][popNode->y].cost
							> popNode->cost + CON_COST) {
						node_Column[popNode->x + 1][popNode->y].cost =
								popNode->cost + CON_COST;
						node_Column[popNode->x + 1][popNode->y].direction = 1; //NE
						if (node_Column[popNode->x + 1][popNode->y].inQueue != 1) {
							node_Column[popNode->x + 1][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x + 1][popNode->y]);
						}
					}
				} else {
					if (node_Column[popNode->x + 1][popNode->y].cost
							> popNode->cost + DIAG_COST) {
						node_Column[popNode->x + 1][popNode->y].cost =
								popNode->cost + DIAG_COST;
						node_Column[popNode->x + 1][popNode->y].direction = 1; //NE
						if (node_Column[popNode->x + 1][popNode->y].inQueue != 1) {
							node_Column[popNode->x + 1][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x + 1][popNode->y]);
						}
					}
				}
			}
			if (node_Column[popNode->x][popNode->y - 1].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 5) {
					if (node_Column[popNode->x][popNode->y - 1].cost
							> popNode->cost + CON_COST) {
						node_Column[popNode->x][popNode->y - 1].cost =
								popNode->cost + CON_COST;
						node_Column[popNode->x][popNode->y - 1].direction = 5; //SW
						if (node_Column[popNode->x][popNode->y - 1].inQueue != 1) {
							node_Column[popNode->x][popNode->y - 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x][popNode->y - 1]);
						}
					}
				} else {
					if (node_Column[popNode->x][popNode->y - 1].cost
							> popNode->cost + DIAG_COST) {
						node_Column[popNode->x][popNode->y - 1].cost =
								popNode->cost + DIAG_COST;
						node_Column[popNode->x][popNode->y - 1].direction = 5; //SW
						if (node_Column[popNode->x][popNode->y - 1].inQueue != 1) {
							node_Column[popNode->x][popNode->y - 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x][popNode->y - 1]);
						}
					}
				}
			}
			if (node_Column[popNode->x + 1][popNode->y - 1].cost
					!= DIJK_WALLCOST) { //no wall
				if (popNode->direction == 3) {
					if (node_Column[popNode->x + 1][popNode->y - 1].cost
							> popNode->cost + CON_COST) {
						node_Column[popNode->x + 1][popNode->y - 1].cost =
								popNode->cost + CON_COST;
						node_Column[popNode->x + 1][popNode->y - 1].direction = 3; //SE
						if (node_Column[popNode->x + 1][popNode->y - 1].inQueue
								!= 1) {
							node_Column[popNode->x + 1][popNode->y - 1].inQueue =
									1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x + 1][popNode->y - 1]);
						}
					}
				} else {
					if (node_Column[popNode->x + 1][popNode->y - 1].cost
							> popNode->cost + DIAG_COST) {
						node_Column[popNode->x + 1][popNode->y - 1].cost =
								popNode->cost + DIAG_COST;
						node_Column[popNode->x + 1][popNode->y - 1].direction = 3; //SE
						if (node_Column[popNode->x + 1][popNode->y - 1].inQueue
								!= 1) {
							node_Column[popNode->x + 1][popNode->y - 1].inQueue =
									1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x + 1][popNode->y - 1]);
						}
					}
				}
			}
		} else { //column
			if (node_Column[popNode->x + 1][popNode->y].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 2) {
					if (node_Column[popNode->x + 1][popNode->y].cost
							> popNode->cost + CON_COST) {
						node_Column[popNode->x + 1][popNode->y].cost =
								popNode->cost + CON_COST;
						node_Column[popNode->x + 1][popNode->y].direction = 2; //E
						if (node_Column[popNode->x + 1][popNode->y].inQueue != 1) {
							node_Column[popNode->x + 1][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x + 1][popNode->y]);
						}
					}
				} else {
					if (node_Column[popNode->x + 1][popNode->y].cost
							> popNode->cost + ST_COST) {
						node_Column[popNode->x + 1][popNode->y].cost =
								popNode->cost + ST_COST;
						node_Column[popNode->x + 1][popNode->y].direction = 2; //E
						if (node_Column[popNode->x + 1][popNode->y].inQueue != 1) {
							node_Column[popNode->x + 1][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x + 1][popNode->y]);
						}
					}
				}
			}
			if (node_Column[popNode->x - 1][popNode->y].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 6) {
					if (node_Column[popNode->x - 1][popNode->y].cost
							> popNode->cost + CON_COST) {
						node_Column[popNode->x - 1][popNode->y].cost =
								popNode->cost + CON_COST;
						node_Column[popNode->x - 1][popNode->y].direction = 6; //E
						if (node_Column[popNode->x - 1][popNode->y].inQueue != 1) {
							node_Column[popNode->x - 1][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x - 1][popNode->y]);
						}
					}
				} else {
					if (node_Column[popNode->x - 1][popNode->y].cost
							> popNode->cost + ST_COST) {
						node_Column[popNode->x - 1][popNode->y].cost =
								popNode->cost + ST_COST;
						node_Column[popNode->x - 1][popNode->y].direction = 6; //E
						if (node_Column[popNode->x - 1][popNode->y].inQueue != 1) {
							node_Column[popNode->x - 1][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Column[popNode->x - 1][popNode->y]);
						}
					}
				}
			}
			if (node_Row[popNode->x][popNode->y + 1].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 1) {
					if (node_Row[popNode->x][popNode->y + 1].cost
							> popNode->cost + CON_COST) {
						node_Row[popNode->x][popNode->y + 1].cost =
								popNode->cost + CON_COST;
						node_Row[popNode->x][popNode->y + 1].direction = 1; //NE
						if (node_Row[popNode->x][popNode->y + 1].inQueue != 1) {
							node_Row[popNode->x][popNode->y + 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x][popNode->y + 1]);
						}
					}
				} else {
					if (node_Row[popNode->x][popNode->y + 1].cost
							> popNode->cost + DIAG_COST) {
						node_Row[popNode->x][popNode->y + 1].cost =
								popNode->cost + DIAG_COST;
						node_Row[popNode->x][popNode->y + 1].direction = 1; //NE
						if (node_Row[popNode->x][popNode->y + 1].inQueue != 1) {
							node_Row[popNode->x][popNode->y + 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x][popNode->y + 1]);
						}
					}
				}
			}
			if (node_Row[popNode->x][popNode->y].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 3) {
					if (node_Row[popNode->x][popNode->y].cost
							> popNode->cost + CON_COST) {
						node_Row[popNode->x][popNode->y].cost =
								popNode->cost + CON_COST;
						node_Row[popNode->x][popNode->y].direction = 3; //SE
						if (node_Row[popNode->x][popNode->y].inQueue != 1) {
							node_Row[popNode->x][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x][popNode->y]);
						}
					}
				} else {
					if (node_Row[popNode->x][popNode->y].cost
							> popNode->cost + DIAG_COST) {
						node_Row[popNode->x][popNode->y].cost =
								popNode->cost + DIAG_COST;
						node_Row[popNode->x][popNode->y].direction = 3; //SE
						if (node_Row[popNode->x][popNode->y].inQueue != 1) {
							node_Row[popNode->x][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x][popNode->y]);
						}
					}
				}
			}
			if (node_Row[popNode->x - 1][popNode->y + 1].cost
					!= DIJK_WALLCOST) { //no wall
				if (popNode->direction == 7) {
					if (node_Row[popNode->x - 1][popNode->y + 1].cost
							> popNode->cost + CON_COST) {
						node_Row[popNode->x - 1][popNode->y + 1].cost =
								popNode->cost + CON_COST;
						node_Row[popNode->x - 1][popNode->y + 1].direction = 7; //NW
						if (node_Row[popNode->x - 1][popNode->y + 1].inQueue
								!= 1) {
							node_Row[popNode->x - 1][popNode->y + 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x - 1][popNode->y + 1]);
						}
					}
				} else {
					if (node_Row[popNode->x - 1][popNode->y + 1].cost
							> popNode->cost + DIAG_COST) {
						node_Row[popNode->x - 1][popNode->y + 1].cost =
								popNode->cost + DIAG_COST;
						node_Row[popNode->x - 1][popNode->y + 1].direction = 7; //NW
						if (node_Row[popNode->x - 1][popNode->y + 1].inQueue
								!= 1) {
							node_Row[popNode->x - 1][popNode->y + 1].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x - 1][popNode->y + 1]);
						}
					}
				}
			}
			if (node_Row[popNode->x - 1][popNode->y].cost != DIJK_WALLCOST) { //no wall
				if (popNode->direction == 5) {
					if (node_Row[popNode->x - 1][popNode->y].cost
							> popNode->cost + CON_COST) {
						node_Row[popNode->x - 1][popNode->y].cost =
								popNode->cost + CON_COST;
						node_Row[popNode->x - 1][popNode->y].direction = 5; //SW
						if (node_Row[popNode->x - 1][popNode->y].inQueue != 1) {
							node_Row[popNode->x - 1][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x - 1][popNode->y]);
						}
					}
				} else {
					if (node_Row[popNode->x - 1][popNode->y].cost
							> popNode->cost + DIAG_COST) {
						node_Row[popNode->x - 1][popNode->y].cost =
								popNode->cost + DIAG_COST;
						node_Row[popNode->x - 1][popNode->y].direction = 5; //SW
						if (node_Row[popNode->x - 1][popNode->y].inQueue != 1) {
							node_Row[popNode->x - 1][popNode->y].inQueue = 1;
							pushQueue_walk_node(&queue_node,
									&node_Row[popNode->x - 1][popNode->y]);
						}
					}
				}
			}
		}

	}

	int toGool_direction = 0;
	int N = 0;

	NODE_T *short_node;
	short_node = &node_Row[0][1]; //&node_Row[0][1];
	toGool_direction = (short_node->direction + 4) % 8; //

	for (int i = 0; i < 255; i++) {
		G_Short_Pass[i] = 0;
	}
	G_Dijk_Path_Len = 0;


	G_Short_Pass[0] -= 1;
	//N++;

#ifdef SIM_DEBUG
	int dbg_backtrace_hops = 0;
#endif
	int dijk_backtrace_guard = 0;
	while (1) {
#ifdef SIM_DEBUG
		dbg_backtrace_hops++;
#endif
		if (++dijk_backtrace_guard > MAX_QUEUE_NODE_NUM) {	//異常系フェイルセーフ(通常は経路長で先に止まる)
			break;
		}
		if (short_node->isRow == 1) {
			short_node = &node_Row[short_node->x][short_node->y];
		} else {
			short_node = &node_Column[short_node->x][short_node->y];
		}

		if (short_node->cost == 0) {
			break;
		}

		toGool_direction = (short_node->direction + 4) % 8;

		/* Dijkstra誘導 未知壁探索用: このホップが通る壁(short_node自身)が
		 * 実際に通過するセルはtoGool_directionの向きで決まる(進行方向側のセル)。
		 * Row型ノードはtoGool_directionが{0,1,7}なら北側セル、{3,4,5}なら南側セル。
		 * Column型ノードは{1,2,3}なら東側セル、{5,6,7}なら西側セルになる
		 * (このグラフではRow型ノードにdirection 2/6が、Column型ノードに
		 * direction 0/4が現れることはないので、この2分岐で網羅できる) */
		if (G_Dijk_Path_Len < MAX_STEP - 1) {
			int16_t cellX, cellY;
			if (short_node->isRow) {
				if (toGool_direction == 0 || toGool_direction == 1
						|| toGool_direction == 7) {
					cellX = short_node->x;
					cellY = short_node->y;
				} else {
					cellX = short_node->x;
					cellY = short_node->y - 1;
				}
			} else {
				if (toGool_direction == 2 || toGool_direction == 1
						|| toGool_direction == 3) {
					cellX = short_node->x;
					cellY = short_node->y;
				} else {
					cellX = short_node->x - 1;
					cellY = short_node->y;
				}
			}
			/* この壁が現時点で既知か未知かは記録時点のスナップショットにせず、
			 * Maze_Unknown_Wall_Scan()が呼ばれる都度Maze_Row_Look/Column_Lookを
			 * 直接見てライブ判定する(同じ経路データを使い回して複数回スキャン
			 * できるようにするため。詳しくはMaze_Unknown_Wall_Scan()参照) */
			G_Dijk_Path_X[G_Dijk_Path_Len] = cellX;
			G_Dijk_Path_Y[G_Dijk_Path_Len] = cellY;
			G_Dijk_Path_WallIsRow[G_Dijk_Path_Len] = short_node->isRow;
			G_Dijk_Path_WallI[G_Dijk_Path_Len] = short_node->x;
			G_Dijk_Path_WallJ[G_Dijk_Path_Len] = short_node->y;
			G_Dijk_Path_Len++;
		}

		if (toGool_direction == 0) {
			if (G_Short_Pass[N] < 0) {
				N++;
			}
			G_Short_Pass[N] += 2;
			short_node->y++;
		} else if (toGool_direction == 1) {
			N++;
			if (short_node->isRow == 1) {
				G_Short_Pass[N] = -3;
				short_node->x++;
				short_node->isRow = 0;
			} else {
				G_Short_Pass[N] = -2;
				short_node->y++;
				short_node->isRow = 1;
			}
		} else if (toGool_direction == 2) {
			if (G_Short_Pass[N] < 0) {
				N++;
			}
			G_Short_Pass[N] += 2;
			short_node->x++;
		} else if (toGool_direction == 3) {
			N++;
			if (short_node->isRow == 1) {
				G_Short_Pass[N] = -2;
				short_node->x++;
				short_node->y--;
				short_node->isRow = 0;
			} else {
				G_Short_Pass[N] = -3;
				short_node->isRow = 1;
			}
		} else if (toGool_direction == 4) {
			if (G_Short_Pass[N] < 0) {
				N++;
			}
			G_Short_Pass[N] += 2;
			short_node->y--;
		} else if (toGool_direction == 5) {
			N++;
			if (short_node->isRow == 1) {
				G_Short_Pass[N] = -3;
				short_node->y--;
				short_node->isRow = 0;
			} else {
				G_Short_Pass[N] = -2;
				short_node->x--;
				short_node->isRow = 1;
			}
		} else if (toGool_direction == 6) {
			if (G_Short_Pass[N] < 0) {
				N++;
			}
			G_Short_Pass[N] += 2;
			short_node->x--;
		} else if (toGool_direction == 7) {
			N++;
			if (short_node->isRow == 1) {
				G_Short_Pass[N] = -2;
				short_node->isRow = 0;
			} else {
				G_Short_Pass[N] = -3;
				short_node->x--;
				short_node->y++;
				short_node->isRow = 1;
			}
		}

	}

	if (G_Short_Pass[0] == 0) {
		G_Short_Pass[0] = -1;
	}
#ifdef SIM_DEBUG
	g_dijkstra_backtrace_hops = dbg_backtrace_hops;
#endif
}

/* 直近のMaze_Dijkstra_Calculation()が記録した経路(G_Dijk_Path_*)を先頭から
 * 走査し、まだ未確認の壁をG_Unknown_Target_X/Yとその壁自体の識別子
 * (G_Unknown_Wall_*)に記録する。戻り値1=見つかった、0=経路上の壁は全て確認済み
 * (Dijkstra再計算タイミングをゴール到達直後とUターン発生時に絞るため、
 * 各壁が現時点で未知かどうかはMaze_Row_Look/Maze_Column_Lookをその都度
 * ライブで見て判定する。これにより、同じ経路データに対してこの関数を
 * 複数回呼び、既に確認済みになった壁を自然に読み飛ばして次の未知壁を
 * 見つけられる — Dijkstraを毎回再計算し直す必要がない) */
int Maze_Unknown_Wall_Scan(void) {
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

/* Maze_Unknown_Wall_Scan()が見つけた目標セルを、Maze_Gool_Setting()のmode==2で
 * BFSの目標地点として使うようセットする。Maze_Unkown_ALL_ModeSet()の兄弟関数 */
void Maze_Unknown_Target_ModeSet(int x, int y) {
	G_Unknown_Target_X = x;
	G_Unknown_Target_Y = y;
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
				printf("%3d",node_Row[i][j].cost);
			}
		}
		printf("+\n\r");
		for (int i = 0; i < MAZE_SIZE + 1; i++) {
			if ((G_Maze_Column[i] & (1u << (j - 1))) == (1u << (j - 1))) {
				printf("   |");
			} else {
				printf(" %3d",node_Column[i][j-1].cost);//
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

