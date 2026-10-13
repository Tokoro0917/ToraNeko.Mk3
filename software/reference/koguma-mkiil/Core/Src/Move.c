/*
 * Move.c
 *
 *  Created on: 2023/08/29
 *      Author: akihi
 */

/*
 * Move.c
 *
 *  Created on: 2022/08/20
 *      Author: akihi
 */

#include "motor.h"
#include "Define.h"
#include "Move.h"
#include "Maze.h"
#include "Wallsensor.h"
#include "stdio.h"
#include "Failsafe.h"
#include "UI.h"
#include"Failsafe.h"
#include "SpeedPlan.h"
#include "LOG.h"
#include "math.h"

int G_Pass_before;
int G_Pass_after;

int Pass_NM = 0;

/* 前壁合わせ(Motor_Robot_Alignment)をしてよいか。
 * 前壁合わせは前センサの値が FlontWall_Distance(636) になるようにモーターを
 * 0.5秒動かすので、前に壁がないと全力で前進してしまう。地図上の壁の有無
 * (探索時のセンサ判定)に加えて、今の前センサの値でも壁があることを確かめる */
#define ALIGN_FRONT_MIN 200	/* 区画中心で前壁があれば約530。これより小さければ合わせない */

static void Robot_Align_If_Wall(int wall_in_front) {
	if (wall_in_front && (Wall_Flont_Av() > ALIGN_FRONT_MIN)) {
		Motor_Robot_Alignment();
		Motor_Stop();
	}
}

/* その場Uターン(左90°を2回)。前に壁があれば前壁に、左90°回ったあとは
 * 元の左の壁(今の前)があればそれに合わせる。
 * 以前は2回目も元の「前」の壁の有無で判定していたため、前が壁で左が空いている
 * 区画では、何もない方向に向かって前壁合わせをして急に前進していた */
void Robot_adjustment() {
	Motor_Stop();
	Robot_Align_If_Wall(G_Wall_data[0] == 1 && G_Wall_data[3] == 1);	//前の壁
	Motor_trapezoid_Turn(90, 600, 20000);
	Motor_Stop();
	Robot_Align_If_Wall(G_Wall_data[1] == 1);	//元の左の壁(今の前)
	Motor_trapezoid_Turn(90, 600, 20000);
	Motor_Stop();
}

void Robot_adjustment_180() {
	Suction_change(20);
	Robot_Align_If_Wall(G_Wall_data[0] == 1 && G_Wall_data[3] == 1);
	Motor_trapezoid_Turn(180, 2000, 50000);
	Motor_Stop();
	Suction_change(35);
}

void Robot_adjustment_Back() {
	Motor_trapezoid_Turn(90, 400, 25000);
	Motor_Stop();
	//Motor_Back();
	Motor_Stop();
	Motor_trapezoid(0, 200, 0, 5000, 19);
	Motor_Stop();
	Motor_trapezoid_Turn(90, 400, 25000);
	Motor_Stop();
	//Motor_Back();
	Motor_Stop();
	Motor_trapezoid_PID(0, 300, 300, 5000, 13);
}

void Robot_Maze_Sula_Action() {
	Motor_Sula_before(500, 500, 500, 5000, 20);
	if ((G_Maze_Flont <= G_Maze_Left) && (G_Maze_Flont <= G_Maze_Right)	//前進
			&& (G_Maze_Flont <= G_Maze_Back)) {
		Motor_trapezoid_PID(500, 500, 500, 5000, 160);
		G_Robot_Direction += 0;
		G_Robot_Lastaction = 0;

	} else if ((G_Maze_Left <= G_Maze_Right)	//左折
	&& (G_Maze_Left <= G_Maze_Back)) {
		Motor_Sula_COS(500, 90, 700, 10000);
		Motor_trapezoid_PID(500, 500, 500, 5000, 25);
		G_Robot_Direction += 3;
		G_Robot_Lastaction = 3;
	} else if (G_Maze_Right <= G_Maze_Back) {	//	右折
		Motor_Sula_COS(500, -90, 700, 10000);
		Motor_trapezoid_PID(500, 500, 500, 5000, 25);
		G_Robot_Direction += 1;
		G_Robot_Lastaction = 1;
	} else if ((G_Maze_Flont == MAX_STEP) && (G_Maze_Left == MAX_STEP)
			&& (G_Maze_Right == MAX_STEP)) {	//Uターン　全部壁あり
		if ((G_Robot_MAZE_X == 0) && (G_Robot_MAZE_Y == 0)) {	//初期位置に戻ってきたとき
			Motor_trapezoid(500, 500, 0, 5000, 70);
			Motor_Stop();
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			Robot_adjustment();
			Motor_Back();
			Motor_Stop();
		} else {
			Motor_trapezoid(500, 500, 0, 5000, 70);
			Motor_Stop();
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			Robot_adjustment();
			Motor_trapezoid_PID(0, 500, 500, 5000, 90);
		}
	} else {	//Uターン　一部壁無し
		Motor_trapezoid(500, 500, 0, 5000, 70);
		Motor_Stop();
		G_Robot_Direction += 2;
		G_Robot_Lastaction = 2;
		Robot_adjustment();
		Motor_trapezoid_PID(0, 500, 500, 5000, 90);
	}

}

/* 1にすると、次の吸引探索のUターンでログ(LOG.c、1kHzで2秒)を取り始める。
 * 1回取ったら0に戻る。ゴール後のUターンの調査用(Maze_Search()が往路のあとに1にする)。
 * 取ったログはモード4 No.7で出力する */
int G_Log_Next_UTurn = 0;

static void Log_UTurn_Start(void) {
	if (G_Log_Next_UTurn) {
		G_Log_Next_UTurn = 0;
		LOG_get_start();
	}
}

/* 区画の中央で止まっているあいだに呼ぶ計算(ダイクストラなど)。NULLなら何もしない。
 * 吸引探索のUターン(行き止まり以外では、計算の結果で進む向きを決め直す)、
 * 既知区間加速の行き止まりUターン(-8)、Robot_Maze_Stop_And_Go() で呼ばれる */
void (*G_Stop_Hook)(void) = NULL;

/* Robot_Maze_Stop_And_Go() で前壁合わせを済ませたら1。Robot_Maze_Go_From_Stop() で
 * 同じ前壁合わせを繰り返さないために使い、そこで0に戻す */
static int s_front_aligned = 0;

static int Stop_Hook_Run(void) {
	if (G_Stop_Hook != NULL) {
		G_Stop_Hook();
		return 1;
	}
	return 0;
}

/* 区画の中央で止まっている状態から、歩数マップで進む向きを決めて発進し、
 * 次の区画の境目まで進む(Uターンのあとと同じ位置)。v: 探索速度、ac: 加速度 */
void Robot_Maze_Go_From_Stop(float v, float ac) {
	Maze_Neighbor_Update();
	if ((G_Maze_Flont <= G_Maze_Left) && (G_Maze_Flont <= G_Maze_Right)
			&& (G_Maze_Flont <= G_Maze_Back)) {	//前進
		G_Robot_Lastaction = 0;
	} else if ((G_Maze_Left <= G_Maze_Right) && (G_Maze_Left <= G_Maze_Back)) {	//左
		if (!s_front_aligned) {
			Robot_Align_If_Wall(G_Wall_data[0] == 1 && G_Wall_data[3] == 1);
		}
		Motor_trapezoid_Turn(90, 600, 20000);
		Motor_Stop();
		G_Robot_Direction += 3;
		G_Robot_Lastaction = 3;
	} else if (G_Maze_Right <= G_Maze_Back) {	//右
		if (!s_front_aligned) {
			Robot_Align_If_Wall(G_Wall_data[0] == 1 && G_Wall_data[3] == 1);
		}
		Motor_trapezoid_Turn(-90, 600, 20000);
		Motor_Stop();
		G_Robot_Direction += 1;
		G_Robot_Lastaction = 1;
	} else {	//後ろ(Robot_adjustment() と同じ。前壁合わせが済んでいれば省く)
		Motor_Stop();
		if (!s_front_aligned) {
			Robot_Align_If_Wall(G_Wall_data[0] == 1 && G_Wall_data[3] == 1);	//前の壁
		}
		Motor_trapezoid_Turn(90, 600, 20000);
		Motor_Stop();
		Robot_Align_If_Wall(G_Wall_data[1] == 1);	//元の左の壁(今の前)
		Motor_trapezoid_Turn(90, 600, 20000);
		Motor_Stop();
		G_Robot_Direction += 2;
		G_Robot_Lastaction = 2;
		G_Just_UTurned = 1;
	}
	s_front_aligned = 0;
	Motor_trapezoid_PID(0, v, v, ac, 90);
}

/* 境目を通過中の状態から、次の区画の入口で壁を見て(Maze_Wall_Update)、
 * 区画の中央で止まり、前に壁があれば壁合わせをして、G_Stop_Hook の計算をしてから発進する。
 * suction: 1なら吸引探索(1000mm/s)、0ならスラローム探索(500mm/s)の動き */
void Robot_Maze_Stop_And_Go(int suction) {
	if (suction) {
		Motor_Sula_before(1000, 1000, 1000, 5000, 15);
		Motor_trapezoid(1000, 1000, 0, 10000, 75);
	} else {
		Motor_Sula_before(500, 500, 500, 5000, 20);
		Motor_trapezoid(500, 500, 0, 5000, 70);
	}
	Motor_Stop();
	Robot_Align_If_Wall(G_Wall_data[0] == 1 && G_Wall_data[3] == 1);	//止まったついでに前壁合わせ
	s_front_aligned = 1;
	Stop_Hook_Run();
	if (suction) {
		Robot_Maze_Go_From_Stop(1000, 10000);
	} else {
		Robot_Maze_Go_From_Stop(500, 5000);
	}
}

void Robot_Maze_Suction_Action() {
	Motor_Sula_before(1000, 1000, 1000, 5000, 15);
	if ((G_Maze_Flont <= G_Maze_Left) && (G_Maze_Flont <= G_Maze_Right)	//前進
			&& (G_Maze_Flont <= G_Maze_Back)) {
		Motor_trapezoid_PID(1000, 1000, 1000, 5000, 165);
		G_Robot_Direction += 0;
		G_Robot_Lastaction = 0;

	} else if ((G_Maze_Left <= G_Maze_Right)	//左折
	&& (G_Maze_Left <= G_Maze_Back)) {
		Motor_Sula_COS(1000, 90, 1640, 60000);
		Motor_trapezoid_PID(1000, 1000, 1000, 5000, 50);
		G_Robot_Direction += 3;
		G_Robot_Lastaction = 3;
	} else if (G_Maze_Right <= G_Maze_Back) {	//	右折
		Motor_Sula_COS(1000, -90, 1640, 60000);
		Motor_trapezoid_PID(1000, 1000, 1000, 5000, 50);
		G_Robot_Direction += 1;
		G_Robot_Lastaction = 1;
	} else if ((G_Maze_Flont == MAX_STEP) && (G_Maze_Left == MAX_STEP)
			&& (G_Maze_Right == MAX_STEP)) {	//Uターン　全部壁あり
		Log_UTurn_Start();
		if ((G_Robot_MAZE_X == 0) && (G_Robot_MAZE_Y == 0)) {	//初期位置に戻ってきたとき
			Motor_trapezoid(1000, 1000, 0, 10000, 75);
			Motor_Stop();
			Suction_change(20);
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			G_Just_UTurned = 1;
			Robot_adjustment();
			Motor_Back();
			Motor_Stop();
			Motor_trapezoid_PID(0, 300, 0, 10000, 45);
			Motor_Stop();
			Motor_Back();
			Motor_Stop();
		} else {
			Motor_trapezoid(1000, 1000, 0, 10000, 75);
			Motor_Stop();
			Stop_Hook_Run();	//行き止まりなのでUターンしかない
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			G_Just_UTurned = 1;
			Robot_adjustment();
			Motor_trapezoid_PID(0, 1000, 1000, 10000, 90);
		}
	} else {	//Uターン　一部壁無し
		Log_UTurn_Start();
		Motor_trapezoid(1000, 1000, 0, 10000, 75);
		Motor_Stop();
		if (Stop_Hook_Run()) {
			//止まっているあいだに計算した歩数マップで、進む向きを決め直す
			Robot_Maze_Go_From_Stop(1000, 10000);
			return;
		}
		G_Robot_Direction += 2;
		G_Robot_Lastaction = 2;
		G_Just_UTurned = 1;
		Robot_adjustment();
		Motor_trapezoid_PID(0, 1000, 1000, 10000, 90);
	}

}

void Robot_Maze_Pass_Action() {
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
	if (G_MAZE_Explored[Known_X][Known_Y] == 1) {
		Known_Pass_Generation();
		for (int i = 0; G_Known_Pass[i] != 0; i++) {
			if (Failsafe_Flag() == 1) {
				break;
			}
			if (G_Known_Pass[i] > 0) {			//1区間前進
				Motor_trapezoid_PID(1000, 4000, 1000, 15000,
						90 * G_Known_Pass[i]);
			} else if (G_Known_Pass[i] == -2) {			//左
				Motor_Sula_ST(1000, 1000, 1000, 5000, 20);
				Motor_Sula_COS(1000, 90, 1630, 60000);
				Motor_trapezoid_PID(1000, 1000, 1000, 5000, 48);
			} else if (G_Known_Pass[i] == -3) {			//右
				Motor_Sula_ST(1000, 1000, 1000, 5000, 20);
				Motor_Sula_COS(1000, -90, 1630, 60000);
				Motor_trapezoid_PID(1000, 1000, 1000, 5000, 48);
			} else if (G_Known_Pass[i] == -8) {			//行き止まりUターン
				Motor_trapezoid(1000, 1000, 0, 10000, 75);
				Motor_Stop();
				Stop_Hook_Run();	//既知区間の残りはこのまま走る(Uターンは変えない)
				G_Just_UTurned = 1;
				Robot_adjustment();
				Motor_trapezoid_PID(0, 1000, 1000, 10000, 90);
			} else if (G_Known_Pass[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(1000, 50, 0);
				Motor_Sula_COS(1000, 90, 750, 20000);
				Motor_Wallcut_END(1000, 85, 0);
			} else if (G_Known_Pass[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(1000, 50, 1);
				Motor_Sula_COS(1000, -90, 750, 20000);
				Motor_Wallcut_END(1000, 85, 1);
			} else if (G_Known_Pass[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(1000, 40, 0);
				Motor_Sula_COS(1000, 180, 680, 13000);
				Motor_Wallcut_END(1000, 70, 0);
			} else if (G_Known_Pass[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(1000, 40, 1);
				Motor_Sula_COS(1000, -180, 680, 13000);
				Motor_Wallcut_END(1000, 70, 1);
			}
			// 斜め(NANAME)区間の実行は未実装・未チューニングのためコメントアウトのまま。
			// 有効化する際は (1) このforループの走査対象をG_Known_Pass[]ではなく
			// Known_Pass_NANAME[]に切り替え、(2) 下記の速度・距離・角度の各定数を
			// 実機で再チューニングすること。生成側はKnown_Pass_Compression_NANAME()
			// (Maze.c)で既にKnown_Pass_NANAME[]として計算済み。
			/* else if (Known_Pass_NANAME[i] == -51) {			//入り　左４５
			 Motor_Wallcut_ST(600, 35, 0);
			 Motor_Sula_COS(600, 45, 450, 8000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 67);
			 } else if (Known_Pass_NANAME[i] == -52) {			//入り　左１３５
			 Motor_Wallcut_ST(600, 80, 0);
			 Motor_Sula_COS(600, 135, 550, 8000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 78);
			 } else if (Known_Pass_NANAME[i] == -53) {			//入り　右４５
			 Motor_Wallcut_ST(600, 35, 1);
			 Motor_Sula_COS(600, -45, 450, 8000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 67);
			 } else if (Known_Pass_NANAME[i] == -54) {			//入り　右１３５
			 Motor_Wallcut_ST(600, 80, 1);
			 Motor_Sula_COS(600, -135, 550, 8000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 78);
			 } else if (Known_Pass_NANAME[i] == -61) {			//出　左４５
			 Motor_Wallcut_ST_NANAME(600, 65, 0);
			 Motor_Sula_COS(600, 45, 600, 10000);
			 Motor_Wallcut_END(600, 43, 0);
			 } else if (Known_Pass_NANAME[i] == -62) {			//出　左１３５
			 Motor_Wallcut_ST_NANAME(600, 45, 0);
			 Motor_Sula_COS(600, 135, 500, 8000);
			 Motor_Wallcut_END(600, 85, 0);
			 } else if (Known_Pass_NANAME[i] == -63) {			//出　右４５
			 Motor_Wallcut_ST_NANAME(600, 65, 1);
			 Motor_Sula_COS(600, -45, 600, 10000);
			 Motor_Wallcut_END(600, 43, 1);
			 } else if (Known_Pass_NANAME[i] == -64) {			//出　右１３５
			 Motor_Wallcut_ST_NANAME(600, 45, 1);
			 Motor_Sula_COS(600, -135, 500, 8000);
			 Motor_Wallcut_END(600, 85, 1);
			 } else if (Known_Pass_NANAME[i] == -65) {			//V90左
			 Motor_Wallcut_ST_NANAME(600, 30, 0);
			 Motor_Sula_COS(600, 90, 600, 10000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 53);
			 } else if (Known_Pass_NANAME[i] == -66) {			//V90右
			 Motor_Wallcut_ST_NANAME(600, 30, 1);
			 Motor_Sula_COS(600, -90, 600, 10000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 53);
			 } else if (Known_Pass_NANAME[i] % 50 == 0) {			//直線
			 Motor_NANAME_PID(600, 1000, 600, 5000,
			 127.3 * Known_Pass_NANAME[i] / -50);

			 }*/
		}
		if (G_Robot_MAZE_X == 0 && G_Robot_MAZE_Y == 0) {
			Motor_trapezoid(1000, 1000, 0, 10000, 90);
			//Robot_adjustment_Back_Only();
		}
	} else {
		Robot_Maze_Suction_Action();
	}
}

/* 最短経路が作れたか確認する。スタートからゴールへ行けない迷路
 * (探索が途中で止まった、未確認の壁を壁とみなすと塞がる等)では
 * Maze_Shortest_Calculation()/Maze_Dijkstra_Calculation()が空の経路を返すので、
 * そのときはエラーを知らせて走らずに戻る。戻り値1=走ってよい */
static int Short_Pass_Check(void) {
	if (G_Short_Pass[0] != 0) {
		return 1;
	}
	printf("ERROR: no shortest path (start cannot reach goal)\r\n");
	LED_Path_Error();
	return 0;
}

/* 最初がターンの経路の、スタートから最初のターンの後距離まで(2026-10-07)。
 * スタート位置から24mmで区画中心、そこから前距離・ターン・後距離。24mm+前距離では
 * 最高速まで加速できないので、最初のターンだけ専用の遅い速度(G_First_Turn の速度)で曲がる。
 * 前距離までに 0→速度、後距離で 速度→最高速 へ、どちらも加速度 SHORT_FIRST_AC で cos加速する。
 * SHORT_FIRST_AC は実機で出せる加速度に合わせる(2026-10-08)。50000にしていたときのログ
 * (モード4 No.9)では、加速中は左右とも電圧が上限(200)に張り付き、実際の加速度は約25000〜28000だった。
 * 後距離のうちに最高速までは上がりきらないので、残りは次の直線(Short_Catchup)で加速する。
 * ターンごとの通過速度(Short_NANAME_MoveTurnV)では、後距離の終わりで出せる速度に合わせて
 * 次のターンの速度を下げる */
#define SHORT_FIRST_AC 25000.0f
#define SHORT_START_X 24.0f	//スタート位置から区画中心まで
/* 最初のターンの吸引(2026-10-08)。G_First_Turn はモード4 No.9〜11(吸引70)で合わせたので、
 * 吸引の弱い最短走行(2000は50)でも、最初のターンの後距離の終わりまではこの吸引にする。
 * 発進前に上げて SHORT_FIRST_SUCTION_WAIT [ms] 待ち、後距離の終わりで元に戻す */
#define SHORT_FIRST_SUCTION 70
/* 加速しながら曲がる最初のターン(2026-10-09、motor.c の Motor_First_Turn_Accel())。
 * 1 にすると、スタートから後距離の終わりまで加速度 SHORT_FIRST_TURN_AC で加速し続け、
 * ターンは走った距離で角度を決める(G_First_Turn の速度で合わせた軌跡を、速度に関係なく通る)。
 * 0 なら今までどおり(前距離で加速、ターンは一定速度、後距離で加速)。モード4 No.12〜14 は 1 で走る */
int G_First_Turn_Accel = 0;
#define SHORT_FIRST_TURN_AC 20000.0f
#define SHORT_FIRST_TURN_MARGIN 0.9f	//後距離が壁切れで早く終わる分の余裕(全体の距離のうち加速に使う割合)
#define SHORT_FIRST_SUCTION_WAIT 500

/* 最初のターンのパラメータ。実機で合わせるのはここ。
 * 前距離・後距離[mm]、角度[deg]、最大角速度[deg/s]、角加速度[deg/s^2]、速度[mm/s]。
 * 角速度・角加速度は、その行の速度のときの値。
 * 速度は 24mm+前距離で届く値にする(加速度25000で、大回り90(前距離45): 約1482、
 * 斜め入り45(15): 約1114、斜め入り135(38): 約1405、大回り180(60): 約1635)。
 *   届く速度 = √(4 × 25000 × (24 + 前距離) / π)
 * 届かない値を書いたときは届く速度で、角速度×k・角加速度×k² にして曲がる。
 * 後距離の終わりの速度 = √(速度² + 4 × 25000 × 後距離 / π)
 *   大回り90: 約2180、斜め入り45: 約1860、斜め入り135: 約1970(後距離 80 / 70 / 60 のとき)
 * 斜め入り45は角度が小さいので最大角速度まで上がらない(角加速度23000では約810で頭打ち)。
 * 調整はモード4 No.9〜No.11(スタート位置から最初のターンだけ走る) */
FirstTurnParam G_First_Turn[FIRST_TURN_NUM] = {
	//前距離, 角度, 最大角速度, 角加速度, 後距離, 速度
	{ 45, 90, 1184, 31640, 80, 1480 },	//FIRST_TURN_BIG90  大回り90(2026-10-08 実機で調整)
	{ 60, 180, 975, 22500, 40, 1500 },	//FIRST_TURN_BIG180 大回り180(スタートからは出ない)
	{ 15, 45, 1000, 23000, 70, 1110 },	//FIRST_TURN_IN45   斜め入り45(2026-10-08 実機で調整)
	{ 38, 135, 1100, 30000, 60, 1400 },	//FIRST_TURN_IN135  斜め入り135(2026-10-08 実機で調整)
};

/* 加速しながら曲がる最初のターンの、後距離の終わりの速度(V を超えない)。
 * スタートから後距離の終わりまでの距離 = 24mm + 前距離 + ターンの長さ + 後距離 の
 * SHORT_FIRST_TURN_MARGIN の割合で、0 から cos加速(加速度 SHORT_FIRST_TURN_AC)で届く速度 */
static float Short_First_Turn_Accel_V(int kind, float V) {
	const FirstTurnParam *p = &G_First_Turn[kind];
	float L = Sula_Ref_Length(p->v, p->ang, p->w, p->w_ac);
	float d = (SHORT_START_X + p->pre + L + p->post) * SHORT_FIRST_TURN_MARGIN;
	float v = sqrt(4 * SHORT_FIRST_TURN_AC * d / PI);
	return (v < V) ? v : V;
}

/* 最初のターンを走る。kind: FIRST_TURN_*、dir: 0 左 / 1 右、V: このあとの最高速。
 * スタート位置から24mm+前距離で 0→v1、ターン、後距離で v1→V(加速度 SHORT_FIRST_AC)。
 * v1 は表の速度(24mm+前距離で届かなければ届く速度。そのときは角速度×k、角加速度×k²)。
 * 戻り値は後距離の終わりの速度 */
float Short_First_Turn_Run(int kind, int dir, float V) {
	const FirstTurnParam *p = &G_First_Turn[kind];
	int naname = (kind == FIRST_TURN_IN45 || kind == FIRST_TURN_IN135);
	float v1 = p->v;
	float v_reach = sqrt(4 * SHORT_FIRST_AC * (SHORT_START_X + p->pre) / PI);
	if (v1 > v_reach) {
		v1 = v_reach;
	}
	if (v1 > V) {
		v1 = V;
	}
	float k = v1 / p->v;

	Suction_Boost_Start(SHORT_FIRST_SUCTION, SHORT_FIRST_SUCTION_WAIT);
	if (!LOG_Is_On()) {
		LOG_get_start();	//最初のターンの調査用(2秒、モード4 No.7で出力)。最短走行ではもう取っているのでそのまま
	}
	if (G_First_Turn_Accel) {
		float vt = Short_First_Turn_Accel_V(kind, V);
		float v = Motor_First_Turn_Accel(SHORT_START_X, p->pre, p->post, p->v,
				dir == 0 ? p->ang : -p->ang, p->w, p->w_ac, naname, dir, vt,
				SHORT_FIRST_TURN_AC);
		Suction_Boost_End();
		return v;
	}
	Motor_Wallcut_ST_Accel(0, v1, SHORT_FIRST_AC, SHORT_START_X, p->pre, dir);
	Motor_Sula_COS(v1, dir == 0 ? p->ang : -p->ang, p->w * k, p->w_ac * k * k);
	if (naname) {
		Motor_Wallcut_END_NANAME_Accel(v1, V, SHORT_FIRST_AC, p->post, dir);
	} else {
		Motor_Wallcut_END_Accel(v1, V, SHORT_FIRST_AC, p->post, dir);
	}
	Suction_Boost_End();
	return G_Motor_V_Target;
}

/* 経路の f 番目の命令が最初のターンとしてどれか(kind: FIRST_TURN_*、dir: 0 左 / 1 右)。
 * どれにも当てはまらなければ0を返す */
static int Short_First_Turn_Kind(int f, int *kind, int *dir) {
	int code = G_Short_Pass_NANAME[f];
	int cp = G_Short_Pass_CP[f];
	if (code <= -4 && code > -50 && (cp == -4 || cp == -6)) {
		*kind = FIRST_TURN_BIG90;
		*dir = (cp == -4) ? 0 : 1;
	} else if (code <= -4 && code > -50 && (cp == -5 || cp == -7)) {
		*kind = FIRST_TURN_BIG180;
		*dir = (cp == -5) ? 0 : 1;
	} else if (code == -51 || code == -53) {
		*kind = FIRST_TURN_IN45;
		*dir = (code == -51) ? 0 : 1;
	} else if (code == -52 || code == -54) {
		*kind = FIRST_TURN_IN135;
		*dir = (code == -52) ? 0 : 1;
	} else {
		return 0;
	}
	return 1;
}

/* Short_First_Turn_Run(kind, dir, V) の後距離の終わりで出せる速度(V を超えない) */
static float Short_First_Turn_Reach(int kind, float V) {
	if (G_First_Turn_Accel) {
		return Short_First_Turn_Accel_V(kind, V);
	}
	const FirstTurnParam *p = &G_First_Turn[kind];
	float v1 = p->v;
	float v_reach = sqrt(4 * SHORT_FIRST_AC * (SHORT_START_X + p->pre) / PI);
	if (v1 > v_reach) {
		v1 = v_reach;
	}
	if (v1 > V) {
		v1 = V;
	}
	float v_end = sqrt(v1 * v1 + 4 * SHORT_FIRST_AC * p->post / PI);
	return (v_end < V) ? v_end : V;
}

static float Short_First_Turn(float V, int f) {
	int kind, dir;
	if (!Short_First_Turn_Kind(f, &kind, &dir)) {
		//ここには来ないはず(test_shortpath で確認)。曲がらずに区画中心まで出るだけにする
		Motor_trapezoid_PID(0, V, V, 60000, SHORT_START_X);
		return V;	//命令は飛ばさない(ループでいつもどおり走る)
	}
	float v = Short_First_Turn_Run(kind, dir, V);
	G_Short_Pass_NANAME[f] = -1;	//最初のターンは走ったので飛ばす
	return v;
}

/* 最短走行の斜めの直線の加速度(2026-10-10)。0 なら各走行の既定値
 * (2000・2400・ターンごと: 直線の加速度 - 5000、2700: 直線の加速度 - 10000)。
 * モード1は番号ごとに main.c で設定し、走り終わったら 0 に戻す */
int G_Short_AC_NANAME = 0;
static float Short_AC_Naname(int AC, int def_minus) {
	return (G_Short_AC_NANAME > 0) ? G_Short_AC_NANAME : AC - def_minus;
}

/* 最初のターンの直後の直線(vs < V のとき)。vs から V までは加速度 SHORT_FIRST_AC で上げ、
 * 残りの距離を返す(残りはいつもどおり V から走る)。直線全体を強い加速度にすると、
 * 最高速への加速と減速まで強くなるので、V に届くまでの区間だけにする。
 * 直線が短くて V に届かないときは、直線の終わりで目標速度が V へ跳ぶ(以前と同じ) */
static float Short_Catchup(float vs, float V, float X, int naname) {
	if (vs >= V) {
		return X;
	}
	float d = PI * (V * V - vs * vs) / (4 * SHORT_FIRST_AC) + 1;
	if (d > X) {
		d = X;
	}
	if (naname) {
		Motor_NANAME_PID(vs, V, V, SHORT_FIRST_AC, d);
	} else {
		Motor_trapezoid_PID(vs, V, V, SHORT_FIRST_AC, d);
	}
	return X - d;
}

/* 2000(Short_NANAME_Move2000 / Short_Dijkstra_Move2000)の走り出し(2026-10-09)。
 * 最初のターン専用の動き(Short_First_Turn)を使わず、以前と同じにする:
 * 最初がターンなら 10mm で 2000 にしてから、ループでいつもの2000のターンを走る。
 * 最初が直線なら Short_Start() と同じ(区画の境目までの114mmでなめらかに加速) */
static float Short_Start(float V);
static float Short_Start_NoFirstTurn(float V) {
	int f = 0;
	while (G_Short_Pass_NANAME[f] == -1) {
		f++;
	}
	if (G_Short_Pass_NANAME[f] >= 1) {
		return Short_Start(V);
	}
	Motor_trapezoid_PID(0, V, V, 70000, 10);
	return V;
}

/* スタート区画からの走り出し(2026-10-07)。戻り値は最初の直線の始めの速度。
 * 最初が直線: スタート位置(区画中心の24mm後ろ)から区画の境目までの 90+24 mm で、
 *   0→V までなめらかに加速し、その半区画ぶんを走ったことにする。
 *   cos加速の加速距離は π V² / (4 Ac) なので Ac = π V² / (4 × 114)
 *   (1000: 約6900、2000: 約27600、2400: 約39700、2700: 約50200)。
 *   1.5区画以上の直線も、ここで加速してから残りを走る(以前は10mmで加速しようとして届かず、14mm短かった)。
 * 最初がターン: Short_First_Turn() */
static float Short_Start(float V) {
	/* 圧縮で0になった直線は -1(飛ばす)として残るので、最初が
	 * ターンの経路では先頭が -1 になり、ターンは2番目以降にある */
	int f = 0;
	while (G_Short_Pass_NANAME[f] == -1) {
		f++;
	}
	if (G_Short_Pass_NANAME[f] >= 1) {
		const float d = 90 + SHORT_START_X;
		float ac = PI * V * V / (4 * d) + 1;
		Motor_trapezoid_PID(0, V, V, ac, d);
		if (G_Short_Pass_NANAME[f] == 1) {
			G_Short_Pass_NANAME[f] = -1;
		} else {
			G_Short_Pass_NANAME[f] -= 1;
		}
		return V;
	}
	return Short_First_Turn(V, f);
}

void Short_NANAME_Move1000(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	Maze_Shortest_Calculation();
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	if (!Short_Pass_Check()) {
		return;
	}

	Motor_Setup_Voltage();
	Suction_Start(15);
	Fun_Flag_OFF();
	HAL_Delay(500);

	Motor_Setup();
	float v_in = Short_Start(1000);	//最初の直線の始めの速度

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		float vs = v_in;	//この区間の始めの速度(最初のターンの直後だけVより遅い)
		if (G_Short_Pass_NANAME[i] != -1) {	//飛ばす命令(-1)では持ち越す
			v_in = 1000;
		}
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			{
				float rest = Short_Catchup(vs, 1000, 90 * G_Short_Pass_NANAME[i], 0);
				if (rest > 0) {
					Motor_trapezoid_PID(1000, MAX, 1000, AC, rest);
				}
			}
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_CP[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(1000, 48, 0);
				Motor_Sula_COS(1000, 88, 600, 10000);
				Motor_Wallcut_END(1000, 35, 0);
			} else if (G_Short_Pass_CP[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(1000, 48, 1);
				Motor_Sula_COS(1000, -88, 600, 10000);
				Motor_Wallcut_END(1000, 35, 1);
			} else if (G_Short_Pass_CP[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(1000, 60, 0);
				Motor_Sula_COS(1000, 179, 650, 10000);
				Motor_Wallcut_END(1000, 40, 0);
			} else if (G_Short_Pass_CP[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(1000, 60, 1);
				Motor_Sula_COS(1000, -179, 650, 10000);
				Motor_Wallcut_END(1000, 40, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(1000, 15, 0);
				Motor_Sula_COS(1000, 44, 750, 10000);
				Motor_Wallcut_END_NANAME(1000, 42, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(1000, 38, 0);
				Motor_Sula_COS(1000, 134, 800, 10000);
				Motor_Wallcut_END_NANAME(1000, 8, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(1000, 15, 1);
				Motor_Sula_COS(1000, -44, 750, 10000);
				Motor_Wallcut_END_NANAME(1000, 42, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(1000, 38, 1);
				Motor_Sula_COS(1000, -134, 800, 10000);
				Motor_Wallcut_END_NANAME(1000, 8, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(1000, 73, 0);
				Motor_Sula_COS(1000, 41.5, 700, 20000);
				Motor_Wallcut_END(1000, 30, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(1000, 25, 0);
				Motor_Sula_COS(1000, 132, 700, 14000);
				Motor_Wallcut_END(1000, 25, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(1000, 73, 1);
				Motor_Sula_COS(1000, -41.5, 700, 20000);
				Motor_Wallcut_END(1000, 30, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(1000, 25, 1);
				Motor_Sula_COS(1000, -132, 700, 14000);
				Motor_Wallcut_END(1000, 25, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(1000, 37, 0);
				//Motor_NANAME_PID(500, 500, 500, 5000, 10);
				Motor_Sula_COS(1000, 88.5, 800, 20000);
				Motor_Wallcut_END_NANAME(1000, 27, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(1000, 37, 1);
				//Motor_NANAME_PID(500, 500, 500, 5000, 10);
				Motor_Sula_COS(1000, -88.5, 800, 20000);
				Motor_Wallcut_END_NANAME(1000, 27, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				{
					float rest = Short_Catchup(vs, 1000, 127.3 * G_Short_Pass_NANAME[i] / -50, 1);
					if (rest > 0) {
						Motor_NANAME_PID(1000, MAX, 1000, Short_AC_Naname(AC, 5000), rest);
					}
				}

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(1000, 1000, 0, 10000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}

/* ---- 2000のターンの値(2026-10-09) ----
 * 以前は Short_NANAME_Move2000(BFS)・Short_Dijkstra_Move2000(ダイクストラ)・モード6(ターン単体テスト)に
 * 別々に書かれていて、ダイクストラだけ値が違っていた。BFS(=モード6)の値で1つの表にまとめた。
 * モード6 No.1〜7(Turn2000_Test、左旋回、壁切れを使わず決まった距離)で合わせて、ここを直す。
 * 前距離・後距離[mm]、角度[deg]、最大角速度[deg/s]、角加速度[deg/s^2] */
Turn2000_Param G_Turn2000[T2000_NUM] = {
	//          前の壁切れ   前距離 角度  最大角速度 角加速度  後の壁切れ   後距離
	[T2000_BIG90]  = { WC2000_NORMAL, 10,  90, 2000,  40000, WC2000_NORMAL, 75 },	//大回り90
	[T2000_BIG180] = { WC2000_NORMAL,  5, 180, 1220,  40000, WC2000_NORMAL, 73 },	//大回り180
	[T2000_IN45]   = { WC2000_NORMAL,  5,  45, 1900, 120000, WC2000_NANAME, 110 },	//斜め入り45
	[T2000_IN135]  = { WC2000_NORMAL, 31, 135, 1500,  80000, WC2000_NANAME, 80 },	//斜め入り135
	[T2000_OUT45]  = { WC2000_NANAME, 8,  45, 1500,  40000, WC2000_NORMAL, 33 },	//斜め出45
	[T2000_OUT135] = { WC2000_NANAME, 5, 135, 1350,  80000, WC2000_NORMAL, 80 },	//斜め出135
	[T2000_V90]    = { WC2000_NANAME, 15,  90, 2000, 130000, WC2000_NANAME, 90},	//V90
};

/* 2000のターンを1つ走る(壁切れあり)。kind: T2000_*、dir: 0 左 / 1 右 */
void Turn2000_Run(int kind, int dir) {
	const Turn2000_Param *p = &G_Turn2000[kind];
	if (p->st_kind == WC2000_NANAME) {
		Motor_Wallcut_ST_NANAME(2000, p->st, dir);
	} else {
		Motor_Wallcut_ST(2000, p->st, dir);
	}
	Motor_Sula_COS(2000, dir == 0 ? p->ang : -p->ang, p->w, p->w_ac);
	if (p->end_kind == WC2000_NANAME) {
		Motor_Wallcut_END_NANAME(2000, p->end, dir);
	} else {
		Motor_Wallcut_END(2000, p->end, dir);
	}
}

/* 斜めの命令(-51〜-54、-61〜-66)をターンの種類と向きにする。ターンでなければ0 */
static int Turn2000_Naname_Kind(int code, int *kind, int *dir) {
	switch (code) {
	case -51: *kind = T2000_IN45;   *dir = 0; return 1;
	case -53: *kind = T2000_IN45;   *dir = 1; return 1;
	case -52: *kind = T2000_IN135;  *dir = 0; return 1;
	case -54: *kind = T2000_IN135;  *dir = 1; return 1;
	case -61: *kind = T2000_OUT45;  *dir = 0; return 1;
	case -63: *kind = T2000_OUT45;  *dir = 1; return 1;
	case -62: *kind = T2000_OUT135; *dir = 0; return 1;
	case -64: *kind = T2000_OUT135; *dir = 1; return 1;
	case -65: *kind = T2000_V90;    *dir = 0; return 1;
	case -66: *kind = T2000_V90;    *dir = 1; return 1;
	default:  return 0;
	}
}

/* 大回りの命令(-4 左90、-6 右90、-5 左180、-7 右180) */
static void Turn2000_Big(int code) {
	if (code == -4) {
		Turn2000_Run(T2000_BIG90, 0);
	} else if (code == -6) {
		Turn2000_Run(T2000_BIG90, 1);
	} else if (code == -5) {
		Turn2000_Run(T2000_BIG180, 0);
	} else if (code == -7) {
		Turn2000_Run(T2000_BIG180, 1);
	}
}

/* モード6 No.1〜7: ターンを1つ、G_Turn2000 の値で走る(左旋回)。壁切れを使わず、前距離・後距離は決まった距離。
 * 斜めから入るターン(斜め出45/135・V90)は、機体を斜めに置いてスタートし、斜め1区間(127.3mm)で2000まで上げてから走る */
void Turn2000_Test(int kind) {
	const Turn2000_Param *p = &G_Turn2000[kind];
	if (p->st_kind == WC2000_NANAME) {
		Motor_trapezoid(0, 2000, 2000, 25000, 127.3);	//25000なら127.3mmで2000に届く
		Motor_trapezoid(2000, 2000, 2000, 15000, p->st);
	} else {
		Motor_trapezoid_PID(0, 2000, 2000, 20000, 90 + 90 + 180);
		Motor_trapezoid_PID(2000, 2000, 2000, 15000, p->st);
	}
	Motor_Sula_COS(2000, p->ang, p->w, p->w_ac);
	Motor_trapezoid(2000, 2000, 2000, 15000, p->end);
	if (p->end_kind == WC2000_NANAME) {
		Motor_trapezoid(2000, 2000, 0, 25000, 127.3);
	} else {
		Motor_trapezoid(2000, 2000, 0, 20000, 180);
	}
}

void Short_NANAME_Move2000(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();

	Maze_Shortest_Calculation();
	//Maze_Dijkstra_Calculation();

	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	if (!Short_Pass_Check()) {
		return;
	}
	Suction_Start(50);
	HAL_Delay(500);

	Motor_Setup();
	float v_in = Short_Start_NoFirstTurn(2000);	//最初がターンでも最初のターン専用の動きは使わない

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		float vs = v_in;	//この区間の始めの速度(最初のターンの直後だけVより遅い)
		if (G_Short_Pass_NANAME[i] != -1) {	//飛ばす命令(-1)では持ち越す
			v_in = 2000;
		}
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			//Suction_change(30);
			{
				float rest = Short_Catchup(vs, 2000, 90 * G_Short_Pass_NANAME[i], 0);
				if (rest > 0) {
					Motor_trapezoid_Asymmetric_PID(2000, MAX, 2000, AC, rest);
				}
			}
			//Suction_change(50);
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			Turn2000_Big(G_Short_Pass_NANAME[i]);	//大回り(G_Turn2000)
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			int kind, dir;
			if (Turn2000_Naname_Kind(G_Short_Pass_NANAME[i], &kind, &dir)) {
				Turn2000_Run(kind, dir);	//斜め入り・斜め出・V90(G_Turn2000)
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				{
					float rest = Short_Catchup(vs, 2000, 127.3 * G_Short_Pass_NANAME[i] / -50, 1);
					if (rest > 0) {
						Motor_NANAME_PID(2000, MAX, 2000, Short_AC_Naname(AC, 5000), rest);
					}
				}

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(2000, 2000, 0, 15000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}

/* 1にすると Short_NANAME_Move2400 がダイクストラの経路を走る
 * (Short_Dijkstra_Move2400 から使う) */
static int Short_Use_Dijkstra = 0;

void Short_NANAME_Move2400(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	if (Short_Use_Dijkstra) {
		Maze_Dijkstra_Calculation();
	} else {
		Maze_Shortest_Calculation();
	}
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	if (!Short_Pass_Check()) {
		return;
	}
	Suction_Start(70);
	HAL_Delay(500);

	Motor_Setup();
	float v_in = Short_Start(2400);	//最初の直線の始めの速度
	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		float vs = v_in;	//この区間の始めの速度(最初のターンの直後だけVより遅い)
		if (G_Short_Pass_NANAME[i] != -1) {	//飛ばす命令(-1)では持ち越す
			v_in = 2400;
		}
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			{
				float rest = Short_Catchup(vs, 2400, 90 * G_Short_Pass_NANAME[i], 0);
				if (rest > 0) {
					Motor_trapezoid_Asymmetric_PID(2400, MAX, 2400, AC, rest);
				}
			}
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_CP[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(2400, 25, 0);
				Motor_Sula_COS(2400, 90, 1600, 80000);
				Motor_Wallcut_END(2400, 100, 0);
			} else if (G_Short_Pass_CP[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(2400, 25, 1);
				Motor_Sula_COS(2400, -90, 1600, 80000);
				Motor_Wallcut_END(2400, 100, 1);
			} else if (G_Short_Pass_CP[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(2400, 15, 0);
				Motor_Sula_COS(2400, 180, 1500, 70000);
				Motor_Wallcut_END(2400, 88, 0);
			} else if (G_Short_Pass_CP[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(2400, 15, 1);
				Motor_Sula_COS(2400, -180, 1500, 70000);
				Motor_Wallcut_END(2400, 88, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(2400, 3, 0);
				Motor_Sula_COS(2400, 45, 2500, 160000);
				Motor_Wallcut_END_NANAME(2400, 113, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(2400, 15, 0);
				Motor_Sula_COS(2400, 135, 2000, 55000);
				Motor_Wallcut_END_NANAME(2400, 85, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(2400, 3, 1);
				Motor_Sula_COS(2400, -45, 2500, 160000);
				Motor_Wallcut_END_NANAME(2400, 113, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(2400, 15, 1);
				Motor_Sula_COS(2400, -135, 2000, 55000);
				Motor_Wallcut_END_NANAME(2400, 85, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(2400, 8, 0);
				Motor_Sula_COS(2400, 45, 1550, 70000);
				Motor_Wallcut_END(2400, 40, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(2400, 17, 0);
				Motor_Sula_COS(2400, 135, 2100, 80000);
				Motor_Wallcut_END(2400, 123, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(2400, 8, 1);
				Motor_Sula_COS(2400, -45, 1550, 70000);
				Motor_Wallcut_END(2400, 40, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(2400, 17, 1);
				Motor_Sula_COS(2400, -135, 2100, 80000);
				Motor_Wallcut_END(2400, 123, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(2400, 5, 0);
				//Motor_NANAME_PID(500, 500, 500, 5000, 10);
				Motor_Sula_COS(2400, 87, 2600, 150000);
				Motor_Wallcut_END_NANAME(2400, 90, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(2400, 5, 1);
				//Motor_NANAME_PID(500, 500, 500, 5000, 10);
				Motor_Sula_COS(2400, -87, 2600, 150000);
				Motor_Wallcut_END_NANAME(2400, 90, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				{
					float rest = Short_Catchup(vs, 2400, 127.3 * G_Short_Pass_NANAME[i] / -50, 1);
					if (rest > 0) {
						Motor_NANAME_PID(2400, MAX, 2400, Short_AC_Naname(AC, 5000), rest);
					}
				}

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(2400, 2400, 0, 30000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}

/* ダイクストラの経路を、コーナー2400の Short_NANAME_Move2400 と同じ動きで走る。
 * 命令列の形はBFSと同じなので、経路の作り方だけを切り替える */
void Short_Dijkstra_Move2400(int MAX, int AC) {
	Short_Use_Dijkstra = 1;
	Short_NANAME_Move2400(MAX, AC);
	Short_Use_Dijkstra = 0;
}

void Short_NANAME_Move2700(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	Maze_Shortest_Calculation();
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	if (!Short_Pass_Check()) {
		return;
	}
	Suction_Start(85);
	HAL_Delay(500);

	Motor_Setup();
	float v_in = Short_Start(2700);	//最初の直線の始めの速度
	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		float vs = v_in;	//この区間の始めの速度(最初のターンの直後だけVより遅い)
		if (G_Short_Pass_NANAME[i] != -1) {	//飛ばす命令(-1)では持ち越す
			v_in = 2700;
		}
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			{
				float rest = Short_Catchup(vs, 2700, 90 * G_Short_Pass_NANAME[i], 0);
				if (rest > 0) {
					Motor_trapezoid_PID(2700, MAX, 2700, AC, rest);
				}
			}
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_CP[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(2700, 25, 0);
				Motor_Sula_COS(2700, 90, 2300, 70000);
				Motor_Wallcut_END(2700, 68, 0);
			} else if (G_Short_Pass_CP[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(2700, 25, 1);
				Motor_Sula_COS(2700, -90, 2300, 70000);
				Motor_Wallcut_END(2700, 68, 1);
			} else if (G_Short_Pass_CP[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(2700, 10, 0);
				Motor_Sula_COS(2700, 180, 1700, 40000);
				Motor_Wallcut_END(2700, 58, 0);
			} else if (G_Short_Pass_CP[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(2700, 10, 1);
				Motor_Sula_COS(2700, -180, 1700, 40000);
				Motor_Wallcut_END(2700, 58, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(2700, 2, 0);
				Motor_Sula_COS(2700, 45, 2850, 100000);
				Motor_Wallcut_END_NANAME(2700, 85, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(2700, 11, 0);
				Motor_Sula_COS(2700, 135, 2200, 60000);
				Motor_Wallcut_END_NANAME(2700, 70, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(2700, 2, 1);
				Motor_Sula_COS(2700, -45, 2850, 100000);
				Motor_Wallcut_END_NANAME(2700, 85, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(2700, 11, 1);
				Motor_Sula_COS(2700, -135, 2200, 60000);
				Motor_Wallcut_END_NANAME(2700, 70, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(2700, 12, 0);
				Motor_Sula_COS(2700, 45, 1800, 70000);
				Motor_Wallcut_END(2700, 10, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(2700, 23, 0);
				Motor_Sula_COS(2700, 135, 2000, 100000);
				Motor_Wallcut_END(2700, 117, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(2700, 12, 1);
				Motor_Sula_COS(2700, -45, 1800, 70000);
				Motor_Wallcut_END(2700, 10, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(2700, 23, 1);
				Motor_Sula_COS(2700, -135, 2000, 100000);
				Motor_Wallcut_END(2700, 117, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(2700, 18, 0);
				Motor_Sula_COS(2700, 82, 2800, 180000);
				Motor_Wallcut_END_NANAME(2700, 97, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(2700, 18, 1);
				Motor_Sula_COS(2700, -82, 2800, 180000);
				Motor_Wallcut_END_NANAME(2700, 97, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				{
					float rest = Short_Catchup(vs, 2700, 127.3 * G_Short_Pass_NANAME[i] / -50, 1);
					if (rest > 0) {
						Motor_NANAME_PID(2700, MAX, 2700, Short_AC_Naname(AC, 10000), rest);
					}
				}

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(2700, 2700, 0, 33000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}

void Short_Dijkstra_Move2000(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();

	//Maze_Shortest_Calculation();
	Maze_Dijkstra_Calculation();

	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	if (!Short_Pass_Check()) {
		return;
	}
	Suction_Start(50);
	HAL_Delay(500);

	Motor_Setup();
	float v_in = Short_Start_NoFirstTurn(2000);	//最初がターンでも最初のターン専用の動きは使わない

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		float vs = v_in;	//この区間の始めの速度(最初のターンの直後だけVより遅い)
		if (G_Short_Pass_NANAME[i] != -1) {	//飛ばす命令(-1)では持ち越す
			v_in = 2000;
		}
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			{
				float rest = Short_Catchup(vs, 2000, 90 * G_Short_Pass_NANAME[i], 0);
				if (rest > 0) {
					Motor_trapezoid_Asymmetric_PID(2000, MAX, 2000, AC, rest);	//BFSと同じ(減速は加速の2倍)
				}
			}
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			Turn2000_Big(G_Short_Pass_CP[i]);	//大回り(G_Turn2000)
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			int kind, dir;
			if (Turn2000_Naname_Kind(G_Short_Pass_NANAME[i], &kind, &dir)) {
				Turn2000_Run(kind, dir);	//斜め入り・斜め出・V90(G_Turn2000)
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				{
					float rest = Short_Catchup(vs, 2000, 127.3 * G_Short_Pass_NANAME[i] / -50, 1);
					if (rest > 0) {
						Motor_NANAME_PID(2000, MAX, 2000, Short_AC_Naname(AC, 5000), rest);
					}
				}

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(2000, 2000, 0, 20000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}


/*
 * ターンごとに通過速度を変える最短走行(2026-10-07、2400基準に作り直し)。
 *
 * Short_NANAME_Move2400 と同じ走り方で、ターンの種類ごとに通過速度を持つ。
 * 2400は一番きついターン(V90)に合わせた速度なので、横加速度に余裕のあるターンは速く、
 * きついターンは遅く回る。
 *
 * ターンのパラメータは Short_NANAME_Move2400 の値(TURNV_V_BASE=2400で調整済み)を基準にし、
 * 通過速度Vに合わせて 角速度 x (V/2400)、角加速度 x (V/2400)^2 に換算する。
 * こうすると理想的には同じ軌跡(同じ旋回半径)になる。横加速度は (V/2400)^2 倍になる。
 * 前後のオフセット距離は理想的には変わらないが、実機では滑りや遅れで変わるので、
 * 速度を変えたターンは offset_st / offset_end を調整する(モード10 No.1〜7)。
 *
 * 速度のつなぎ方(SpeedPlan.c):
 * - 間に直線があるとき: 後距離はそのターンの速度で一定に走り、直線で次のターンの速度に合わせる。
 *   直線が短くて加減速しきれなければ、その範囲まで前後のターンの速度を下げる。
 * - ターンが直線を挟まずにつながるとき: 前のターンの後距離のうちに、次のターンの速度へ
 *   加速度 TURNV_AC_BACK で変える(後距離の8割で変えきる。壁切れで後距離が早く終わる分の余裕)。
 *   次のターンの前距離は、そのターンの速度で一定に走る。
 *   変えきれないときは、変えきれる速度まで速い方のターンを下げる。
 *
 * TurnV_Table の v を全部 2400 にすると Short_NANAME_Move2400 と同じ走りになる。
 *
 * 速度の初期値の考え方(README「ターンごとの通過速度」):
 *   2400 のパラメータでのピーク横加速度は V90 が最大(約109m/s^2)。V90 の限界を2200とすると
 *   限界は約91.5m/s^2 で、そこに届く速度は 大回り90 約2800、大回り180 約2900、斜め出45 約2980、
 *   斜め入り45 約2420、斜め出135 約2450、斜め入り135 約2510。
 *   初期値はそこから控えめにした(大回り180はほかの人も遅めにしているので特に控えめ)。
 */
#define TURNV_V_BASE 2400.0f
#define TURNV_V_START 2400.0f	/* スタート区間の終わりの速度の上限 */
#define TURNV_V_GOAL 2400.0f	/* ゴール停止区間に入る速度の上限 */
#define TURNV_AC_BACK 30000.0f	/* 後距離で次のターンの速度へ変えるときの加速度 */

#define WC_NORMAL 0
#define WC_NANAME 1

typedef struct {
	float v;			/* 通過速度[mm/s]。これを調整する */
	float angle;		/* 左旋回の角度[deg]。右は符号を反転 */
	float w_max;		/* V_BASEでの最大角速度 */
	float w_ac;			/* V_BASEでの角加速度 */
	int st_kind;		/* 前オフセットの壁切れ: WC_NORMAL / WC_NANAME */
	float offset_st;
	int end_kind;		/* 後オフセット */
	float offset_end;
} TurnV_Param;

enum {
	TV_BIG90,		/* 大回り90  (-4 / -6) */
	TV_BIG180,		/* 大回り180 (-5 / -7) */
	TV_IN45,		/* 斜め入り45  (-51 / -53) */
	TV_IN135,		/* 斜め入り135 (-52 / -54) */
	TV_OUT45,		/* 斜め出45  (-61 / -63) */
	TV_OUT135,		/* 斜め出135 (-62 / -64) */
	TV_V90,			/* V90 (-65 / -66) */
	TV_NUM
};

/* 角度・角速度・角加速度・オフセットは Short_NANAME_Move2400 と同じ値 */
static TurnV_Param TurnV_Table[TV_NUM] = {
	/*            v     angle  w_max  w_ac     st_kind    st   end_kind   end */
	[TV_BIG90]  = {2600,  90, 1600,  80000, WC_NORMAL, 25, WC_NORMAL, 100},
	[TV_BIG180] = {2500, 180, 1500,  70000, WC_NORMAL, 15, WC_NORMAL,  88},
	[TV_IN45]   = {2400,  45, 2500, 160000, WC_NORMAL,  3, WC_NANAME, 113},
	[TV_IN135]  = {2400, 135, 2000,  55000, WC_NORMAL, 15, WC_NANAME,  85},
	[TV_OUT45]  = {2600,  45, 1550,  70000, WC_NANAME,  8, WC_NORMAL,  40},
	[TV_OUT135] = {2400, 135, 2100,  80000, WC_NANAME, 17, WC_NORMAL, 123},
	[TV_V90]    = {2200,  87, 2600, 150000, WC_NANAME,  5, WC_NANAME,  90},
};

/* パスのコード -> テーブル番号と向き(0:左 1:右)。ターンでなければ -1 */
static int TurnV_Lookup(int16_t code, int *dir) {
	switch (code) {
	case -4:  *dir = 0; return TV_BIG90;
	case -6:  *dir = 1; return TV_BIG90;
	case -5:  *dir = 0; return TV_BIG180;
	case -7:  *dir = 1; return TV_BIG180;
	case -51: *dir = 0; return TV_IN45;
	case -53: *dir = 1; return TV_IN45;
	case -52: *dir = 0; return TV_IN135;
	case -54: *dir = 1; return TV_IN135;
	case -61: *dir = 0; return TV_OUT45;
	case -63: *dir = 1; return TV_OUT45;
	case -62: *dir = 0; return TV_OUT135;
	case -64: *dir = 1; return TV_OUT135;
	case -65: *dir = 0; return TV_V90;
	case -66: *dir = 1; return TV_V90;
	default:  return -1;
	}
}

static float TurnV_Speed(int16_t code) {
	int dir;
	int k = TurnV_Lookup(code, &dir);
	return (k >= 0) ? TurnV_Table[k].v : TURNV_V_BASE;
}

static float TurnV_Back(int16_t code) {
	int dir;
	int k = TurnV_Lookup(code, &dir);
	return (k >= 0) ? TurnV_Table[k].offset_end : 0;
}

/* ターンを1つ走る。V: 前距離とターンの速度、V_out: 後距離の終わりの速度
 * (V と違えば後距離のうちに加速度 TURNV_AC_BACK で変える) */
static void TurnV_Run(int16_t code, float V, float V_out) {
	int dir;
	int k = TurnV_Lookup(code, &dir);
	if (k < 0) {
		return;
	}
	const TurnV_Param *p = &TurnV_Table[k];
	float s = V / TURNV_V_BASE;
	float angle = (dir == 0) ? p->angle : -p->angle;

	if (p->st_kind == WC_NANAME) {
		Motor_Wallcut_ST_NANAME(V, p->offset_st, dir);
	} else {
		Motor_Wallcut_ST(V, p->offset_st, dir);
	}
	Motor_Sula_COS(V, angle, p->w_max * s, p->w_ac * s * s);
	if (V_out == V) {
		if (p->end_kind == WC_NANAME) {
			Motor_Wallcut_END_NANAME(V, p->offset_end, dir);
		} else {
			Motor_Wallcut_END(V, p->offset_end, dir);
		}
	} else {
		if (p->end_kind == WC_NANAME) {
			Motor_Wallcut_END_NANAME_Accel(V, V_out, TURNV_AC_BACK, p->offset_end, dir);
		} else {
			Motor_Wallcut_END_Accel(V, V_out, TURNV_AC_BACK, p->offset_end, dir);
		}
	}
}

/* 1にすると Short_NANAME_MoveTurnV がダイクストラの経路を走る
 * (Short_Dijkstra_MoveTurnV から使う) */
static int TurnV_Use_Dijkstra = 0;

void Short_NANAME_MoveTurnV(int MAX, int AC) {
	static SpeedPlan_t plan;

	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	if (TurnV_Use_Dijkstra) {
		Maze_Dijkstra_Calculation();
	} else {
		Maze_Shortest_Calculation();
	}
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	if (!Short_Pass_Check()) {
		return;
	}

	/* 走り出し(Short_Start と同じ動き)。速度計画より前に命令列を書き換えておく。
	 * 最初が直線: 区画の境目までの半区画ぶんをスタート区間に含める。
	 * 最初がターン: 最初のターン(Short_First_Turn_Run)をスタート区間に含め、
	 *   後距離の終わりで出せる速度をスタート区間の終わりの速度の上限にする */
	int f = 0;
	while (G_Short_Pass_NANAME[f] == -1) {
		f++;
	}
	enum { START_STRAIGHT, START_TURN, START_OTHER } start_mode = START_OTHER;
	int first_kind = 0, first_dir = 0;
	float v_start = TURNV_V_START;
	if (G_Short_Pass_NANAME[f] >= 1) {
		start_mode = START_STRAIGHT;
		if (G_Short_Pass_NANAME[f] == 1) {
			G_Short_Pass_NANAME[f] = -1;
		} else {
			G_Short_Pass_NANAME[f] -= 1;
		}
	} else if (Short_First_Turn_Kind(f, &first_kind, &first_dir)) {
		start_mode = START_TURN;
		G_Short_Pass_NANAME[f] = -1;
		v_start = Short_First_Turn_Reach(first_kind, TURNV_V_START);
	}

	plan.turn_v = TurnV_Speed;
	plan.v_max = MAX;
	plan.ac = AC;
	plan.ac_diag = Short_AC_Naname(AC, 5000);	/* Short_NANAME_Move2400 の斜め直線と同じ */
	plan.v_start = v_start;
	plan.v_goal = TURNV_V_GOAL;
	plan.turn_back = TurnV_Back;
	plan.ac_back = TURNV_AC_BACK;
	if (SpeedPlan_Make(G_Short_Pass_NANAME, &plan) != 0) {
		return;
	}

	Suction_Start(70);
	HAL_Delay(500);

	Motor_Setup();
	float v0 = plan.v_start_out;
	if (start_mode == START_TURN) {
		Short_First_Turn_Run(first_kind, first_dir, v0);
	} else if (start_mode == START_STRAIGHT) {
		const float d = 90 + SHORT_START_X;
		Motor_trapezoid_PID(0, v0, v0, PI * v0 * v0 / (4 * d) + 1, d);
	} else {	//ここには来ないはず(test_shortpath で確認)。区画中心まで出るだけにする
		Motor_trapezoid_PID(0, v0, v0, 60000, SHORT_START_X);
	}

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		int16_t code = G_Short_Pass_NANAME[i];
		switch (SpeedPlan_Kind(code)) {
		case SP_STRAIGHT:			//区間前進
			Motor_trapezoid_Asymmetric_PID(plan.v_in[i], MAX, plan.v_out[i],
					AC, 90 * code);
			break;
		case SP_DIAGONAL:			//斜め直線
			Motor_NANAME_PID(plan.v_in[i], MAX, plan.v_out[i], plan.ac_diag,
					127.3 * code / -50);
			break;
		case SP_TURN:
			TurnV_Run(code, plan.v_in[i], plan.v_out[i]);
			break;
		default:
			break;
		}
	}
	if (Failsafe_Flag() == 0) {
		float vg = plan.v_goal_in;
		Motor_trapezoid_PID(vg, vg, 0, 30000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}
}

/* ダイクストラの経路を、Short_NANAME_MoveTurnV と同じ動きで走る */
void Short_Dijkstra_MoveTurnV(int MAX, int AC) {
	TurnV_Use_Dijkstra = 1;
	Short_NANAME_MoveTurnV(MAX, AC);
	TurnV_Use_Dijkstra = 0;
}

/* ターンを1つだけ、TurnV_Table の速度とパラメータで走る(モード10 No.1〜7、左旋回)。
 * モード7(2.4m/s の旋回パターンテスト)と同じく壁切れを使わず、前距離・後距離は固定の距離で走る。
 * kind: 0 大回り90、1 大回り180、2 斜め入り45、3 斜め入り135、4 斜め出45、5 斜め出135、6 V90。
 * 斜めから入るターン(斜め出45/135・V90)は、斜め入り45(表の速度)で斜めに入ってから、
 * 斜めの直線1区間で表の速度に合わせて走る */
void TurnV_Test(int kind) {
	if (kind < 0 || kind >= TV_NUM) {
		return;
	}
	const TurnV_Param *p = &TurnV_Table[kind];
	float V = p->v;
	float s = V / TURNV_V_BASE;

	if (p->st_kind == WC_NANAME) {
		const TurnV_Param *q = &TurnV_Table[TV_IN45];
		float V1 = q->v;
		float s1 = V1 / TURNV_V_BASE;
		Motor_trapezoid_PID(0, V1, V1, 30000, 90 + 90 + 180);
		Motor_trapezoid_PID(V1, V1, V1, 15000, q->offset_st);
		Motor_Sula_COS(V1, q->angle, q->w_max * s1, q->w_ac * s1 * s1);
		Motor_trapezoid(V1, V1, V1, 15000, q->offset_end);
		Motor_NANAME_PID(V1, V, V, 30000, 127.3);
		Motor_trapezoid(V, V, V, 15000, p->offset_st);
	} else {
		Motor_trapezoid_PID(0, V, V, 30000, 90 + 90 + 180);
		Motor_trapezoid_PID(V, V, V, 15000, p->offset_st);
	}
	Motor_Sula_COS(V, p->angle, p->w_max * s, p->w_ac * s * s);
	Motor_trapezoid(V, V, V, 15000, p->offset_end);
	if (p->end_kind == WC_NANAME) {
		Motor_trapezoid(V, V, 0, 30000, 127.3 * 2);
	} else {
		Motor_trapezoid(V, V, 0, 30000, 180);
	}
}
