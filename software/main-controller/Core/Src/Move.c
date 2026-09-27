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

int G_Pass_before;
int G_Pass_after;

int Pass_NM = 0;

void Robot_adjustment() {
	Motor_Stop();
	if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {
		Motor_Robot_Alignment();
		Motor_Stop();
	}
	Motor_trapezoid_Turn(90, 600, 20000);
	Motor_Stop();
	if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {
		Motor_Robot_Alignment();
		Motor_Stop();
	}
	Motor_trapezoid_Turn(90, 600, 20000);
	Motor_Stop();
}

void Robot_adjustment_180() {
	Suction_change(20);
	if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {
		Motor_Robot_Alignment();
		Motor_Stop();
	}
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
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			G_Just_UTurned = 1;
			Robot_adjustment();
			Motor_trapezoid_PID(0, 1000, 1000, 10000, 90);
		}
	} else {	//Uターン　一部壁無し
		Motor_trapezoid(1000, 1000, 0, 10000, 75);
		Motor_Stop();
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

	Motor_Setup_Voltage();
	Suction_Start(15);
	Fun_Flag_OFF();
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 24);
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 1000, 1000, 20000, 24);			//24
	}

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			Motor_trapezoid_PID(1000, MAX, 1000, AC,
					90 * G_Short_Pass_NANAME[i]);
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
				Motor_NANAME_PID(1000, MAX, 1000, AC - 5000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

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
	Suction_Start(50);
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 2000, 2000, 30000, 90 + 24);
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 2000, 2000, 70000, 10);			//24
	}

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			//Suction_change(30);
			Motor_trapezoid_Asymmetric_PID(2000, MAX, 2000, AC,
					90 * G_Short_Pass_NANAME[i]);
			//Suction_change(50);
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_NANAME[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(2000, 10, 0);
				Motor_Sula_COS(2000, 90, 2000, 40000);
				Motor_Wallcut_END(2000, 75, 0);
			} else if (G_Short_Pass_NANAME[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(2000, 10, 1);
				Motor_Sula_COS(2000, -90, 2000, 40000);
				Motor_Wallcut_END(2000, 75, 1);
			} else if (G_Short_Pass_NANAME[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(2000, 5, 0);
				Motor_Sula_COS(2000, 180, 1220, 40000);
				Motor_Wallcut_END(2000, 73, 0);
			} else if (G_Short_Pass_NANAME[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(2000, 5, 1);
				Motor_Sula_COS(2000, -180, 1220, 40000);
				Motor_Wallcut_END(2000, 73, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(2000, 5, 0);
				Motor_Sula_COS(2000, 45, 1900, 120000);
				Motor_Wallcut_END_NANAME(2000, 112, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(2000, 31, 0);
				Motor_Sula_COS(2000, 135, 1500, 80000);
				Motor_Wallcut_END_NANAME(2000, 98, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(2000, 5, 1);
				Motor_Sula_COS(2000, -45, 1900, 120000);
				Motor_Wallcut_END_NANAME(2000, 112, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(2000, 31, 1);
				Motor_Sula_COS(2000, -135, 1500, 80000);
				Motor_Wallcut_END_NANAME(2000, 98, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(2000, 13, 0);
				Motor_Sula_COS(2000, 45, 1500, 40000);
				Motor_Wallcut_END(2000, 25, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(2000, 10, 0);
				Motor_Sula_COS(2000, 135, 1350, 70000);
				Motor_Wallcut_END(2000, 90, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(2000, 13, 1);
				Motor_Sula_COS(2000, -45, 1500, 40000);
				Motor_Wallcut_END(2000, 25, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(2000, 10, 1);
				Motor_Sula_COS(2000, -135, 1350, 70000);
				Motor_Wallcut_END(2000, 90, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(2000, 13, 0);
				Motor_Sula_COS(2000, 90, 2000, 130000);
				Motor_Wallcut_END_NANAME(2000, 80, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(2000, 13, 1);
				Motor_Sula_COS(2000, -90, 2000, 130000);
				Motor_Wallcut_END_NANAME(2000, 80, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				Motor_NANAME_PID(2000, MAX, 2000, AC - 5000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

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
	Maze_Shortest_Calculation();
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	Suction_Start(70);
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 2400, 2400, 30000, 90 + 24);			//24
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 2400, 2400, 60000, 10);			//24
	}
	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			Motor_trapezoid_Asymmetric_PID(2400, MAX, 2400, AC,
					90 * G_Short_Pass_NANAME[i]);
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
				Motor_NANAME_PID(2400, MAX, 2400, AC - 10000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

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
	Suction_Start(85);
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 2700, 2700, 25000, 90 + 24);			//24
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 2700, 2700, 60000, 10);			//24
	}
	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			Motor_trapezoid_PID(2700, MAX, 2700, AC,
					90 * G_Short_Pass_NANAME[i]);
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
				Motor_NANAME_PID(2700, MAX, 2700, AC - 10000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

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
	Suction_Start(50);
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 2000, 2000, 30000, 90 + 24);
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 2000, 2000, 70000, 10);			//24
	}

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			Motor_trapezoid_PID(2000, MAX, 2000, AC,
					90 * G_Short_Pass_NANAME[i]);
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_CP[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(2000, 15, 0);
				Motor_Sula_COS(2000, 88.5, 1200, 30000);
				Motor_Wallcut_END(2000, 35, 0);
			} else if (G_Short_Pass_CP[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(2000, 15, 1);
				Motor_Sula_COS(2000, -88.5, 1200, 30000);
				Motor_Wallcut_END(2000, 35, 1);
			} else if (G_Short_Pass_CP[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(2000, 30, 0);
				Motor_Sula_COS(2000, 178, 1280, 30000);
				Motor_Wallcut_END(2000, 60, 0);
			} else if (G_Short_Pass_CP[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(2000, 30, 1);
				Motor_Sula_COS(2000, -178, 1280, 30000);
				Motor_Wallcut_END(2000, 60, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(2000, 8, 0);
				Motor_Sula_COS(2000, 43, 1600, 55000);
				Motor_Wallcut_END_NANAME(2000, 55, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(2000, 10, 0);
				Motor_Sula_COS(2000, 134, 1300, 50000);
				Motor_Wallcut_END_NANAME(2000, 9, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(2000, 8, 1);
				Motor_Sula_COS(2000, -43, 1600, 55000);
				Motor_Wallcut_END_NANAME(2000, 55, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(2000, 10, 1);
				Motor_Sula_COS(2000, -134, 1300, 50000);
				Motor_Wallcut_END_NANAME(2000, 9, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(2000, 22, 0);
				Motor_Sula_COS(2000, 42, 1500, 40000);
				Motor_Wallcut_END(2000, 21, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(2000, 8, 0);
				Motor_Sula_COS(2000, 131.5, 1700, 50000);
				Motor_Wallcut_END(2000, 70, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(2000, 22, 1);
				Motor_Sula_COS(2000, -42, 1500, 40000);
				Motor_Wallcut_END(2000, 21, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(2000, 8, 1);
				Motor_Sula_COS(2000, -131.5, 1700, 50000);
				Motor_Wallcut_END(2000, 70, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(2000, 7, 0);
				Motor_Sula_COS(2000, 84, 2000, 80000);
				Motor_Wallcut_END_NANAME(2000, 35, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(2000, 7, 1);
				Motor_Sula_COS(2000, -84, 2000, 80000);
				Motor_Wallcut_END_NANAME(2000, 35, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				Motor_NANAME_PID(2000, MAX, 2000, AC - 5000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

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

