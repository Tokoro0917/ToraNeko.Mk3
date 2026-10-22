/*
 * wall_ctrl.h
 * 壁のあり・なしの判定と、壁に沿って走るための補正
 *
 * 中身（wall_ctrl.c）は、壁のあり・なしと手の判定まで作った（しきい値は仮。実機で決め直す）。壁に沿う補正はステップ6・7で作る。
 * PC のテストでは、test/ の中の仮の関数（test_maze.c の「仮の機体」、test_mode.c）がこの関数を持っている。
 *
 * ■ 壁があるか
 * 区画の入口の辺に着いたときに、その区画の前・左・右の壁があるかを答える（KOGUMA の G_Wall_data にあたる）。
 *
 * ■ 前センサに手がかざされているか
 * 前の壁よりずっと近くに何かある（手をかざしている）ときに true（KOGUMA の Sensor_Enter にあたる）。
 * モードを選んだあとのスタートに使う（sys/ui.c）。フェイルセーフの解除は、2026-10-15 からスイッチ。
 */
#ifndef CTRL_WALL_CTRL_H
#define CTRL_WALL_CTRL_H

#include <stdbool.h>
#include <stdint.h>

bool wall_ctrl_has_wall_front(void);
bool wall_ctrl_has_wall_left(void);
bool wall_ctrl_has_wall_right(void);
bool wall_ctrl_is_hand_front(void);

/* 4 つの壁センサの値（ON−OFF。LED の光の分だけ）。センサ値の表示などで使う */
int32_t wall_ctrl_get_front_left(void);
int32_t wall_ctrl_get_left(void);
int32_t wall_ctrl_get_right(void);
int32_t wall_ctrl_get_front_right(void);

#endif /* CTRL_WALL_CTRL_H */
