/*
 * wall_ctrl.h
 * 壁のあり・なしの判定と、壁に沿って走るための補正
 *
 * 中身（wall_ctrl.c）はステップ6で作る。今は、探索（sys/explore.c）とモード選択（sys/ui.c）が使う関数の形だけを先に決めておく。
 * PC のテストでは、test/ の中の仮の関数（test_maze.c の「仮の機体」、test_mode.c）がこの関数を持っている。
 *
 * ■ 壁があるか
 * 区画の入口の辺に着いたときに、その区画の前・左・右の壁があるかを答える（KOGUMA の G_Wall_data にあたる）。
 *
 * ■ 前センサに手がかざされているか
 * 前の壁よりずっと近くに何かある（手をかざしている）ときに true（KOGUMA の Sensor_Enter にあたる）。
 * モードを選んだあとのスタートと、フェイルセーフの解除に使う（sys/ui.c）。
 */
#ifndef CTRL_WALL_CTRL_H
#define CTRL_WALL_CTRL_H

#include <stdbool.h>

bool wall_ctrl_has_wall_front(void);
bool wall_ctrl_has_wall_left(void);
bool wall_ctrl_has_wall_right(void);
bool wall_ctrl_is_hand_front(void);

#endif /* CTRL_WALL_CTRL_H */
