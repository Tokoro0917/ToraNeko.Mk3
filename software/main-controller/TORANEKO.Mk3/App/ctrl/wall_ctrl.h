/*
 * wall_ctrl.h
 * 壁のあり・なしの判定と、壁に沿って走るための補正
 *
 * 中身（wall_ctrl.c）は、壁のあり・なし、手の判定、横の壁からのずれ（壁に沿って走る補正に使う）。
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

/* 横の壁からのずれ（区画のまん中にいるときの値との差）と、そのとき使った壁（0 なし、1 左、2 右、3 両方）。
 * + なら左の壁に近い。壁に沿って走る補正（ctrl/control.c）で使う */
float wall_ctrl_get_side_error(void);
int wall_ctrl_get_side_walls_used(void);

/* 前の壁からのずれ（前壁合わせに使う。区画のまん中にいるときの値との差）。
 *   距離のずれ：+ なら前の壁に近すぎる（下がるとよい）
 *   向きのずれ：+ なら左前のほうが近い＝右を向いている（左へ回すとよい） */
float wall_ctrl_get_front_distance_error(void);
float wall_ctrl_get_front_angle_error(void);

/* 4 つの壁センサの値（ON−OFF。LED の光の分だけ）。センサ値の表示などで使う */
int32_t wall_ctrl_get_front_left(void);
int32_t wall_ctrl_get_left(void);
int32_t wall_ctrl_get_right(void);
int32_t wall_ctrl_get_front_right(void);

/* 4 つの壁センサの値を、壁までの距離 [mm] に直したもの（ctrl/wall_dist.c。区画のまん中で 42mm） */
float wall_ctrl_get_front_left_mm(void);
float wall_ctrl_get_left_mm(void);
float wall_ctrl_get_right_mm(void);
float wall_ctrl_get_front_right_mm(void);

#endif /* CTRL_WALL_CTRL_H */
