/*
 * wall_dist.h
 * 壁センサの値を、壁までの距離 [mm] に直す（中身と表の作り方は wall_dist.c）
 *
 * 距離は、区画のまん中にいるとき、どれも WALL_DIST_CENTER_MM（42mm）になる。
 *   横（左・右）：機体の中心線から横の壁の表面まで
 *   前（左前・右前）：タイヤの軸から前の壁の表面まで
 * 表より近い・遠いときは、表のいちばん近い・遠い距離になる。
 */
#ifndef CTRL_WALL_DIST_H
#define CTRL_WALL_DIST_H

#include <stdint.h>

#define WALL_DIST_CENTER_MM 42.0f  /* 区画のまん中にいるときの、壁までの距離 [mm]（(90 − 6) ÷ 2） */

float wall_dist_front_left_mm(int32_t value);
float wall_dist_left_mm(int32_t value);
float wall_dist_right_mm(int32_t value);
float wall_dist_front_right_mm(int32_t value);

#endif /* CTRL_WALL_DIST_H */
