/*
 * trajectory.h
 * 目標の速さを作る（台形加減速）
 */
#ifndef CTRL_TRAJECTORY_H
#define CTRL_TRAJECTORY_H

#include <stdbool.h>

void trajectory_reset(void);
void trajectory_start_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2);
void trajectory_update(void);
bool trajectory_is_done(void);

/* 今の目標の速さ [mm/s]・加速度 [mm/s²]・その動きで進んだ距離 [mm] */
float trajectory_get_speed_mm_s(void);
float trajectory_get_accel_mm_s2(void);
float trajectory_get_distance_mm(void);

#endif /* CTRL_TRAJECTORY_H */
