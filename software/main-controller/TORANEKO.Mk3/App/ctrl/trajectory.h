/*
 * trajectory.h
 * 目標の速さ・角速度を作る（台形加減速）
 */
#ifndef CTRL_TRAJECTORY_H
#define CTRL_TRAJECTORY_H

#include <stdbool.h>

/* 今の動きの種類 */
typedef enum {
  TRAJECTORY_STRAIGHT = 0,  /* 直進（前に進む速さを台形にする） */
  TRAJECTORY_PIVOT,         /* その場旋回（前には進まず、角速度を台形にする） */
  TRAJECTORY_SLALOM,        /* スラローム（前に進む速さは一定のまま、角速度を台形にする） */
} trajectory_kind_t;

void trajectory_reset(void);
void trajectory_start_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2);
void trajectory_start_pivot(float angle_deg, float max_omega_deg_s, float alpha_deg_s2);
void trajectory_start_slalom(float angle_deg, float speed_mm_s, float max_omega_deg_s, float alpha_deg_s2);
void trajectory_update(void);
bool trajectory_is_done(void);
trajectory_kind_t trajectory_get_kind(void);

/* 今の目標の、前に進む速さ [mm/s]・加速度 [mm/s²]（その場旋回のあいだは 0。スラロームのあいだは一定の速さ・0）。
 * その動きで進んだ距離 [mm]（直進のときだけ。旋回・スラロームのあいだは 0） */
float trajectory_get_speed_mm_s(void);
float trajectory_get_accel_mm_s2(void);
float trajectory_get_distance_mm(void);

/* 旋回・スラロームの、今の目標の角速度 [deg/s]・角加速度 [deg/s²]・その動きで回った角度 [deg]
 * （左回りが +。直進のあいだは 0） */
float trajectory_get_omega_deg_s(void);
float trajectory_get_alpha_deg_s2(void);
float trajectory_get_angle_deg(void);
float trajectory_get_overrun_mm(void);

#endif /* CTRL_TRAJECTORY_H */
