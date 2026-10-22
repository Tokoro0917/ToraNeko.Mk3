/*
 * odometry.h
 * エンコーダから、タイヤの速さと走った距離を求める
 */
#ifndef CTRL_ODOMETRY_H
#define CTRL_ODOMETRY_H

void odometry_init(void);
void odometry_update(void);

/* 左・右のタイヤの速さ [mm/s]（前に進む向きが +） */
float odometry_get_left_speed_mm_s(void);
float odometry_get_right_speed_mm_s(void);

/* 機体の速さ [mm/s]（左右の平均） */
float odometry_get_speed_mm_s(void);

/* 電源を入れてから走った距離 [mm]（左右の平均。後ろに進むと減る） */
float odometry_get_distance_mm(void);

#endif /* CTRL_ODOMETRY_H */
