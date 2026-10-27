/*
 * control.h
 * 速度と角度の制御：目標の速さ・向きになるように、モーターにかける電圧を決める
 */
#ifndef CTRL_CONTROL_H
#define CTRL_CONTROL_H

#include <stdbool.h>

void control_start(void);
void control_stop(void);
void control_update(void);
void control_set_wall_control(bool on);
void control_set_front_align(bool on);
bool control_is_running(void);

/* 今の動きを始めてからの、位置のずれ [mm] と角度のずれ [deg]（目標 − 測った値）。動作確認用 */
float control_get_position_error_mm(void);
float control_get_angle_error_deg(void);

#endif /* CTRL_CONTROL_H */
