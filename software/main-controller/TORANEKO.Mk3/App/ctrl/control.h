/*
 * control.h
 * 速度と角度の制御：目標の速さ・向きになるように、モーターにかける電圧を決める
 */
#ifndef CTRL_CONTROL_H
#define CTRL_CONTROL_H

#include <stdbool.h>

void control_start(void);
void control_restart(void);
void control_set_heading_hold(bool on);
void control_stop(void);
void control_update(void);
void control_set_wall_control(bool on);
void control_set_on_diagonal(bool on);
void control_set_diag_control(bool on);
void control_reset_heading_reference(void);
void control_set_heading_auto_reset(bool on);
void control_set_front_align(bool on);
bool control_is_running(void);

/* 今の動きを始めてからの、位置のずれ [mm] と角度のずれ [deg]（目標 − 測った値）。動作確認用 */
float control_get_position_error_mm(void);
float control_get_angle_error_deg(void);

/* フェイルセーフで止めたわけ（最後に止めたもの。control_start() で「なし」に戻る） */
typedef enum {
  CONTROL_FAILSAFE_NONE,      /* まだ止めていない */
  CONTROL_FAILSAFE_POSITION,  /* 位置のずれが大きすぎた */
  CONTROL_FAILSAFE_ANGLE,     /* 角度のずれが大きすぎた */
  CONTROL_FAILSAFE_SLOW,      /* 目標の速さより、ずっと遅かった（ぶつかった。F2） */
  CONTROL_FAILSAFE_STALL,     /* 走っているはずなのに、ほとんど進んでいなかった（壁を押し続けた。F3） */
} control_failsafe_reason_t;

control_failsafe_reason_t control_get_failsafe_reason(void);

#endif /* CTRL_CONTROL_H */
