/*
 * control.c
 * 速度と角度の制御：目標の速さ・向きになるように、モーターにかける電圧を決める
 *
 * ■ 流れ（制御周期 0.5ms ごと。sys/scheduler.c から control_update() が呼ばれる）
 *   1. 目標の速さを 1 周期ぶん進める（ctrl/trajectory.c）
 *   2. 前に進む分の電圧を決める
 *        フィードフォワード：目標の速さ × FF_SPEED + 目標の加速度 × FF_ACCEL（+ 摩擦の分）
 *        フィードバック    ：速さのずれ × SPEED_KP + 位置のずれ × SPEED_KI
 *   3. 向きを保つ分の電圧を決める（今は直進だけなので、目標の角速度は 0、目標の角度は始めたときの向き）
 *        フィードバック    ：角速度のずれ × OMEGA_KP + 角度のずれ × ANGLE_KI
 *   4. 左 = 前に進む分 − 向きの分、右 = 前に進む分 + 向きの分 をモーターにかける
 *      （左回りが + なので、左に向けたいときは右を速く、左を遅くする）
 *   5. 走行ログに 1 行渡す（sys/log.c が、決めた間隔ごとに残す）
 *   6. 位置か角度のずれが大きすぎたら、フェイルセーフで止める
 * 係数は config/ctrl_gains.h にある。
 *
 * ■ ずれの足し合わせ
 * 「速さのずれ × 0.5ms」を足していくと、目標の位置と本当の位置のずれ [mm] になる。
 * 同じように「角速度のずれ × 0.5ms」を足すと、角度のずれ [deg] になる。
 * 速さのずれだけで電圧を決めると、少し遅れたぶんを取り戻せないので、足し合わせたずれも使う（I の項）。
 *
 * ■ 割り込み
 * control_start()・control_stop() はメインの処理から呼ぶ。値を書いているあいだに割り込みが
 * 中途半端な値を使わないように、先に「動いていない」にしてから値を書き、最後に「動いている」にする。
 */
#include "ctrl/control.h"

#include "config/ctrl_gains.h"
#include "config/robot_params.h"
#include "ctrl/odometry.h"
#include "ctrl/trajectory.h"
#include "ctrl/wall_ctrl.h"
#include "drv/imu.h"
#include "drv/motor.h"
#include "drv/wall.h"
#include "sys/failsafe.h"
#include "sys/log.h"

/* 割り込みの中で書き換える（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile bool s_running = false;
static volatile float s_position_error_mm = 0.0f;  /* 位置のずれ（速さのずれの足し合わせ） */
static volatile float s_angle_error_deg = 0.0f;    /* 角度のずれ（角速度のずれの足し合わせ） */
static volatile float s_start_angle_deg = 0.0f;    /* 始めたときの向き */
static volatile float s_left_voltage_v = 0.0f;     /* 最後にかけた電圧（ログ用） */
static volatile float s_right_voltage_v = 0.0f;

/* 制御を始める（止まっているところから）。今の向きを、まっすぐの向きにする */
void control_start(void)
{
  s_running = false;  /* 書いているあいだは、割り込みに使わせない */
  trajectory_reset();
  s_position_error_mm = 0.0f;
  s_angle_error_deg = 0.0f;
  s_start_angle_deg = imu_get_angle_deg();
  s_running = true;   /* 最後に、動いているにする */
}

/* 制御をやめて、モーターを止める */
void control_stop(void)
{
  s_running = false;
  motor_stop();
}

/* 制御の途中か */
bool control_is_running(void)
{
  return s_running;
}

/* 前に進む分の電圧 [V] を決める */
static float forward_voltage(void)
{
  float target_speed_mm_s = trajectory_get_speed_mm_s();
  float speed_error_mm_s = target_speed_mm_s - odometry_get_speed_mm_s();
  s_position_error_mm = s_position_error_mm + speed_error_mm_s * CTRL_PERIOD_S;

  float feedforward_v = target_speed_mm_s * FF_SPEED_V_PER_MM_S
                      + trajectory_get_accel_mm_s2() * FF_ACCEL_V_PER_MM_S2;
  if (target_speed_mm_s > 0.0f) {
    feedforward_v = feedforward_v + FF_FRICTION_V;
  } else if (target_speed_mm_s < 0.0f) {
    feedforward_v = feedforward_v - FF_FRICTION_V;
  }

  float feedback_v = speed_error_mm_s * SPEED_KP + s_position_error_mm * SPEED_KI;
  return feedforward_v + feedback_v;
}

/* 向きを保つ分の電圧 [V] を決める（+ なら左に向ける）。目標の角速度は 0（直進） */
static float turn_voltage(void)
{
  float target_omega_deg_s = 0.0f;
  float omega_error_deg_s = target_omega_deg_s - imu_get_gyro_z_dps();
  s_angle_error_deg = s_angle_error_deg + omega_error_deg_s * CTRL_PERIOD_S;

  return omega_error_deg_s * OMEGA_KP + s_angle_error_deg * ANGLE_KI;
}

/* 走行ログに、今の値を 1 行渡す */
static void record_log(void)
{
  log_values_t values = {0};  /* {0} で、全部の値を 0 にしてから入れる（壁の制御の値は、まだないので 0 のまま） */
  values.distance_mm = odometry_get_distance_mm();
  values.speed_mm_s = odometry_get_speed_mm_s();
  values.speed_target_mm_s = trajectory_get_speed_mm_s();
  values.omega_deg_s = imu_get_gyro_z_dps();
  values.omega_target_deg_s = 0.0f;
  values.angle_deg = imu_get_angle_deg() - s_start_angle_deg;
  values.sensor_left = wall_ctrl_get_left();
  values.sensor_right = wall_ctrl_get_right();
  values.voltage_left_v = s_left_voltage_v;
  values.voltage_right_v = s_right_voltage_v;
  values.battery_v = wall_get_battery_v();
  values.phase = LOG_PHASE_STRAIGHT;
  log_record(values);
}

/* 位置か角度のずれが大きすぎたら true（ぶつかった、モーターの向きが逆、など） */
static bool error_too_large(void)
{
  if (s_position_error_mm > POSITION_ERROR_LIMIT_MM || s_position_error_mm < -POSITION_ERROR_LIMIT_MM) {
    return true;
  }
  if (s_angle_error_deg > ANGLE_ERROR_LIMIT_DEG || s_angle_error_deg < -ANGLE_ERROR_LIMIT_DEG) {
    return true;
  }
  return false;
}

/* 制御周期ごとに呼ぶ（流れは先頭のコメント）。制御の途中でなければ何もしない */
void control_update(void)
{
  if (!s_running) {
    return;
  }

  trajectory_update();
  float forward_v = forward_voltage();
  float turn_v = turn_voltage();
  s_left_voltage_v = forward_v - turn_v;
  s_right_voltage_v = forward_v + turn_v;
  motor_set_voltage(s_left_voltage_v, s_right_voltage_v);
  record_log();

  if (error_too_large()) {
    s_running = false;
    failsafe_stop();  /* モーターを止めて、フェイルセーフで止まった状態にする */
  }
}

/* 位置のずれ [mm]（目標 − 測った値。+ なら目標より遅れている） */
float control_get_position_error_mm(void)
{
  return s_position_error_mm;
}

/* 角度のずれ [deg]（目標 − 測った値。+ なら目標より右を向いている） */
float control_get_angle_error_deg(void)
{
  return s_angle_error_deg;
}
