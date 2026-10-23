/*
 * adjust.c
 * 動きを 1 つずつ走らせて調整する（モード2・6。MODES.md）
 *
 * ■ モード6 No.1 直進（adjust_straight）
 * ゆっくり 360mm（ハーフの 4 区画）まっすぐ走って止まる。
 *   - 止まったら、本当に走った距離をものさしで測る。360mm より短ければタイヤの直径（config/robot_params.h の
 *     TIRE_DIAMETER_MM）を小さく、長ければ大きくする（新しい直径 = 今の直径 × 測った距離 ÷ 360）
 *   - 走行ログ（3ms ごと）を残すので、モード7 No.1 で PC へ出し、KOGUMA のビューアで目標の速さと
 *     測った速さを見比べて、config/ctrl_gains.h の係数を合わせる
 *   - 走り終わったら、エンコーダで測った距離と、ジャイロで測った向きのずれをシリアルに出す
 *
 * ■ モード2 No.4 その場 180° 旋回（adjust_pivot_180）
 * その場で左に 180° 回って止まる（U ターンの部品）。
 *   - 止まったら、本当に 180° 回ったかを目で見る（ジャイロの値と合っているか）
 *   - 走行ログ（1ms ごと）を残すので、目標の角速度と測った角速度を見比べて、config/ctrl_gains.h の係数を合わせる
 *   - 走り終わったら、ジャイロで測った角度と、エンコーダで測った前後のずれをシリアルに出す
 */
#include "sys/adjust.h"

#include <stdio.h>

#include "ctrl/control.h"
#include "ctrl/odometry.h"
#include "ctrl/trajectory.h"
#include "drv/imu.h"
#include "sys/failsafe.h"
#include "sys/log.h"
#include "sys/ui.h"

#define STRAIGHT_DISTANCE_MM  360.0f   /* 走る距離 [mm]（4 区画） */
#define STRAIGHT_SPEED_MM_S   200.0f   /* 最高速 [mm/s] */
#define STRAIGHT_ACCEL_MM_S2  1000.0f  /* 加速度 [mm/s²] */
#define STRAIGHT_LOG_INTERVAL_MS 3     /* 走行ログの間隔 [ms]（約 2.3 秒走るので、1000 行に入るように） */
#define SETTLE_MS             300      /* 止まってから、制御を続けて待つ時間 [ms]（ぴたりと止めるため） */

#define PIVOT_ANGLE_DEG       180.0f   /* 回る角度 [deg]（左回り） */
#define PIVOT_OMEGA_DEG_S     720.0f   /* 最高の角速度 [deg/s]（KOGUMA は 2000。まずゆっくり） */
#define PIVOT_ALPHA_DEG_S2    7200.0f  /* 角加速度 [deg/s²]（KOGUMA は 50000。まずゆっくり） */
#define PIVOT_LOG_INTERVAL_MS 1        /* 走行ログの間隔 [ms]（約 0.65 秒なので、1000 行に入る） */

/* 決めた距離・角度を走り終えるか、フェイルセーフで止まるまで待つ */
static void wait_until_done(void)
{
  while (!trajectory_is_done() && !failsafe_is_stopped()) {
    ui_wait_ms(UI_POLL_MS);
  }
}

/* 止まるまで制御を続けてから、制御とログを止める（フェイルセーフで止まったときは、すぐ止める） */
static void settle_and_stop(void)
{
  if (!failsafe_is_stopped()) {
    ui_wait_ms(SETTLE_MS);  /* 目標の速さ 0 のまま制御を続けて、ぴたりと止める */
  }
  control_stop();
  log_stop();
}

/* 終わったことを LED で知らせる。フェイルセーフで止まったときは、スイッチが押されるまで待って解除する */
static void signal_end(void)
{
  if (failsafe_is_stopped()) {
    /* 7 個全部を点滅させて、スイッチが押されるまで待ち、フェイルセーフを解除する */
    ui_wait_failsafe_reset();
    failsafe_clear();
  } else {
    ui_signal_ok();
  }
}

/* 直進の結果をシリアルに出す */
static void print_straight_result(float start_distance_mm, float start_angle_deg)
{
  float moved_mm = odometry_get_distance_mm() - start_distance_mm;
  float turned_deg = imu_get_angle_deg() - start_angle_deg;
  printf("Straight: target %.1f mm, encoder %.1f mm, angle %.2f deg\r\n",
         STRAIGHT_DISTANCE_MM, moved_mm, turned_deg);
  printf("  position error %.2f mm, angle error %.2f deg\r\n",
         control_get_position_error_mm(), control_get_angle_error_deg());
  if (failsafe_is_stopped()) {
    printf("  stopped by failsafe\r\n");
  }
}

/* モード6 No.1 直進：ゆっくり 360mm まっすぐ走って止まる（流れは先頭のコメント） */
void adjust_straight(void)
{
  float start_distance_mm = odometry_get_distance_mm();
  float start_angle_deg = imu_get_angle_deg();

  log_start(STRAIGHT_LOG_INTERVAL_MS);
  control_start();
  trajectory_start_straight(STRAIGHT_DISTANCE_MM, STRAIGHT_SPEED_MM_S, 0.0f, STRAIGHT_ACCEL_MM_S2);
  wait_until_done();
  settle_and_stop();

  print_straight_result(start_distance_mm, start_angle_deg);
  signal_end();
}

/* その場旋回の結果をシリアルに出す */
static void print_pivot_result(float start_distance_mm, float start_angle_deg)
{
  float moved_mm = odometry_get_distance_mm() - start_distance_mm;
  float turned_deg = imu_get_angle_deg() - start_angle_deg;
  printf("Pivot: target %.1f deg, gyro %.2f deg, moved %.2f mm\r\n",
         PIVOT_ANGLE_DEG, turned_deg, moved_mm);
  printf("  position error %.2f mm, angle error %.2f deg\r\n",
         control_get_position_error_mm(), control_get_angle_error_deg());
  if (failsafe_is_stopped()) {
    printf("  stopped by failsafe\r\n");
  }
}

/* モード2 No.4 その場 180° 旋回：その場で左に 180° 回って止まる（流れは先頭のコメント） */
void adjust_pivot_180(void)
{
  float start_distance_mm = odometry_get_distance_mm();
  float start_angle_deg = imu_get_angle_deg();

  log_start(PIVOT_LOG_INTERVAL_MS);
  control_start();
  trajectory_start_pivot(PIVOT_ANGLE_DEG, PIVOT_OMEGA_DEG_S, PIVOT_ALPHA_DEG_S2);
  wait_until_done();
  settle_and_stop();

  print_pivot_result(start_distance_mm, start_angle_deg);
  signal_end();
}
