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
 *   - 走り終わったら、エンコーダで測った距離と、ジャイロで測った向きのずれを覚えておく
 *
 * ■ モード6 No.7 壁に沿って直進（adjust_straight_along_wall）
 * 区画のまん中から 4 区画（360mm）を、探索の速さ（300mm/s）で、横の壁に沿って走る補正を入れて走って止まる。
 *   - 左右に壁がある通路で、少し左か右に寄せて（または少し斜めに）置いて走らせ、まん中に戻るかを見る
 *   - 走行ログの wall_err（ずれ）、wall_p・wall_d（P・D の分）、w_target（補正を足した目標の角速度）で、
 *     config/ctrl_gains.h の WALL_KP・WALL_KD を合わせる
 *
 * ■ モード2 No.4 その場 180° 旋回（adjust_pivot_180）
 * その場で左に 180° 回って止まる（U ターンの部品）。
 *   - 止まったら、本当に 180° 回ったかを目で見る（ジャイロの値と合っているか）
 *   - 走行ログ（1ms ごと）を残すので、目標の角速度と測った角速度を見比べて、config/ctrl_gains.h の係数を合わせる
 *   - 走り終わったら、ジャイロで測った角度と、エンコーダで測った前後のずれを覚えておく
 *
 * ■ モード2 No.1 スラローム 90°（adjust_slalom_90）
 * 探索の速さ（300mm/s）で、左に 90° のスラロームをする（探索の小回り）。
 *   1. 区画のまん中から、次の区画の入口の辺まで 45mm を、300mm/s まで速くしながら進む
 *   2. 前距離（曲がる前の直進）、左に 90° 曲がる、後距離（曲がったあとの直進）で、左の区画の入口の辺へ
 *      （300mm/s、最高 600deg/s、8000deg/s² なら、曲がるあいだに前へ 40.4mm、横へ 40.3mm 進むので、
 *        前距離・後距離はそれぞれ約 4.6mm にすると、区画 45mm ちょうどになる。PC で計算した値）
 *   3. 入口の辺から 45mm 進みながら止まり、左の区画のまん中で止まる
 *   - 止まったら、左の区画のまん中で、左を向いて止まっているかを目で見る
 *     曲がりが足りない・行きすぎるなら、前距離・後距離（SLALOM_PRE_MM・SLALOM_POST_MM）を合わせる
 *   - 走行ログ（2ms ごと）を残す
 *
 * ■ 走り出す前
 * 走り出す前に、ジャイロのゼロ点を測り直す（約 0.5 秒。スタートの合図のあと、機体は止まっている）。
 * 電源を入れた直後に測ったゼロ点は、機体を手で動かしていると大きくずれるため。
 *
 * ■ 結果の出し方
 * 走っているあいだはケーブルを挿していないので、結果は RAM に覚えておき、ケーブルを挿してから
 * モード7 No.1（ログ出力）で、走行ログの前に出す（adjust_print_last_result()）。電源を切ると消える。
 */
#include "sys/adjust.h"

#include <stdbool.h>
#include <stdio.h>

#include "config/run_params.h"
#include "ctrl/control.h"
#include "ctrl/odometry.h"
#include "ctrl/run.h"
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
#define PIVOT_LOG_INTERVAL_MS 1        /* 走行ログの間隔 [ms]（約 0.65 秒なので、1000 行に入る） */

#define SLALOM_LOG_INTERVAL_MS 2       /* 走行ログの間隔 [ms]（約 1.2 秒なので、1000 行に入る） */
/* 探索の速さ・スラローム・その場旋回の値は config/run_params.h（探索と同じ値を使う） */

/* 最後に走らせた動きの結果（走り終わったときはケーブルを挿していないので、覚えておいて、あとで出す） */
typedef enum {
  RESULT_NONE = 0,   /* まだ何も走らせていない */
  RESULT_STRAIGHT,   /* 直進 */
  RESULT_PIVOT,      /* その場旋回 */
  RESULT_SLALOM,     /* スラローム */
} result_kind_t;

typedef struct {
  result_kind_t kind;
  float target;             /* 目標（直進は距離 [mm]、旋回は角度 [deg]） */
  float moved_mm;           /* エンコーダで測った、進んだ距離 [mm] */
  float turned_deg;         /* ジャイロで測った、回った角度 [deg] */
  float position_error_mm;  /* 終わったときの位置のずれ [mm] */
  float angle_error_deg;    /* 終わったときの角度のずれ [deg] */
  bool failsafe;            /* フェイルセーフで止まったか */
} run_result_t;

static run_result_t s_result = {RESULT_NONE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

/* 今の動き（直進・スラローム）が終わるまで、休まずに待つ。止まらずに次の動きへつなぐときに使う。
 * ui_wait_ms() で 10ms ずつ待つと、そのあいだに機体が 3mm（300mm/s のとき）進んでしまい、
 * 次の動きの距離がずれるので、終わったらすぐに戻る（trajectory_is_done() は割り込みが書き換える） */
static void wait_until_done_quick(void)
{
  while (!trajectory_is_done() && !failsafe_is_stopped()) {
  }
}

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

/* 直進の結果を覚える（ケーブルは走り終わってから挿すので、あとでモード7 No.1 で出す） */
static void save_straight_result(float start_distance_mm, float start_angle_deg)
{
  s_result.kind = RESULT_STRAIGHT;
  s_result.target = STRAIGHT_DISTANCE_MM;
  s_result.moved_mm = odometry_get_distance_mm() - start_distance_mm;
  s_result.turned_deg = imu_get_angle_deg() - start_angle_deg;
  s_result.position_error_mm = control_get_position_error_mm();
  s_result.angle_error_deg = control_get_angle_error_deg();
  s_result.failsafe = failsafe_is_stopped();
}

/* 探索で、区画の入口に着いてから次の動きを頼まれるまでの、いちばん長かった時間を出す（探索を走らせていれば） */
static void print_search_wait(void)
{
  uint32_t wait_us = run_get_longest_wait_us();
  if (wait_us > 0) {
    printf("Search: longest wait between moves %lu us\r\n", wait_us);
  }
}

/* 最後に走らせた動き（直進・その場旋回）の結果をシリアルに出す。モード7 No.1（ログ出力）でも呼ぶ。
 * 電源を切ると消える（走行ログと同じ） */
void adjust_print_last_result(void)
{
  if (s_result.kind == RESULT_STRAIGHT) {
    printf("Straight: target %.1f mm, encoder %.1f mm, angle %.2f deg\r\n",
           s_result.target, s_result.moved_mm, s_result.turned_deg);
  } else if (s_result.kind == RESULT_PIVOT) {
    printf("Pivot: target %.1f deg, gyro %.2f deg, moved %.2f mm\r\n",
           s_result.target, s_result.turned_deg, s_result.moved_mm);
  } else if (s_result.kind == RESULT_SLALOM) {
    printf("Slalom: target %.1f deg, gyro %.2f deg, encoder %.1f mm\r\n",
           s_result.target, s_result.turned_deg, s_result.moved_mm);
  } else {
    print_search_wait();
    return;  /* 調整の動きは、まだ何も走らせていない */
  }
  printf("  position error %.2f mm, angle error %.2f deg\r\n",
         s_result.position_error_mm, s_result.angle_error_deg);
  if (s_result.failsafe) {
    printf("  stopped by failsafe\r\n");
  }
  print_search_wait();
}

/* モード6 No.1 直進：ゆっくり 360mm まっすぐ走って止まる（流れは先頭のコメント） */
void adjust_straight(void)
{
  imu_remeasure_offset();  /* ジャイロのゼロ点を測り直す（約 0.5 秒） */
  float start_distance_mm = odometry_get_distance_mm();
  float start_angle_deg = imu_get_angle_deg();

  log_start(STRAIGHT_LOG_INTERVAL_MS);
  control_set_wall_control(false);  /* 横の壁に沿って走る補正は入れない */
  control_start();
  trajectory_start_straight(STRAIGHT_DISTANCE_MM, STRAIGHT_SPEED_MM_S, 0.0f, STRAIGHT_ACCEL_MM_S2);
  wait_until_done();
  settle_and_stop();

  save_straight_result(start_distance_mm, start_angle_deg);
  adjust_print_last_result();  /* ケーブルを挿したまま走らせたときのために、すぐにも出す */
  signal_end();
}

/* その場旋回の結果を覚える（あとでモード7 No.1 で出す） */
static void save_pivot_result(float start_distance_mm, float start_angle_deg)
{
  s_result.kind = RESULT_PIVOT;
  s_result.target = PIVOT_ANGLE_DEG;
  s_result.moved_mm = odometry_get_distance_mm() - start_distance_mm;
  s_result.turned_deg = imu_get_angle_deg() - start_angle_deg;
  s_result.position_error_mm = control_get_position_error_mm();
  s_result.angle_error_deg = control_get_angle_error_deg();
  s_result.failsafe = failsafe_is_stopped();
}

/* モード2 No.4 その場 180° 旋回：その場で左に 180° 回って止まる（流れは先頭のコメント） */
void adjust_pivot_180(void)
{
  imu_remeasure_offset();  /* ジャイロのゼロ点を測り直す（約 0.5 秒） */
  float start_distance_mm = odometry_get_distance_mm();
  float start_angle_deg = imu_get_angle_deg();

  log_start(PIVOT_LOG_INTERVAL_MS);
  control_set_wall_control(false);  /* 横の壁に沿って走る補正は入れない */
  control_start();
  trajectory_start_pivot(PIVOT_ANGLE_DEG, PIVOT_OMEGA_DEG_S, PIVOT_ALPHA_DEG_S2);
  wait_until_done();
  settle_and_stop();

  save_pivot_result(start_distance_mm, start_angle_deg);
  adjust_print_last_result();
  signal_end();
}

/* スラロームの結果を覚える（あとでモード7 No.1 で出す）。target は角度、moved はスラローム全体の距離 */
static void save_slalom_result(float start_distance_mm, float start_angle_deg)
{
  s_result.kind = RESULT_SLALOM;
  s_result.target = SLALOM_ANGLE_DEG;
  s_result.moved_mm = odometry_get_distance_mm() - start_distance_mm;
  s_result.turned_deg = imu_get_angle_deg() - start_angle_deg;
  s_result.position_error_mm = control_get_position_error_mm();
  s_result.angle_error_deg = control_get_angle_error_deg();
  s_result.failsafe = failsafe_is_stopped();
}

/* モード2 No.1 スラローム 90°：区画のまん中から走り出し、次の区画で左に 90° 曲がり、左の区画のまん中で止まる
 * （流れは先頭のコメント） */
void adjust_slalom_90(void)
{
  imu_remeasure_offset();  /* ジャイロのゼロ点を測り直す（約 0.5 秒） */
  float start_distance_mm = odometry_get_distance_mm();
  float start_angle_deg = imu_get_angle_deg();

  log_start(SLALOM_LOG_INTERVAL_MS);
  control_set_wall_control(false);  /* 横の壁に沿って走る補正は入れない */
  control_start();
  /* 1. 次の区画の入口の辺まで、速くしながら進む */
  trajectory_start_straight(HALF_CELL_MM, SEARCH_SPEED_MM_S, SEARCH_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
  wait_until_done_quick();
  /* 2. 前距離、左に 90°、後距離 */
  trajectory_start_straight(SLALOM_PRE_MM, SEARCH_SPEED_MM_S, SEARCH_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
  wait_until_done_quick();
  trajectory_start_slalom(SLALOM_ANGLE_DEG, SEARCH_SPEED_MM_S, SLALOM_OMEGA_DEG_S, SLALOM_ALPHA_DEG_S2);
  wait_until_done_quick();
  trajectory_start_straight(SLALOM_POST_MM, SEARCH_SPEED_MM_S, SEARCH_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
  wait_until_done_quick();
  /* 3. 左の区画のまん中まで進んで止まる */
  trajectory_start_straight(HALF_CELL_MM, SEARCH_SPEED_MM_S, 0.0f, SEARCH_ACCEL_MM_S2);
  wait_until_done();
  settle_and_stop();

  save_slalom_result(start_distance_mm, start_angle_deg);
  adjust_print_last_result();
  signal_end();
}

/* モード6 No.7 壁に沿って直進：区画のまん中から 4 区画を、横の壁に沿って走る補正を入れて、探索の速さで走って止まる */
void adjust_straight_along_wall(void)
{
  imu_remeasure_offset();  /* ジャイロのゼロ点を測り直す（約 0.5 秒） */
  float start_distance_mm = odometry_get_distance_mm();
  float start_angle_deg = imu_get_angle_deg();

  log_start(STRAIGHT_LOG_INTERVAL_MS);
  control_set_wall_control(true);  /* 横の壁に沿って走る補正を入れる */
  control_start();
  trajectory_start_straight(STRAIGHT_DISTANCE_MM, SEARCH_SPEED_MM_S, 0.0f, SEARCH_ACCEL_MM_S2);
  wait_until_done();
  settle_and_stop();
  control_set_wall_control(false);

  save_straight_result(start_distance_mm, start_angle_deg);
  adjust_print_last_result();
  signal_end();
}
