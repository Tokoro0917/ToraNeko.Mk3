/*
 * odometry.c
 * エンコーダから、タイヤの速さと走った距離を求める
 *
 * ■ カウントから mm へ
 * エンコーダはモーターの軸で 1 回転 16384 カウント。タイヤ 1 回転はモーター 3.73 回転（ギア比）なので、
 * タイヤ 1 回転は 16384 × 3.73 ≒ 61000 カウント。そのあいだにタイヤは 1 周ぶん（直径 × 円周率 ≒ 44mm）進む。
 * なので、1 カウントは 44 ÷ 61000 ≒ 0.00072mm。
 *
 * ■ 速さ
 * 制御周期（0.5ms）ごとに、前の回から増えたカウントを mm に直し、0.5ms で割る（= 2000 倍する）。
 * 1 カウントの差が 1.4mm/s になるので、止まっていても速さは ±1.4mm/s くらいゆれる。
 *
 * ■ 割り込み
 * odometry_update() は制御周期の割り込み（sys/scheduler.c）で、encoder_update() のあとに呼ぶ。
 */
#include "ctrl/odometry.h"

#include <stdint.h>

#include "config/robot_params.h"
#include "drv/encoder.h"

#define PI_VALUE 3.14159265f

/* 1 カウントで進む距離 [mm] */
#define MM_PER_COUNT (TIRE_DIAMETER_MM * PI_VALUE / (ENCODER_COUNT_PER_REV * GEAR_RATIO))

/* 割り込みの中で書き換える（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile int32_t s_last_left_count = 0;   /* 前の回の左のカウント */
static volatile int32_t s_last_right_count = 0;
static volatile float s_left_speed_mm_s = 0.0f;
static volatile float s_right_speed_mm_s = 0.0f;
static volatile float s_distance_mm = 0.0f;

/* 起動時に 1 回だけ呼ぶ（encoder_init() のあと）。今のカウントを、前の回の値にしておく */
void odometry_init(void)
{
  s_last_left_count = encoder_get_left_total();
  s_last_right_count = encoder_get_right_total();
  s_left_speed_mm_s = 0.0f;
  s_right_speed_mm_s = 0.0f;
  s_distance_mm = 0.0f;
}

/* 制御周期ごとに呼ぶ。前の回から進んだ量で、左右の速さと走った距離を求める */
void odometry_update(void)
{
  int32_t left_count = encoder_get_left_total();
  int32_t right_count = encoder_get_right_total();

  float left_moved_mm = (left_count - s_last_left_count) * MM_PER_COUNT;
  float right_moved_mm = (right_count - s_last_right_count) * MM_PER_COUNT;
  s_last_left_count = left_count;
  s_last_right_count = right_count;

  s_left_speed_mm_s = left_moved_mm / CTRL_PERIOD_S;
  s_right_speed_mm_s = right_moved_mm / CTRL_PERIOD_S;
  s_distance_mm = s_distance_mm + (left_moved_mm + right_moved_mm) / 2.0f;
}

/* 左のタイヤの速さ [mm/s] */
float odometry_get_left_speed_mm_s(void)
{
  return s_left_speed_mm_s;
}

/* 右のタイヤの速さ [mm/s] */
float odometry_get_right_speed_mm_s(void)
{
  return s_right_speed_mm_s;
}

/* 機体の速さ [mm/s]（左右の平均） */
float odometry_get_speed_mm_s(void)
{
  return (s_left_speed_mm_s + s_right_speed_mm_s) / 2.0f;
}

/* 電源を入れてから走った距離 [mm] */
float odometry_get_distance_mm(void)
{
  return s_distance_mm;
}
