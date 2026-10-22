/*
 * trajectory.c
 * 目標の速さを作る（台形加減速）
 *
 * ■ 台形加減速
 * 速さを時間で書くと、台形の形になる動き方。
 *   1. 決めた加速度で、最高速まで速くする
 *   2. 最高速のまま進む
 *   3. 残りの距離がちょうど止まれる（終わりの速さになれる）距離になったら、同じ大きさの加速度で遅くする
 * 距離が短くて最高速まで届かないときは、2 がなく、三角形になる。
 *
 * ■ 減速の始めどき
 * 速さ v から終わりの速さ v_end まで、加速度 a で遅くするのに要る距離は (v² − v_end²) ÷ (2a)。
 * 残りの距離がこれ以下になったら減速する。減速中は、残りの距離 r から速さを
 * v = √(v_end² + 2a × r) で決める（ちょうど残りの距離で v_end になる速さ）。
 * 0.5ms ごとに少しずつ減らすより、ぴったり終わりの位置で止まれる。
 *
 * ■ 割り込み
 * trajectory_update() は制御周期の割り込み（ctrl/control.c）から呼ぶ。
 * trajectory_start_straight() はメインの処理から呼ぶ。値を書いているあいだに割り込みが中途半端な値を
 * 使わないように、先に「動いていない」にしてから値を書き、最後に「動いている」にする。
 */
#include "ctrl/trajectory.h"

#include <math.h>

#include "config/robot_params.h"

/* 割り込みの中で書き換える（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile bool s_running = false;        /* 動きの途中か */
static volatile float s_speed_mm_s = 0.0f;     /* 今の目標の速さ */
static volatile float s_accel_mm_s2 = 0.0f;    /* 今の目標の加速度（加速中 +、減速中 -、それ以外 0） */
static volatile float s_distance_mm = 0.0f;    /* この動きで進んだ距離（目標） */

/* 動きの決まり（trajectory_start_straight() で決める） */
static volatile float s_goal_distance_mm = 0.0f;
static volatile float s_max_speed_mm_s = 0.0f;
static volatile float s_end_speed_mm_s = 0.0f;
static volatile float s_accel_limit_mm_s2 = 0.0f;

/* 止まっている状態にする（速さ 0、動いていない） */
void trajectory_reset(void)
{
  s_running = false;
  s_speed_mm_s = 0.0f;
  s_accel_mm_s2 = 0.0f;
  s_distance_mm = 0.0f;
}

/* まっすぐ distance_mm 進む動きを始める。今の目標の速さから始め、最高速 max_speed_mm_s まで
 * accel_mm_s2 で速くし、終わりに end_speed_mm_s になるように遅くする */
void trajectory_start_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2)
{
  s_running = false;  /* 書いているあいだは、割り込みに使わせない */
  s_goal_distance_mm = distance_mm;
  s_max_speed_mm_s = max_speed_mm_s;
  s_end_speed_mm_s = end_speed_mm_s;
  s_accel_limit_mm_s2 = accel_mm_s2;
  s_distance_mm = 0.0f;
  s_running = true;   /* 最後に、動いているにする */
}

/* 速さ speed_mm_s から終わりの速さまで遅くするのに要る距離 [mm] */
static float braking_distance_mm(float speed_mm_s)
{
  float end = s_end_speed_mm_s;
  if (speed_mm_s <= end) {
    return 0.0f;
  }
  return (speed_mm_s * speed_mm_s - end * end) / (2.0f * s_accel_limit_mm_s2);
}

/* 目標の速さを 1 周期ぶん進める（加速・そのまま・減速のどれかにする） */
static void update_speed(void)
{
  float remaining_mm = s_goal_distance_mm - s_distance_mm;
  float step_mm_s = s_accel_limit_mm_s2 * CTRL_PERIOD_S;  /* 1 周期で変える速さ */

  if (remaining_mm <= braking_distance_mm(s_speed_mm_s)) {
    /* 減速：残りの距離でちょうど終わりの速さになる速さにする */
    float end = s_end_speed_mm_s;
    s_speed_mm_s = sqrtf(end * end + 2.0f * s_accel_limit_mm_s2 * remaining_mm);
    s_accel_mm_s2 = -s_accel_limit_mm_s2;
  } else if (s_speed_mm_s + step_mm_s <= s_max_speed_mm_s) {
    /* 加速 */
    s_speed_mm_s = s_speed_mm_s + step_mm_s;
    s_accel_mm_s2 = s_accel_limit_mm_s2;
  } else {
    /* 最高速のまま */
    s_speed_mm_s = s_max_speed_mm_s;
    s_accel_mm_s2 = 0.0f;
  }
}

/* 制御周期ごとに呼ぶ。目標の速さを決め、その速さで 1 周期ぶん進める。決めた距離に着いたら終わる */
void trajectory_update(void)
{
  if (!s_running) {
    return;
  }
  update_speed();
  s_distance_mm = s_distance_mm + s_speed_mm_s * CTRL_PERIOD_S;

  if (s_distance_mm >= s_goal_distance_mm) {
    /* 着いた。このあとは、終わりの速さのまま（止まるなら 0 のまま） */
    s_distance_mm = s_goal_distance_mm;
    s_speed_mm_s = s_end_speed_mm_s;
    s_accel_mm_s2 = 0.0f;
    s_running = false;
  }
}

/* 今の動きが終わったか */
bool trajectory_is_done(void)
{
  if (s_running) {
    return false;
  }
  return true;
}

/* 今の目標の速さ [mm/s] */
float trajectory_get_speed_mm_s(void)
{
  return s_speed_mm_s;
}

/* 今の目標の加速度 [mm/s²] */
float trajectory_get_accel_mm_s2(void)
{
  return s_accel_mm_s2;
}

/* 今の動きで進んだ距離（目標） [mm] */
float trajectory_get_distance_mm(void)
{
  return s_distance_mm;
}
