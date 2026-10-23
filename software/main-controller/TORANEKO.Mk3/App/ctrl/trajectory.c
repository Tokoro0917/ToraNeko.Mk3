/*
 * trajectory.c
 * 目標の速さ・角速度を作る（台形加減速）
 *
 * ■ 台形加減速
 * 速さを時間で書くと、台形の形になる動き方。
 *   1. 決めた加速度で、最高速まで速くする
 *   2. 最高速のまま進む
 *   3. 残りの量がちょうど止まれる（終わりの速さになれる）量になったら、同じ大きさの加速度で遅くする
 * 量が少なくて最高速まで届かないときは、2 がなく、三角形になる。
 * 直進（距離 [mm]・速さ [mm/s]）でも、その場旋回（角度 [deg]・角速度 [deg/s]）でも、計算は同じなので、
 * 中では「量」「速さ」「加速度」とだけ呼んで、同じ計算を使う。
 *
 * ■ 減速の始めどき
 * 速さ v から終わりの速さ v_end まで、加速度 a で遅くするのに要る量は (v² − v_end²) ÷ (2a)。
 * 残りの量がこれ以下になったら減速する。減速中は、残りの量 r から速さを
 * v = √(v_end² + 2a × r) で決める（ちょうど残りの量で v_end になる速さ）。
 * 0.5ms ごとに少しずつ減らすより、ぴったり終わりの位置で止まれる。
 *
 * ■ 旋回の向き
 * 右回り（角度がマイナス）のときは、中では + の量として計算し、取り出すときに - をかける。
 *
 * ■ 割り込み
 * trajectory_update() は制御周期の割り込み（ctrl/control.c）から呼ぶ。
 * trajectory_start_*() はメインの処理から呼ぶ。値を書いているあいだに割り込みが中途半端な値を
 * 使わないように、先に「動いていない」にしてから値を書き、最後に「動いている」にする。
 */
#include "ctrl/trajectory.h"

#include <math.h>

#include "config/robot_params.h"

/* 割り込みの中で書き換える（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile bool s_running = false;    /* 動きの途中か */
static volatile bool s_is_pivot = false;   /* 今の動きがその場旋回なら true、直進なら false */
static volatile float s_direction = 1.0f;  /* 旋回の向き（左回り 1、右回り -1）。直進は 1 */
static volatile float s_speed = 0.0f;      /* 今の目標の速さ（直進は mm/s、旋回は deg/s。いつも + ） */
static volatile float s_accel = 0.0f;      /* 今の目標の加速度（加速中 +、減速中 -、それ以外 0） */
static volatile float s_amount = 0.0f;     /* この動きで進んだ量（直進は mm、旋回は deg。いつも + ） */

/* 動きの決まり（trajectory_start_*() で決める） */
static volatile float s_goal_amount = 0.0f;
static volatile float s_max_speed = 0.0f;
static volatile float s_end_speed = 0.0f;
static volatile float s_accel_limit = 0.0f;

/* 止まっている状態にする（速さ 0、動いていない） */
void trajectory_reset(void)
{
  s_running = false;
  s_is_pivot = false;
  s_direction = 1.0f;
  s_speed = 0.0f;
  s_accel = 0.0f;
  s_amount = 0.0f;
}

/* まっすぐ distance_mm 進む動きを始める。今の目標の速さから始め、最高速 max_speed_mm_s まで
 * accel_mm_s2 で速くし、終わりに end_speed_mm_s になるように遅くする */
void trajectory_start_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2)
{
  s_running = false;  /* 書いているあいだは、割り込みに使わせない */
  if (s_is_pivot) {
    s_speed = 0.0f;   /* 旋回のあとは、止まったところから始める */
  }
  s_is_pivot = false;
  s_direction = 1.0f;
  s_goal_amount = distance_mm;
  s_max_speed = max_speed_mm_s;
  s_end_speed = end_speed_mm_s;
  s_accel_limit = accel_mm_s2;
  s_amount = 0.0f;
  s_running = true;   /* 最後に、動いているにする */
}

/* その場で angle_deg 回る動きを始める（左回りが +）。止まったところから始め、最高の角速度 max_omega_deg_s まで
 * alpha_deg_s2 で速くし、終わりに止まる */
void trajectory_start_pivot(float angle_deg, float max_omega_deg_s, float alpha_deg_s2)
{
  s_running = false;  /* 書いているあいだは、割り込みに使わせない */
  s_is_pivot = true;
  if (angle_deg >= 0.0f) {
    s_direction = 1.0f;
    s_goal_amount = angle_deg;
  } else {
    s_direction = -1.0f;
    s_goal_amount = -angle_deg;
  }
  s_speed = 0.0f;
  s_max_speed = max_omega_deg_s;
  s_end_speed = 0.0f;
  s_accel_limit = alpha_deg_s2;
  s_amount = 0.0f;
  s_running = true;   /* 最後に、動いているにする */
}

/* 速さ speed から終わりの速さまで遅くするのに要る量 */
static float braking_amount(float speed)
{
  float end = s_end_speed;
  if (speed <= end) {
    return 0.0f;
  }
  return (speed * speed - end * end) / (2.0f * s_accel_limit);
}

/* 目標の速さを 1 周期ぶん進める（加速・そのまま・減速のどれかにする） */
static void update_speed(void)
{
  float remaining = s_goal_amount - s_amount;
  float step = s_accel_limit * CTRL_PERIOD_S;  /* 1 周期で変える速さ */

  if (remaining <= braking_amount(s_speed)) {
    /* 減速：残りの量でちょうど終わりの速さになる速さにする */
    float end = s_end_speed;
    s_speed = sqrtf(end * end + 2.0f * s_accel_limit * remaining);
    s_accel = -s_accel_limit;
  } else if (s_speed + step <= s_max_speed) {
    /* 加速 */
    s_speed = s_speed + step;
    s_accel = s_accel_limit;
  } else {
    /* 最高速のまま */
    s_speed = s_max_speed;
    s_accel = 0.0f;
  }
}

/* 制御周期ごとに呼ぶ。目標の速さを決め、その速さで 1 周期ぶん進める。決めた量に着いたら終わる */
void trajectory_update(void)
{
  if (!s_running) {
    return;
  }
  update_speed();
  s_amount = s_amount + s_speed * CTRL_PERIOD_S;

  if (s_amount >= s_goal_amount) {
    /* 着いた。このあとは、終わりの速さのまま（止まるなら 0 のまま） */
    s_amount = s_goal_amount;
    s_speed = s_end_speed;
    s_accel = 0.0f;
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

/* 今の（最後に始めた）動きが、その場旋回か */
bool trajectory_is_pivot(void)
{
  return s_is_pivot;
}

/* 直進の、今の目標の速さ [mm/s]（旋回のあいだは 0） */
float trajectory_get_speed_mm_s(void)
{
  if (s_is_pivot) {
    return 0.0f;
  }
  return s_speed;
}

/* 直進の、今の目標の加速度 [mm/s²]（旋回のあいだは 0） */
float trajectory_get_accel_mm_s2(void)
{
  if (s_is_pivot) {
    return 0.0f;
  }
  return s_accel;
}

/* 直進の、今の動きで進んだ距離 [mm]（旋回のあいだは 0） */
float trajectory_get_distance_mm(void)
{
  if (s_is_pivot) {
    return 0.0f;
  }
  return s_amount;
}

/* 旋回の、今の目標の角速度 [deg/s]（左回りが +。直進のあいだは 0） */
float trajectory_get_omega_deg_s(void)
{
  if (!s_is_pivot) {
    return 0.0f;
  }
  return s_direction * s_speed;
}

/* 旋回の、今の目標の角加速度 [deg/s²]（左回りに速くなる向きが +。直進のあいだは 0） */
float trajectory_get_alpha_deg_s2(void)
{
  if (!s_is_pivot) {
    return 0.0f;
  }
  return s_direction * s_accel;
}

/* 旋回の、今の動きで回った角度 [deg]（左回りが +。直進のあいだは 0） */
float trajectory_get_angle_deg(void)
{
  if (!s_is_pivot) {
    return 0.0f;
  }
  return s_direction * s_amount;
}
