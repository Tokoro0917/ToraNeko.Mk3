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
 * 直進（距離 [mm]・速さ [mm/s]）でも、その場旋回・スラローム（角度 [deg]・角速度 [deg/s]）でも、計算は同じなので、
 * 中では「量」「速さ」「加速度」とだけ呼んで、同じ計算を使う。
 * スラロームは、前に進む速さを一定（s_forward_speed_mm_s）にしたまま、角速度を台形にする。
 *
 * ■ 減速の始めどき
 * 速さ v から終わりの速さ v_end まで、加速度 a で遅くするのに要る量は (v² − v_end²) ÷ (2a)。
 * 残りの量がこれ以下になったら減速する。減速中は、残りの量 r から速さを
 * v = √(v_end² + 2a × r) で決める（ちょうど残りの量で v_end になる速さ）。
 * 0.5ms ごとに少しずつ減らすより、ぴったり終わりの位置で止まれる。
 *
 * ■ 動きと動きのあいだに進んだ距離
 * 探索では、区画の入口に着いて動きが終わってから、次の動きを頼まれるまでに、壁の記録と行き先の計算で
 * 30〜40ms かかる。そのあいだも機体は終わりの速さ（300mm/s）で進み続けるので、10mm ほど進んでしまう
 * （2026-10-24 の探索のログで、曲がり始めが毎回 10〜13mm 遅れ、壁に当たりかけて気づいた）。
 * そこで、動いていないあいだに進んだ距離（s_overrun_mm）も数えておき、次の直進の距離からその分を引く。
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

#define MIN_DISTANCE_MM 0.01f  /* 直進の距離のいちばん小さい値 [mm]（あいだに進んだ分を引いて 0 以下になったとき） */

/* 割り込みの中で書き換える（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile bool s_running = false;    /* 動きの途中か */
static volatile trajectory_kind_t s_kind = TRAJECTORY_STRAIGHT;  /* 今の動きの種類 */
static volatile float s_forward_speed_mm_s = 0.0f;  /* スラロームのあいだの、前に進む速さ（一定） */
static volatile float s_direction = 1.0f;  /* 旋回の向き（左回り 1、右回り -1）。直進は 1 */
static volatile float s_speed = 0.0f;      /* 今の目標の速さ（直進は mm/s、旋回は deg/s。いつも + ） */
static volatile float s_accel = 0.0f;      /* 今の目標の加速度（加速中 +、減速中 -、それ以外 0） */
static volatile float s_amount = 0.0f;     /* この動きで進んだ量（直進は mm、旋回は deg。いつも + ） */
static volatile float s_overrun_mm = 0.0f; /* 動きが終わってから、次の動きを始めるまでに進んだ距離 [mm] */

/* 動きの決まり（trajectory_start_*() で決める） */
static volatile float s_goal_amount = 0.0f;
static volatile float s_max_speed = 0.0f;
static volatile float s_end_speed = 0.0f;
static volatile float s_accel_limit = 0.0f;

/* 止まっている状態にする（速さ 0、動いていない） */
void trajectory_reset(void)
{
  s_running = false;
  s_kind = TRAJECTORY_STRAIGHT;
  s_direction = 1.0f;
  s_forward_speed_mm_s = 0.0f;
  s_speed = 0.0f;
  s_accel = 0.0f;
  s_amount = 0.0f;
  s_overrun_mm = 0.0f;
}

/* まっすぐ distance_mm 進む動きを始める。今の目標の速さから始め、最高速 max_speed_mm_s まで
 * accel_mm_s2 で速くし、終わりに end_speed_mm_s になるように遅くする */
void trajectory_start_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2)
{
  s_running = false;  /* 書いているあいだは、割り込みに使わせない */
  s_speed = trajectory_get_speed_mm_s();  /* 今の、前に進む目標の速さから始める（その場旋回のあとは 0、スラロームのあとはその速さ） */
  s_kind = TRAJECTORY_STRAIGHT;
  /* 前の動きが終わってから進んでしまった分を引く。引きすぎて 0 以下になったら、すぐに終わるように少しだけにする */
  distance_mm = distance_mm - s_overrun_mm;
  s_overrun_mm = 0.0f;
  if (distance_mm < MIN_DISTANCE_MM) {
    distance_mm = MIN_DISTANCE_MM;
  }
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
  s_kind = TRAJECTORY_PIVOT;
  s_forward_speed_mm_s = 0.0f;
  s_overrun_mm = 0.0f;  /* その場旋回は止まったところから始めるので、進んだ分はない */
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

/* 前に進む速さ speed_mm_s のまま、angle_deg 曲がる動きを始める（左回りが +）。角速度は 0 から始め、
 * 最高の角速度 max_omega_deg_s まで alpha_deg_s2 で速くし、曲がり終わりに 0 に戻す。曲がり終わったあとも、
 * 前に進む速さはそのまま（次の直進は、この速さから始まる） */
void trajectory_start_slalom(float angle_deg, float speed_mm_s, float max_omega_deg_s, float alpha_deg_s2)
{
  s_running = false;  /* 書いているあいだは、割り込みに使わせない */
  s_kind = TRAJECTORY_SLALOM;
  s_forward_speed_mm_s = speed_mm_s;
  s_overrun_mm = 0.0f;  /* スラロームの前は前距離の直進なので、あいだに進んだ分は、その直進で引いてある */
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
    /* 動いていないあいだも、前に進む目標の速さが 0 でなければ、その分だけ進んだ距離を数えておく */
    s_overrun_mm = s_overrun_mm + trajectory_get_speed_mm_s() * CTRL_PERIOD_S;
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

/* 今の（最後に始めた）動きの種類 */
trajectory_kind_t trajectory_get_kind(void)
{
  return s_kind;
}

/* 今の目標の、前に進む速さ [mm/s]（その場旋回は 0、スラロームは一定の速さ） */
float trajectory_get_speed_mm_s(void)
{
  if (s_kind == TRAJECTORY_STRAIGHT) {
    return s_speed;
  }
  if (s_kind == TRAJECTORY_SLALOM) {
    return s_forward_speed_mm_s;
  }
  return 0.0f;
}

/* 今の目標の、前に進む加速度 [mm/s²]（直進のときだけ。ほかは 0） */
float trajectory_get_accel_mm_s2(void)
{
  if (s_kind == TRAJECTORY_STRAIGHT) {
    return s_accel;
  }
  return 0.0f;
}

/* 直進の、今の動きで進んだ距離 [mm]（旋回・スラロームのあいだは 0） */
float trajectory_get_distance_mm(void)
{
  if (s_kind == TRAJECTORY_STRAIGHT) {
    return s_amount;
  }
  return 0.0f;
}

/* 旋回・スラロームの、今の目標の角速度 [deg/s]（左回りが +。直進のあいだは 0） */
float trajectory_get_omega_deg_s(void)
{
  if (s_kind == TRAJECTORY_STRAIGHT) {
    return 0.0f;
  }
  return s_direction * s_speed;
}

/* 旋回・スラロームの、今の目標の角加速度 [deg/s²]（左回りに速くなる向きが +。直進のあいだは 0） */
float trajectory_get_alpha_deg_s2(void)
{
  if (s_kind == TRAJECTORY_STRAIGHT) {
    return 0.0f;
  }
  return s_direction * s_accel;
}

/* 旋回・スラロームの、今の動きで回った角度 [deg]（左回りが +。直進のあいだは 0） */
/* 今の直進の進む距離を delta_mm だけ長くする（− なら短くする）。壁切れ（ctrl/run.c）で、位置のずれを直すのに使う。
 * もう進んだ量より短くはしない。メインの処理から呼ぶ（割り込みは s_goal_amount を読むだけなので、書きかえてよい） */
void trajectory_add_distance(float delta_mm)
{
  if (s_kind != TRAJECTORY_STRAIGHT || !s_running) {
    return;
  }
  float goal = s_goal_amount + delta_mm;
  if (goal < s_amount + MIN_DISTANCE_MM) {
    goal = s_amount + MIN_DISTANCE_MM;
  }
  s_goal_amount = goal;
}

/* 今の動きが終わってから、まだ次の動きを頼まれていないあいだに進んだ距離 [mm]（次の直進から引く分）。
 * 前壁補正（ctrl/run.c）が、センサを読んだときの位置を知るのに使う */
float trajectory_get_overrun_mm(void)
{
  return s_overrun_mm;
}

float trajectory_get_angle_deg(void)
{
  if (s_kind == TRAJECTORY_STRAIGHT) {
    return 0.0f;
  }
  return s_direction * s_amount;
}
