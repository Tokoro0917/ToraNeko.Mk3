/*
 * run.c
 * 機体を動かす（探索の動き：発進・直進・スラローム・止まる・その場で向きを変えて発進）
 *
 * ■ 2 つの状態（run.h の先頭のコメント）
 *   走っている  ：区画の入口の辺を、探索の速さ（300mm/s）で通るところ。動きの関数は、ここに着いたら戻る
 *   止まっている：区画のまん中で止まっている
 * 呼んだ側（sys/explore.c）は、関数が戻ってから次の動きを頼むまでのあいだに、壁を記録して次の行き先を決める。
 * そのあいだも機体は 300mm/s で前に進み続ける（目標の速さは、前の動きの終わりの速さのまま）。
 *
 * ■ 動きの作り方（値は config/run_params.h）
 *   発進       ：スタート区画（お尻を後ろの壁に付けて置く）から、北隣の区画の入口まで、速くしながら進む
 *   直進       ：1 区画（90mm）を 300mm/s のまま進む
 *   左・右     ：前距離 → スラローム 90° → 後距離 で、左・右の区画の入口へ（モード2 No.1 で確かめた値）
 *   止まる     ：入口から 45mm 進みながら止まり、まん中で止まる。モーターの出力を 0 にしてから戻る
 *   向きを変えて発進：その場で 90°・180° 回ってから（回らないこともある）、45mm 進みながら速くして、入口へ
 *
 * ■ 横の壁に沿って走る補正
 * 発進のときに入れる（ctrl/control.c）。直進のあいだだけ効く（スラローム・その場旋回のあいだは効かない）。
 *
 * ■ 前壁合わせ
 * 止まっているところから発進するとき（U ターンや、止まって計算したあと）、前に壁があれば、
 * 前の壁を使って位置と向きを ALIGN_TIME_MS（0.5 秒）直してから、向きを変える（ctrl/control.c）
 *
 * ■ フェイルセーフ（run.h の約束 3）
 * 動きの途中でフェイルセーフで止まったら、すぐに戻る。止まっているあいだに呼ばれたら、何もせずにすぐ戻る。
 */
#include "ctrl/run.h"

#include <stdbool.h>

#include "config/run_params.h"
#include "ctrl/control.h"
#include "ctrl/trajectory.h"
#include "ctrl/wall_ctrl.h"
#include "drv/imu.h"
#include "drv/timebase.h"
#include "sys/failsafe.h"
#include "sys/log.h"

#define SEARCH_LOG_INTERVAL_MS 8  /* 探索の走行ログの間隔 [ms]（1000 行で 8 秒ぶん） */

/* 動きが戻ってから次の動きを頼まれるまでの時間を測る（探索の計算の時間） */
static bool s_waiting = false;          /* 走ったまま戻って、次の動きを待っているところか */
static uint32_t s_return_us = 0;        /* 戻った時刻 [µs] */
static uint32_t s_longest_wait_us = 0;  /* いちばん長かった待ち時間 [µs] */

/* 走ったまま戻るときに呼ぶ。戻った時刻を覚える */
static void mark_return(void)
{
  s_return_us = timebase_get_us();
  s_waiting = true;
}

/* 次の動きを頼まれたときに呼ぶ。戻ってからの時間が、いちばん長ければ覚える */
static void measure_wait(void)
{
  if (!s_waiting) {
    return;
  }
  uint32_t wait_us = timebase_get_us() - s_return_us;
  if (wait_us > s_longest_wait_us) {
    s_longest_wait_us = wait_us;
  }
  s_waiting = false;
}

/* いちばん長かった待ち時間 [µs] */
uint32_t run_get_longest_wait_us(void)
{
  return s_longest_wait_us;
}

/* 今の動きが終わるまで、休まずに待つ。フェイルセーフで止まったら、すぐに戻る。
 * 少しずつ休みながら待つと、そのあいだに機体が進んで次の動きの距離がずれるので、休まない
 * （trajectory_is_done() は割り込みが書き換える） */
static void wait_until_done(void)
{
  while (!trajectory_is_done() && !failsafe_is_stopped()) {
  }
}

/* distance_mm まっすぐ進んで、探索の速さで戻る。走っているときは速さを変えずに進み、
 * 止まっているところからなら、進みながら探索の速さまで速くする */
static void go_straight(float distance_mm)
{
  if (failsafe_is_stopped()) {
    return;
  }
  trajectory_start_straight(distance_mm, SEARCH_SPEED_MM_S, SEARCH_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
  wait_until_done();
}

/* 前距離 → スラローム angle_deg（左が +、右が -）→ 後距離 で、横の区画の入口へ */
static void slalom(float angle_deg)
{
  go_straight(SLALOM_PRE_MM);
  if (failsafe_is_stopped()) {
    return;
  }
  trajectory_start_slalom(angle_deg, SEARCH_SPEED_MM_S, SLALOM_OMEGA_DEG_S, SLALOM_ALPHA_DEG_S2);
  wait_until_done();
  go_straight(SLALOM_POST_MM);
}

/* その場で angle_deg 回って止まる（左が +） */
static void pivot(float angle_deg)
{
  if (failsafe_is_stopped()) {
    return;
  }
  trajectory_start_pivot(angle_deg, PIVOT_OMEGA_DEG_S, PIVOT_ALPHA_DEG_S2);
  wait_until_done();
}

/* 前の壁で、位置と向きを ALIGN_TIME_MS 直す（止まっているときに呼ぶ） */
static void align_to_front_wall(void)
{
  if (failsafe_is_stopped()) {
    return;
  }
  control_set_front_align(true);
  timebase_wait_ms(ALIGN_TIME_MS);
  control_set_front_align(false);
}

/* スタート区画から走り出し、北隣の区画 (0, 1) の入口の辺まで進む。
 * スタート区画では、機体のお尻を後ろの壁に付けて置く（タイヤの軸は区画のまん中より START_OFFSET_MM 後ろ） */
void run_search_start(void)
{
  if (failsafe_is_stopped()) {
    return;
  }
  imu_remeasure_offset();  /* ジャイロのゼロ点を測り直す（約 0.5 秒。電源を入れたときに手で動かしていてもよいように） */
  s_waiting = false;
  s_longest_wait_us = 0;
  log_start(SEARCH_LOG_INTERVAL_MS);
  control_set_wall_control(true);  /* 探索では、横の壁に沿って走る補正を入れる */
  control_start();
  go_straight(START_OFFSET_MM + HALF_CELL_MM);
  mark_return();
}

/* 走っている → 1 区画まっすぐ進み、次の区画の入口の辺へ */
void run_search_straight(void)
{
  measure_wait();
  go_straight(CELL_MM);
  mark_return();
}

/* 走っている → スラロームで左 90° */
void run_search_turn_left(void)
{
  measure_wait();
  slalom(SLALOM_ANGLE_DEG);
  mark_return();
}

/* 走っている → スラロームで右 90° */
void run_search_turn_right(void)
{
  measure_wait();
  slalom(-SLALOM_ANGLE_DEG);
  mark_return();
}

/* 走っている → 今入った区画のまん中で止まる。止まったら、モーターの出力を 0 にしてから戻る
 * （このあと Flash に保存して CPU が止まっても、機体が動かないように。run.h の約束 2） */
void run_search_stop_at_center(void)
{
  measure_wait();
  if (failsafe_is_stopped()) {
    return;
  }
  trajectory_start_straight(HALF_CELL_MM, SEARCH_SPEED_MM_S, 0.0f, SEARCH_ACCEL_MM_S2);
  wait_until_done();
  if (failsafe_is_stopped()) {
    return;
  }
  timebase_wait_ms(STOP_SETTLE_MS);  /* 目標の速さ 0 のまま制御を続けて、ぴたりと止める */
  control_stop();                    /* 制御をやめて、モーターを止める */
}

/* 止まっている → 前に壁があれば前壁合わせをして、その場で向きを変えて（turn）、
 * 次の区画の入口の辺まで、速くしながら進む */
void run_search_go_from_center(run_turn_t turn)
{
  if (failsafe_is_stopped()) {
    return;
  }
  control_start();  /* 止まっているので、制御を始め直す */
  if (wall_ctrl_has_wall_front()) {
    align_to_front_wall();
    control_start();  /* 直した向きを、まっすぐの向きにし直す */
  }
  if (turn == RUN_TURN_LEFT) {
    pivot(90.0f);
  } else if (turn == RUN_TURN_RIGHT) {
    pivot(-90.0f);
  } else if (turn == RUN_TURN_BACK) {
    pivot(180.0f);
  }
  go_straight(HALF_CELL_MM);
  mark_return();
}
