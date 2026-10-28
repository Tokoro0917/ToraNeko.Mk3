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
 *   向きを変えて発進：その場で 90° 回ってから（回らないこともある）、45mm 進みながら速くして、入口へ。
 *                    U ターンは 90° を 2 回（左に 90° → 左に 90°）
 *
 * ■ 最短走行の動き（モード1 は sys/shortest.c、モード3 は sys/turn_test.c が使う）
 *   発進       ：制御を始めるだけ（動かない）。すぐあとに直進を頼む
 *   直進       ：決めた距離を、最高速まで速くしてから、決めた終わりの速さ（次のターンの速さ）まで遅くして進む
 *   斜めの直進 ：同じ。斜めのあいだは、横の壁に沿って走る補正を入れない（横のセンサが柱や斜めの壁を見るので）
 *   左・右     ：探索と同じスラローム（前距離 → 90° → 後距離）
 *   大回り・斜めのターン：前距離 → スラローム → 後距離。値は config/turn_params.h（ターンの速さで曲がる）
 *   止まる     ：決めた距離を進みながら止まり、モーターの出力を 0 にする
 *
 * ■ 前壁補正（config/run_params.h の FRONT_CORRECTION_*）
 * 区画の入口で曲がる・止まるとき、その区画に前の壁があれば、前センサで壁までの距離を測る。
 * 入口の辺なら FRONT_WALL_AT_ENTRY_MM（87mm）のはずなので、ずれた分だけ、前距離と止まる距離を直す。
 * センサを読んだときには、入口の辺から少し進んでいる（探索の計算のあいだも進む）ので、その分を足して比べる。
 * かけた量（かけないときは、かけるはずだった量）は数えておき、モード7 No.1 で出す（run_get_front_correction_*）。
 *
 * ■ 横の壁に沿って走る補正
 * 発進のときに入れる（ctrl/control.c）。直進のあいだだけ効く（スラローム・その場旋回のあいだは効かない）。
 *
 * ■ 前壁合わせ
 * 止まっているところから発進するとき（U ターンや、止まって計算したあと）、前に壁があれば、
 * 前の壁を使って位置と向きを ALIGN_TIME_MS（0.5 秒）直してから、向きを変える（ctrl/control.c）。
 * U ターンでは、1 回目の 90° のあとにも、前（もとの横）に壁があれば、もう一度合わせる。
 * 行き止まりなら、前の壁で前後と向き、横の壁で左右と向きが直る
 *
 * ■ フェイルセーフ（run.h の約束 3）
 * 動きの途中でフェイルセーフで止まったら、すぐに戻る。止まっているあいだに呼ばれたら、何もせずにすぐ戻る。
 */
#include "ctrl/run.h"

#include <stdbool.h>

#include "config/run_params.h"
#include "config/turn_params.h"
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

/* 前壁補正の記録（探索を始めるたびに 0 に戻す） */
static int s_front_correction_count = 0;            /* 測った回数 */
static float s_front_correction_min_mm = 0.0f;      /* いちばん小さかった量 [mm] */
static float s_front_correction_max_mm = 0.0f;      /* いちばん大きかった量 [mm] */

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

/* 前壁補正の量 [mm] を決めて記録する（+ なら、壁が思ったより遠い＝まだ手前にいるので、そのぶん長く進む）。
 * 前に壁がないときは 0。FRONT_CORRECTION_ENABLE が 0 なら、記録だけして 0 を返す */
static float front_correction_mm(void)
{
  if (!wall_ctrl_has_wall_front()) {
    return 0.0f;
  }
  /* 今いる位置は、入口の辺から、まだ頼んでいない分（trajectory_get_overrun_mm()）だけ進んだところ */
  float correction_mm = wall_ctrl_get_front_distance_mm() + trajectory_get_overrun_mm() - FRONT_WALL_AT_ENTRY_MM;
  if (correction_mm > FRONT_CORRECTION_LIMIT_MM) {
    correction_mm = FRONT_CORRECTION_LIMIT_MM;
  }
  if (correction_mm < -FRONT_CORRECTION_LIMIT_MM) {
    correction_mm = -FRONT_CORRECTION_LIMIT_MM;
  }

  if (s_front_correction_count == 0 || correction_mm < s_front_correction_min_mm) {
    s_front_correction_min_mm = correction_mm;
  }
  if (s_front_correction_count == 0 || correction_mm > s_front_correction_max_mm) {
    s_front_correction_max_mm = correction_mm;
  }
  s_front_correction_count = s_front_correction_count + 1;

  if (FRONT_CORRECTION_ENABLE == 0) {
    return 0.0f;
  }
  return correction_mm;
}

/* 前壁補正を測った回数・いちばん小さかった量・いちばん大きかった量 [mm]（モード7 No.1 で出す） */
int run_get_front_correction_count(void)
{
  return s_front_correction_count;
}

float run_get_front_correction_min_mm(void)
{
  return s_front_correction_min_mm;
}

float run_get_front_correction_max_mm(void)
{
  return s_front_correction_max_mm;
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

/* 前距離 → スラローム angle_deg（左が +、右が -）→ 後距離 で、横の区画の入口へ。
 * 前に壁があれば、前距離を前壁補正で直す（前距離は 0 より短くしない） */
static void slalom(float angle_deg)
{
  float pre_mm = SLALOM_PRE_MM + front_correction_mm();
  if (pre_mm < 0.0f) {
    pre_mm = 0.0f;
  }
  go_straight(pre_mm);
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

/* 前に壁があれば前壁合わせをして、直した向きを、まっすぐの向きにし直す（止まっているときに呼ぶ） */
static void align_if_front_wall(void)
{
  if (wall_ctrl_has_wall_front()) {
    align_to_front_wall();
    control_start();  /* 直した向きを、まっすぐの向きにし直す */
  }
}

/* 制御とログを始める（探索と最短走行の発進で使う）。横の壁に沿って走る補正も入れる */
static void start_control(void)
{
  imu_remeasure_offset();  /* ジャイロのゼロ点を測り直す（約 0.5 秒。電源を入れたときに手で動かしていてもよいように） */
  s_waiting = false;
  s_longest_wait_us = 0;
  s_front_correction_count = 0;
  log_start(SEARCH_LOG_INTERVAL_MS);
  control_set_wall_control(true);  /* 横の壁に沿って走る補正を入れる */
  control_start();
}

/* スタート区画から走り出し、北隣の区画 (0, 1) の入口の辺まで進む。
 * スタート区画では、機体のお尻を後ろの壁に付けて置く（タイヤの軸は区画のまん中より START_OFFSET_MM 後ろ） */
void run_search_start(void)
{
  if (failsafe_is_stopped()) {
    return;
  }
  start_control();
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

/* 走っている → 見たことのある区画を cells 区画まっすぐ進み、その先の区画の入口の辺へ（既知区間加速。sys/explore.c）。
 * KNOWN_MAX_SPEED_MM_S まで速くしてから、探索の速さに戻して着く */
void run_search_known_straight(int cells)
{
  measure_wait();
  if (!failsafe_is_stopped()) {
    trajectory_start_straight(cells * CELL_MM, KNOWN_MAX_SPEED_MM_S, SEARCH_SPEED_MM_S, KNOWN_ACCEL_MM_S2);
    wait_until_done();
  }
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
  /* 前に壁があれば（行き止まりなど）、止まる距離を前壁補正で直す */
  float stop_mm = HALF_CELL_MM + front_correction_mm();
  trajectory_start_straight(stop_mm, SEARCH_SPEED_MM_S, 0.0f, SEARCH_ACCEL_MM_S2);
  wait_until_done();
  if (failsafe_is_stopped()) {
    return;
  }
  timebase_wait_ms(STOP_SETTLE_MS);  /* 目標の速さ 0 のまま制御を続けて、ぴたりと止める */
  control_stop();                    /* 制御をやめて、モーターを止める */
}

/* 止まっている → 前に壁があれば前壁合わせをして、その場で向きを変えて（turn）、
 * 次の区画の入口の辺まで、速くしながら進む。U ターンは、左に 90° 回るたびに、前に壁があれば前壁合わせをする */
void run_search_go_from_center(run_turn_t turn)
{
  if (failsafe_is_stopped()) {
    return;
  }
  control_start();  /* 止まっているので、制御を始め直す */
  align_if_front_wall();
  if (turn == RUN_TURN_LEFT) {
    pivot(90.0f);
  } else if (turn == RUN_TURN_RIGHT) {
    pivot(-90.0f);
  } else if (turn == RUN_TURN_BACK) {
    pivot(90.0f);           /* 1 回目の 90°。前には、もとの右の壁が来る */
    align_if_front_wall();
    pivot(90.0f);           /* 2 回目の 90°。これで反対を向く */
  }
  go_straight(HALF_CELL_MM);
  mark_return();
}

/* ---------------------------------------------------------------------------
 * 最短走行の動き（モード1。sys/shortest.c が使う）
 * ------------------------------------------------------------------------- */

/* 最短走行を始める。ジャイロのゼロ点を測り直して、制御とログを始める（機体はまだ動かない） */
void run_fast_start(void)
{
  if (failsafe_is_stopped()) {
    return;
  }
  start_control();
}

/* distance_mm まっすぐ進む。max_speed_mm_s まで速くしてから、end_speed_mm_s（次のターンの速さ）まで遅くして戻る。
 * 横の壁に沿って走る補正を入れる */
void run_fast_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2)
{
  if (failsafe_is_stopped()) {
    return;
  }
  control_set_wall_control(true);
  trajectory_start_straight(distance_mm, max_speed_mm_s, end_speed_mm_s, accel_mm_s2);
  wait_until_done();
}

/* 斜めに distance_mm まっすぐ進む（run_fast_straight と同じ。横の壁に沿って走る補正は入れない） */
void run_fast_diag_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2)
{
  if (failsafe_is_stopped()) {
    return;
  }
  control_set_wall_control(false);
  trajectory_start_straight(distance_mm, max_speed_mm_s, end_speed_mm_s, accel_mm_s2);
  wait_until_done();
}

/* ターンの速さで、前距離 pre_mm → 角度 angle_deg（左が +）のスラローム → 後距離 post_mm と走る。
 * 前距離・後距離はまっすぐなので、加速度は何でもよい（速さは変わらない）。横の壁に沿って走る補正は入れない */
static void fast_turn(float angle_deg, float omega_deg_s, float pre_mm, float post_mm)
{
  if (failsafe_is_stopped()) {
    return;
  }
  control_set_wall_control(false);
  trajectory_start_straight(pre_mm, FAST_TURN_SPEED_MM_S, FAST_TURN_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
  wait_until_done();
  if (failsafe_is_stopped()) {
    return;
  }
  trajectory_start_slalom(angle_deg, FAST_TURN_SPEED_MM_S, omega_deg_s, FAST_TURN_ALPHA_DEG_S2);
  wait_until_done();
  if (failsafe_is_stopped()) {
    return;
  }
  trajectory_start_straight(post_mm, FAST_TURN_SPEED_MM_S, FAST_TURN_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
  wait_until_done();
}

/* 左に曲がるなら angle_deg、右なら −angle_deg */
static float signed_angle(float angle_deg, bool is_left)
{
  if (is_left) {
    return angle_deg;
  }
  return -angle_deg;
}

/* 大回り・斜めのターンを 1 つ走る（is_left が true なら左）。ターンの速さで走っているところから呼ぶ。
 * 始める位置と終わる位置は、config/turn_params.h の先頭のコメントと test/test_turn.c の表 */
void run_fast_turn(run_fast_turn_t turn, bool is_left)
{
  if (turn == RUN_FAST_TURN_BIG90) {
    fast_turn(signed_angle(90.0f, is_left), BIG90_OMEGA_DEG_S, BIG90_PRE_MM, BIG90_POST_MM);
  } else if (turn == RUN_FAST_TURN_BIG180) {
    fast_turn(signed_angle(180.0f, is_left), BIG180_OMEGA_DEG_S, BIG180_PRE_MM, BIG180_POST_MM);
  } else if (turn == RUN_FAST_TURN_IN45) {
    fast_turn(signed_angle(45.0f, is_left), IN45_OMEGA_DEG_S, IN45_PRE_MM, IN45_POST_MM);
  } else if (turn == RUN_FAST_TURN_IN135) {
    fast_turn(signed_angle(135.0f, is_left), IN135_OMEGA_DEG_S, IN135_PRE_MM, IN135_POST_MM);
  } else if (turn == RUN_FAST_TURN_OUT45) {
    fast_turn(signed_angle(45.0f, is_left), OUT45_OMEGA_DEG_S, OUT45_PRE_MM, OUT45_POST_MM);
  } else if (turn == RUN_FAST_TURN_OUT135) {
    fast_turn(signed_angle(135.0f, is_left), OUT135_OMEGA_DEG_S, OUT135_PRE_MM, OUT135_POST_MM);
  } else if (turn == RUN_FAST_TURN_V90) {
    fast_turn(signed_angle(90.0f, is_left), V90_OMEGA_DEG_S, V90_PRE_MM, V90_POST_MM);
  }
}

/* ターンの速さで走っている → 探索と同じスラロームで左 90° */
void run_fast_turn_left(void)
{
  slalom(SLALOM_ANGLE_DEG);
}

/* ターンの速さで走っている → 探索と同じスラロームで右 90° */
void run_fast_turn_right(void)
{
  slalom(-SLALOM_ANGLE_DEG);
}

/* distance_mm 進みながら止まり、モーターの出力を 0 にして、ログを止める（run_fast_stop・run_fast_diag_stop の中身） */
static void stop_after(float distance_mm, float max_speed_mm_s, float accel_mm_s2)
{
  if (!failsafe_is_stopped()) {
    trajectory_start_straight(distance_mm, max_speed_mm_s, 0.0f, accel_mm_s2);
    wait_until_done();
  }
  if (!failsafe_is_stopped()) {
    timebase_wait_ms(STOP_SETTLE_MS);  /* 目標の速さ 0 のまま制御を続けて、ぴたりと止める */
  }
  control_stop();
  control_set_wall_control(false);
  log_stop();
}

/* distance_mm 進みながら止まり（max_speed_mm_s まで速くしてもよい）、モーターの出力を 0 にして、ログを止める。
 * 横の壁に沿って走る補正は入れる */
void run_fast_stop(float distance_mm, float max_speed_mm_s, float accel_mm_s2)
{
  control_set_wall_control(true);
  stop_after(distance_mm, max_speed_mm_s, accel_mm_s2);
}

/* 斜めに distance_mm 進みながら止まる（run_fast_stop と同じ。横の壁に沿って走る補正は入れない） */
void run_fast_diag_stop(float distance_mm, float max_speed_mm_s, float accel_mm_s2)
{
  control_set_wall_control(false);
  stop_after(distance_mm, max_speed_mm_s, accel_mm_s2);
}
