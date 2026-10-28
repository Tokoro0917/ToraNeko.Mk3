/*
 * shortest.c
 * 最短走行（モード1。MODES.md）
 *
 * RAM の迷路で、スタートからゴールまでの最短経路を作り、その経路のとおりに走って、ゴールの区画で止まる。
 * まだ見ていない壁は「ある」とみなす。No. で走り方と速さを選ぶ（config/run_params.h の SHORTEST1〜5）。
 *
 * ■ No.1〜3：曲がるところは、探索と同じ小回りだけ
 *   1. 経路を作る（maze/path.c の path_make_shortest_small_turn()）
 *   2. 直進は最高速まで速くしてから、小回りの速さ（300mm/s）まで遅くする。左・右は探索と同じスラローム 90°
 *   3. 最後の直進で、ゴールの最初の区画のまん中で止まる
 *
 * ■ No.4・5：大回り・斜めのターンで曲がる（2026-10-28 に作った。ターンの値は仮で、実機はまだ）
 *   1. 経路を作る（maze/path.c の path_make_shortest()。大回り 90°・180°、斜め入り・斜め出 45°・135°、V90°、斜めの直進）
 *   2. 直進・斜めの直進は、最高速まで速くしてから、ターンの速さ（config/turn_params.h、今は 500mm/s）まで遅くする。
 *      ただし、短くてターンの速さまで上げきれないとき（スタートの直後など）は、上げられる速さで終わり、
 *      次のターンをその速さで曲がる（同じ形のまま遅く曲がる。ctrl/run.c の fast_turn()）
 *   3. 最後が直進なら、ゴールの最初の区画のまん中で止まる。最後がターン（ゴールにターンで入る）なら、
 *      区画のまん中に着いたところから SHORTEST_STOP_ACCEL_MM_S2 で止まる（区画の中で止まる）
 *   小回り（MOTION_TURN_LEFT・RIGHT）は経路に残らない（PC で 500 個の迷路を調べて 0 個）が、残ったら走らずに「道がない」にする
 *
 * 経路は、その走り方と速さで本当にかかる時間が短いものを選ぶ（ダイクストラの走行時間のコスト。maze/dijkstra.c）。
 *
 * ■ スタートの位置
 * 経路は、スタート区画 (0,0) の南の辺から始まる。機体は、お尻を後ろの壁に付けて置くので、
 * タイヤの軸は南の辺より HALF_CELL_MM − START_OFFSET_MM（35mm）前にある。最初の直進は、その分だけ短くする。
 */
#include "sys/shortest.h"

#include <math.h>

#include "config/run_params.h"
#include "config/turn_params.h"
#include "ctrl/run.h"
#include "maze/dijkstra.h"
#include "maze/path.h"
#include "sys/failsafe.h"
#include "sys/goal.h"

static path_t s_path;  /* 最短経路（大きいので、関数の中ではなくここに置く） */

/* 大回り・斜めで走る No. か（No.4・5） */
static bool uses_diagonal(int number)
{
  if (number >= 4) {
    return true;
  }
  return false;
}

/* No. ごとの、直進の最高速 [mm/s] */
static float max_speed_mm_s(int number)
{
  if (number == 2) {
    return SHORTEST2_MAX_SPEED_MM_S;
  }
  if (number == 3) {
    return SHORTEST3_MAX_SPEED_MM_S;
  }
  if (number == 4) {
    return SHORTEST4_MAX_SPEED_MM_S;
  }
  if (number == 5) {
    return SHORTEST5_MAX_SPEED_MM_S;
  }
  return SHORTEST1_MAX_SPEED_MM_S;
}

/* No. ごとの、直進の加速度 [mm/s²] */
static float accel_mm_s2(int number)
{
  if (number == 2) {
    return SHORTEST2_ACCEL_MM_S2;
  }
  if (number == 3) {
    return SHORTEST3_ACCEL_MM_S2;
  }
  if (number == 4) {
    return SHORTEST4_ACCEL_MM_S2;
  }
  if (number == 5) {
    return SHORTEST5_ACCEL_MM_S2;
  }
  return SHORTEST1_ACCEL_MM_S2;
}

/* No. ごとの、斜めの直進の最高速 [mm/s]（No.4・5 だけ使う） */
static float diag_max_speed_mm_s(int number)
{
  if (number == 5) {
    return SHORTEST5_DIAG_MAX_SPEED_MM_S;
  }
  return SHORTEST4_DIAG_MAX_SPEED_MM_S;
}

/* No. ごとの、斜めの直進の加速度 [mm/s²]（No.4・5 だけ使う） */
static float diag_accel_mm_s2(int number)
{
  if (number == 5) {
    return SHORTEST5_DIAG_ACCEL_MM_S2;
  }
  return SHORTEST4_DIAG_ACCEL_MM_S2;
}

/* 経路の中に、小回り（探索用の左右のターン）があるか（No.4・5 では走れない） */
static bool has_small_turn(void)
{
  for (int i = 0; i < s_path.count; i++) {
    if (s_path.steps[i].motion == MOTION_TURN_LEFT || s_path.steps[i].motion == MOTION_TURN_RIGHT) {
      return true;
    }
  }
  return false;
}

/* 最短経路を作って s_path に入れる。number は No.（直進の速さで、選ぶ経路が変わる）。走れる経路ができなければ false。
 * 最初の動きは直進でないといけない（スタート区画の中で曲がる動きは作っていない） */
static bool make_path(int number, int goal_x, int goal_y)
{
  /* ダイクストラを、この No. の速さで走る時間で比べるようにしてから経路を作り、終わったら元に戻す
   * （未知壁探索など、ほかでダイクストラを使うときに影響しないように） */
  bool found = false;
  if (uses_diagonal(number)) {
    dijkstra_set_time_cost_diagonal(FAST_TURN_SPEED_MM_S, max_speed_mm_s(number), accel_mm_s2(number),
                                    diag_max_speed_mm_s(number), diag_accel_mm_s2(number));
    found = path_make_shortest(goal_x, goal_y, GOAL_WIDTH, GOAL_HEIGHT, &s_path);
  } else {
    dijkstra_set_time_cost_small_turn(SEARCH_SPEED_MM_S, max_speed_mm_s(number), accel_mm_s2(number));
    found = path_make_shortest_small_turn(goal_x, goal_y, GOAL_WIDTH, GOAL_HEIGHT, &s_path);
  }
  dijkstra_set_step_cost();
  if (!found) {
    return false;
  }
  if (uses_diagonal(number) && has_small_turn()) {
    return false;  /* 小回りが残った（先頭のコメント。普通は起きない） */
  }
  if (s_path.count == 0) {
    return false;
  }
  if (s_path.steps[0].motion != MOTION_STRAIGHT) {
    return false;
  }
  return true;
}

/* スタートからゴールへの道があるか（モードを決めたときに調べる。number は No.） */
bool shortest_has_route(int number, int goal_x, int goal_y)
{
  return make_path(number, goal_x, goal_y);
}

/* 経路の i 番目の直進の長さ [mm]。最初の直進は、スタートの置き方の分だけ短くする（先頭のコメント） */
static float straight_distance_mm(int i)
{
  float distance_mm = s_path.steps[i].half_count * HALF_CELL_MM;
  if (i == 0) {
    distance_mm = distance_mm - (HALF_CELL_MM - START_OFFSET_MM);
  }
  return distance_mm;
}

/* No.1〜3：経路の i 番目の動きを走る。最後の動き（ゴールの区画のまん中まで進む直進）では止まる */
static void run_small_turn_step(int i, float max_speed, float accel)
{
  motion_t motion = s_path.steps[i].motion;
  bool is_last = (i == s_path.count - 1);

  if (motion == MOTION_STRAIGHT && is_last) {
    run_fast_stop(straight_distance_mm(i), max_speed, accel);
  } else if (motion == MOTION_STRAIGHT) {
    run_fast_straight(straight_distance_mm(i), max_speed, SEARCH_SPEED_MM_S, accel);  /* 次の小回りの速さで終わる */
  } else if (motion == MOTION_TURN_LEFT) {
    run_fast_turn_left();
  } else if (motion == MOTION_TURN_RIGHT) {
    run_fast_turn_right();
  }
}

/* No.4・5：動き motion が、どの大回り・斜めのターンか */
static run_fast_turn_t fast_turn_kind(motion_t motion)
{
  if (motion == MOTION_BIG_TURN_LEFT_90 || motion == MOTION_BIG_TURN_RIGHT_90) {
    return RUN_FAST_TURN_BIG90;
  }
  if (motion == MOTION_BIG_TURN_LEFT_180 || motion == MOTION_BIG_TURN_RIGHT_180) {
    return RUN_FAST_TURN_BIG180;
  }
  if (motion == MOTION_DIAG_IN45_LEFT || motion == MOTION_DIAG_IN45_RIGHT) {
    return RUN_FAST_TURN_IN45;
  }
  if (motion == MOTION_DIAG_IN135_LEFT || motion == MOTION_DIAG_IN135_RIGHT) {
    return RUN_FAST_TURN_IN135;
  }
  if (motion == MOTION_DIAG_OUT45_LEFT || motion == MOTION_DIAG_OUT45_RIGHT) {
    return RUN_FAST_TURN_OUT45;
  }
  if (motion == MOTION_DIAG_OUT135_LEFT || motion == MOTION_DIAG_OUT135_RIGHT) {
    return RUN_FAST_TURN_OUT135;
  }
  return RUN_FAST_TURN_V90;
}

/* No.4・5：動き motion が左に曲がるターンか */
static bool is_left_turn(motion_t motion)
{
  if (motion == MOTION_BIG_TURN_LEFT_90 || motion == MOTION_BIG_TURN_LEFT_180
      || motion == MOTION_DIAG_IN45_LEFT || motion == MOTION_DIAG_IN135_LEFT
      || motion == MOTION_DIAG_OUT45_LEFT || motion == MOTION_DIAG_OUT135_LEFT
      || motion == MOTION_DIAG_V90_LEFT) {
    return true;
  }
  return false;
}

/* 速さ start_mm_s から、distance_mm で上げられる速さ（加速度 accel_mm_s2）。ターンの速さより上にはしない。
 * 直進の終わりの速さに使う（上げきれない速さを頼むと、着いたときに速さが飛んでしまうので） */
static float end_speed_mm_s(float start_mm_s, float distance_mm, float accel_mm_s2)
{
  float reachable_mm_s = sqrtf(start_mm_s * start_mm_s + 2.0f * accel_mm_s2 * distance_mm);
  if (reachable_mm_s > FAST_TURN_SPEED_MM_S) {
    return FAST_TURN_SPEED_MM_S;
  }
  return reachable_mm_s;
}

/* No.4・5：経路を前から順に走る。speed_mm_s は今の速さ（直進が終わった速さで、次のターンを曲がる） */
static void run_diagonal_path(int number)
{
  float max_speed = max_speed_mm_s(number);
  float accel = accel_mm_s2(number);
  float diag_max_speed = diag_max_speed_mm_s(number);
  float diag_accel = diag_accel_mm_s2(number);
  float speed_mm_s = 0.0f;
  bool last_was_turn = false;

  for (int i = 0; i < s_path.count; i++) {
    motion_t motion = s_path.steps[i].motion;
    bool is_last = (i == s_path.count - 1);
    last_was_turn = false;

    if (motion == MOTION_STRAIGHT && is_last) {
      run_fast_stop(straight_distance_mm(i), max_speed, accel);  /* ゴールの区画のまん中で止まる */
      speed_mm_s = 0.0f;
    } else if (motion == MOTION_STRAIGHT) {
      float distance_mm = straight_distance_mm(i);
      float end_mm_s = end_speed_mm_s(speed_mm_s, distance_mm, accel);
      run_fast_straight(distance_mm, max_speed, end_mm_s, accel);
      speed_mm_s = end_mm_s;
    } else if (motion == MOTION_DIAG_STRAIGHT) {
      float distance_mm = s_path.steps[i].half_count * DIAG_STEP_MM;
      float end_mm_s = end_speed_mm_s(speed_mm_s, distance_mm, diag_accel);
      run_fast_diag_straight(distance_mm, diag_max_speed, end_mm_s, diag_accel);
      speed_mm_s = end_mm_s;
    } else {
      run_fast_turn(fast_turn_kind(motion), is_left_turn(motion), speed_mm_s);
      last_was_turn = true;
    }
    if (failsafe_is_stopped()) {
      return;
    }
  }

  if (last_was_turn) {
    /* ゴールにターンで入って、区画のまん中に今の速さで着いた：そこから止まる */
    float stop_mm = speed_mm_s * speed_mm_s / (2.0f * SHORTEST_STOP_ACCEL_MM_S2);
    run_fast_stop(stop_mm, speed_mm_s, SHORTEST_STOP_ACCEL_MM_S2);
  }
}

/* 最短走行をする（流れは先頭のコメント）。number は No.（走り方と速さを選ぶ） */
shortest_result_t shortest_run(int number, int goal_x, int goal_y)
{
  if (!make_path(number, goal_x, goal_y)) {
    return SHORTEST_RESULT_NO_ROUTE;
  }
  float max_speed = max_speed_mm_s(number);
  float accel = accel_mm_s2(number);

  run_fast_start();
  if (uses_diagonal(number)) {
    run_diagonal_path(number);
  } else {
    for (int i = 0; i < s_path.count; i++) {
      run_small_turn_step(i, max_speed, accel);
      if (failsafe_is_stopped()) {
        break;
      }
    }
  }

  if (failsafe_is_stopped()) {
    run_fast_stop(0.0f, 0.0f, accel);  /* 制御とログを止める（フェイルセーフのときは、動かさずに止めるだけ） */
    return SHORTEST_RESULT_FAILSAFE;
  }
  return SHORTEST_RESULT_DONE;
}
