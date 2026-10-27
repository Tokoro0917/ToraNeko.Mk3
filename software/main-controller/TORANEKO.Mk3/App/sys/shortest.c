/*
 * shortest.c
 * 最短走行（モード1。MODES.md）
 *
 * RAM の迷路で、スタートからゴールまでの最短経路を作り、その経路のとおりに走って、ゴールの区画のまん中で止まる。
 *   1. 経路を作る（maze/path.c の path_make_shortest_small_turn()）。まだ見ていない壁は「ある」とみなす
 *   2. 経路の動きを、前から順に 1 つずつ走る（ctrl/run.c の run_fast_*）
 *      - 直進      ：最高速まで速くしてから、ターンの速さ（300mm/s）まで遅くする。最後の直進では止まる
 *      - 左・右    ：探索と同じスラローム 90°
 *   3. ゴールの区画のまん中で止まる
 *
 * 大回りと斜めの動きは、まだ実機で合わせていないので使わない（曲がるところは、探索と同じ小回りだけ）。
 * 速さは No. ごとに決める（config/run_params.h の SHORTEST1〜3）。No. が大きいほど速い。
 *
 * ■ スタートの位置
 * 経路は、スタート区画 (0,0) の南の辺から始まる。機体は、お尻を後ろの壁に付けて置くので、
 * タイヤの軸は南の辺より HALF_CELL_MM − START_OFFSET_MM（35mm）前にある。最初の直進は、その分だけ短くする。
 */
#include "sys/shortest.h"

#include "config/run_params.h"
#include "ctrl/run.h"
#include "maze/path.h"
#include "sys/failsafe.h"
#include "sys/goal.h"

static path_t s_path;  /* 最短経路（大きいので、関数の中ではなくここに置く） */

/* No. ごとの、直進の最高速 [mm/s] */
static float max_speed_mm_s(int number)
{
  if (number == 2) {
    return SHORTEST2_MAX_SPEED_MM_S;
  }
  if (number == 3) {
    return SHORTEST3_MAX_SPEED_MM_S;
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
  return SHORTEST1_ACCEL_MM_S2;
}

/* 最短経路を作って s_path に入れる。走れる経路ができなければ false。
 * 最初の動きは直進でないといけない（スタート区画の中で曲がる動きは作っていない） */
static bool make_path(int goal_x, int goal_y)
{
  if (!path_make_shortest_small_turn(goal_x, goal_y, GOAL_WIDTH, GOAL_HEIGHT, &s_path)) {
    return false;
  }
  if (s_path.count == 0) {
    return false;
  }
  if (s_path.steps[0].motion != MOTION_STRAIGHT) {
    return false;
  }
  return true;
}

/* スタートからゴールへの道があるか（モードを決めたときに調べる） */
bool shortest_has_route(int goal_x, int goal_y)
{
  return make_path(goal_x, goal_y);
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

/* 経路の i 番目の動きを走る。最後の動き（ゴールの区画のまん中まで進む直進）では止まる */
static void run_step(int i, float max_speed, float accel)
{
  motion_t motion = s_path.steps[i].motion;
  bool is_last = (i == s_path.count - 1);

  if (motion == MOTION_STRAIGHT && is_last) {
    run_fast_stop(straight_distance_mm(i), max_speed, accel);
  } else if (motion == MOTION_STRAIGHT) {
    run_fast_straight(straight_distance_mm(i), max_speed, accel);
  } else if (motion == MOTION_TURN_LEFT) {
    run_fast_turn_left();
  } else if (motion == MOTION_TURN_RIGHT) {
    run_fast_turn_right();
  }
}

/* 最短走行をする（流れは先頭のコメント）。number は No.（速さを選ぶ） */
shortest_result_t shortest_run(int number, int goal_x, int goal_y)
{
  if (!make_path(goal_x, goal_y)) {
    return SHORTEST_RESULT_NO_ROUTE;
  }
  float max_speed = max_speed_mm_s(number);
  float accel = accel_mm_s2(number);

  run_fast_start();
  for (int i = 0; i < s_path.count; i++) {
    run_step(i, max_speed, accel);
    if (failsafe_is_stopped()) {
      break;
    }
  }

  if (failsafe_is_stopped()) {
    run_fast_stop(0.0f, 0.0f, accel);  /* 制御とログを止める（フェイルセーフのときは、動かさずに止めるだけ） */
    return SHORTEST_RESULT_FAILSAFE;
  }
  return SHORTEST_RESULT_DONE;
}
