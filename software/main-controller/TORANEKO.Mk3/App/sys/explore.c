/*
 * explore.c
 * 探索走行（モード0）：迷路の処理（maze/）に次の行き先を聞き、機体を動かす処理（ctrl/run）で 1 区画ずつ動かす。
 * 迷路の処理と機体を動かす処理をつなぐ、ただ 1 つの場所。
 *
 * ■ 機体の位置
 * 位置は s_x, s_y, s_dir で覚える。機体は次のどちらかの状態にいる（ctrl/run.h、maze/search.c と同じ考え方）。
 *   走っている  （s_stopped が false）：区画 (s_x, s_y) の入口の辺にいて、向き s_dir でその区画に入るところ。
 *                                       ここでセンサがその区画の前・左・右の壁を見て、記録する
 *   止まっている（s_stopped が true） ：区画 (s_x, s_y) の真ん中で、向き s_dir を向いて止まっている
 *
 * ■ 止まるところ
 *   - U ターン（止まって、後ろを向いて発進する）
 *   - 未知壁探索で計算するとき（ダイクストラは重いので、止まっているときだけ計算する。KOGUMA と同じ）
 *   - 終わり（スタートに戻ったとき、片道でゴールに着いたとき）
 *
 * ■ 既知区間加速（No.4 だけ。2026-10-27 に追加）
 * 走っているとき（行き・未知壁を見に行くあいだ・帰り）、今いる区画から先に「見たことのある区画をまっすぐ進む」が 2 区画以上続くなら、
 * 1 区画ずつではなく、まとめて速く走る（run_search_known_straight()。速さは config/run_params.h の KNOWN_*）。
 * 見たことのある区画では、壁がもうわかっているので、1 区画ずつ止まらずに考えても同じ道になる（maze/path.c の
 * path_make_known と同じ考え方）。曲がるところは、ふだんどおり 1 区画ずつ小回りで走る。
 * まっすぐ進む区画を数えるだけにして、経路（path_t。約 4KB）は作らない（RAM が残り少ないため）。
 *
 * ■ Flash への保存（MODES.md の 5）
 *   - スタートに戻ったとき、片道でゴールに止まったとき
 *   - フェイルセーフで止まったとき（最後に壁を記録した 5 区画ぶんを、記録する前の状態に戻してから）
 */
#include "sys/explore.h"

#include <stdbool.h>

#include "ctrl/run.h"
#include "ctrl/wall_ctrl.h"
#include "maze/maze_map.h"
#include "maze/search.h"
#include "sys/failsafe.h"
#include "sys/flash.h"

/* 1 つの区間（行き、未知壁探索、帰り）で、これだけ動いても終わらなければ、おかしいとみなしてやめる。
 * 壁の読み間違いなどで、同じところを回り続けないようにするため。
 * 32×32 の迷路の区画は全部で 1024 個なので、十分に大きい */
#define MOVE_LIMIT 5000

/* 機体の位置と向き（上の「機体の位置」） */
static int s_x = 0;
static int s_y = 0;
static direction_t s_dir = DIR_NORTH;
static bool s_stopped = true;

static bool s_use_known = false;  /* 既知区間加速をするか（No.4 のときだけ true） */

/* ---------------------------------------------------------------------------
 * 1 区画ぶんの動き
 * ------------------------------------------------------------------------- */

/* 今入った区画の前・左・右の壁を、センサで見て記録する。区画の入口の辺に着いたときに呼ぶ */
static void record_walls_here(void)
{
  bool wall_front = wall_ctrl_has_wall_front();
  bool wall_left = wall_ctrl_has_wall_left();
  bool wall_right = wall_ctrl_has_wall_right();
  maze_map_record_walls(s_x, s_y, s_dir, wall_front, wall_left, wall_right);
}

/* 今入った区画の真ん中で止まる。もう止まっていれば何もしない */
static void stop_here(void)
{
  if (s_stopped) {
    return;
  }
  run_search_stop_at_center();
  s_stopped = true;
}

/* 止まったあとに返す結果。止まる途中でぶつかって（フェイルセーフで止まって）いたら、
 * result の代わりに EXPLORE_RESULT_FAILSAFE を返す（5 区画ぶんを戻して保存するため） */
static explore_result_t result_after_stop(explore_result_t result)
{
  if (failsafe_is_stopped()) {
    return EXPLORE_RESULT_FAILSAFE;
  }
  return result;
}

/* 動き action をしたあとの向き */
static direction_t direction_after(direction_t dir, action_t action)
{
  if (action == ACTION_TURN_LEFT) {
    return maze_turn_left(dir);
  }
  if (action == ACTION_TURN_RIGHT) {
    return maze_turn_right(dir);
  }
  if (action == ACTION_UTURN) {
    return maze_turn_back(dir);
  }
  return dir;  /* ACTION_FORWARD */
}

/* 止まっているところから発進するときの、向きの変え方 */
static run_turn_t turn_from_center(action_t action)
{
  if (action == ACTION_TURN_LEFT) {
    return RUN_TURN_LEFT;
  }
  if (action == ACTION_TURN_RIGHT) {
    return RUN_TURN_RIGHT;
  }
  if (action == ACTION_UTURN) {
    return RUN_TURN_BACK;
  }
  return RUN_TURN_NONE;  /* ACTION_FORWARD */
}

/* 動き action で、今の区画を出て、次の区画の入口の辺まで動く。位置と向きも進める。
 *   止まっているとき：その場で向きを変えて発進する
 *   走っているとき  ：直進とスラロームはそのまま走る。U ターンは、止まってから後ろを向いて発進する
 * action に ACTION_STOP を渡してはいけない（呼ぶ側で調べる） */
static void move(action_t action)
{
  if (s_stopped) {
    run_search_go_from_center(turn_from_center(action));
  } else if (action == ACTION_FORWARD) {
    run_search_straight();
  } else if (action == ACTION_TURN_LEFT) {
    run_search_turn_left();
  } else if (action == ACTION_TURN_RIGHT) {
    run_search_turn_right();
  } else {
    /* ACTION_UTURN：止まってから、後ろを向いて発進する（止まる途中でぶつかったら、発進しない） */
    run_search_stop_at_center();
    if (!failsafe_is_stopped()) {
      run_search_go_from_center(RUN_TURN_BACK);
    }
  }

  s_stopped = false;
  s_dir = direction_after(s_dir, action);
  s_x = maze_next_x(s_x, s_dir);
  s_y = maze_next_y(s_y, s_dir);
}

/* 今いる区画（入口の辺にいる）から、見たことのある区画をまっすぐ何区画続けて進めるかを数える（既知区間加速）。
 * 見たことのない区画、目的地、まっすぐ以外の動きになる区画の手前で数えるのをやめる。
 * 歩数マップ（search_update_step_map()）は、呼ぶ前に作っておくこと */
static int count_known_straight(void)
{
  int x = s_x;
  int y = s_y;
  int count = 0;
  while (count < MAZE_SIZE) {
    if (!maze_map_is_visited(x, y)) {
      break;
    }
    if (search_is_in_target(x, y)) {
      break;
    }
    if (search_decide_action(x, y, s_dir) != ACTION_FORWARD) {
      break;
    }
    count = count + 1;
    x = maze_next_x(x, s_dir);
    y = maze_next_y(y, s_dir);
  }
  return count;
}

/* 既知区間加速：走っているときに、まっすぐ進む見たことのある区画が 2 区画以上続くなら、まとめて速く走る。
 * 走ったら true（位置を進め、着いた区画の壁を記録してある）。走らなかったら false */
static bool try_known_straight(void)
{
  if (!s_use_known || s_stopped) {
    return false;
  }
  int count = count_known_straight();
  if (count < 2) {
    return false;  /* 1 区画だけなら、ふだんの直進と同じ */
  }
  run_search_known_straight(count);
  for (int i = 0; i < count; i++) {
    s_x = maze_next_x(s_x, s_dir);
    s_y = maze_next_y(s_y, s_dir);
  }
  if (!failsafe_is_stopped()) {
    record_walls_here();
  }
  return true;
}

/* ---------------------------------------------------------------------------
 * 区間ごとの走り方
 * ------------------------------------------------------------------------- */

/* 目的地（search_set_target() で決めた範囲）に着くまで、足立法で 1 区画ずつ走る。
 * 今いる区画の壁は、記録してあること。止まっているところから始めてもよい。
 * 着いたら EXPLORE_RESULT_DONE（その区画の入口の辺にいて、止まらない） */
static explore_result_t run_to_target(void)
{
  for (int move_count = 0; move_count < MOVE_LIMIT; move_count++) {
    if (search_is_in_target(s_x, s_y)) {
      return EXPLORE_RESULT_DONE;
    }

    search_update_step_map();
    if (try_known_straight()) {
      if (failsafe_is_stopped()) {
        return EXPLORE_RESULT_FAILSAFE;
      }
      continue;  /* 見たことのある区画をまとめて走った。着いた区画で、また考える */
    }
    action_t action = search_decide_action(s_x, s_y, s_dir);
    if (action == ACTION_STOP) {
      stop_here();  /* どこへ進んでも目的地に着けない */
      return result_after_stop(EXPLORE_RESULT_NO_ROUTE);
    }

    move(action);
    if (failsafe_is_stopped()) {
      return EXPLORE_RESULT_FAILSAFE;
    }
    record_walls_here();
  }

  stop_here();  /* 動きすぎた */
  return result_after_stop(EXPLORE_RESULT_NO_ROUTE);
}

/* 未知壁探索の経路を、止まって計算し直す（ダイクストラは重いので、止まっているときだけ計算する）。
 * ゴールへの道が見つからなければ、このあとの search_check_unknown_walls() が UNKNOWN_WALL_NO_PLAN を返す。
 * 止まる途中でぶつかった（フェイルセーフで止まった）ときは、計算せずに false を返す */
static bool stop_and_plan(int goal_x, int goal_y, int goal_width, int goal_height)
{
  stop_here();
  if (failsafe_is_stopped()) {
    return false;
  }
  search_plan_unknown_walls(goal_x, goal_y, goal_width, goal_height);
  return true;
}

/* 未知壁探索（流れは KOGUMA_PORTING.md の「ToraNeko の未知壁探索の流れ」）。ゴールの区画の入口の辺に着いたところから呼ぶ。
 * 最短経路の上の見ていない壁を、なくなるまで見に行く。なくなったら、止まらずに EXPLORE_RESULT_DONE を返す
 * （このあと、そのままスタートへ戻る）。
 * ゴールへの道が見つからないなど、おかしなことが起きたときも、未知壁探索をやめて EXPLORE_RESULT_DONE を返す
 * （止まったまま。このあとスタートへ戻る） */
static explore_result_t scan_unknown_walls(int goal_x, int goal_y, int goal_width, int goal_height)
{
  /* ゴールの区画の真ん中で止まって、見に行く壁を計算する */
  if (!stop_and_plan(goal_x, goal_y, goal_width, goal_height)) {
    return EXPLORE_RESULT_FAILSAFE;
  }

  for (int move_count = 0; move_count < MOVE_LIMIT; move_count++) {
    int cell_x;
    int cell_y;
    direction_t wall_dir;
    unknown_wall_result_t result = search_check_unknown_walls(&cell_x, &cell_y, &wall_dir);

    if (result == UNKNOWN_WALL_NONE) {
      /* 見に行く壁がなくなった：止まらずにスタートへ戻る（最短経路は、最短走行のときに計算する） */
      return EXPLORE_RESULT_DONE;
    }
    if (result == UNKNOWN_WALL_NO_PLAN) {
      /* ゴールへの道が見つからなかった（壁の読み間違いなど）：未知壁探索をやめる */
      stop_here();
      return result_after_stop(EXPLORE_RESULT_DONE);
    }
    if (result == UNKNOWN_WALL_BLOCKED) {
      /* 経路の上に壁があった（経路がふさがれた）：止まって計算し直す */
      if (!stop_and_plan(goal_x, goal_y, goal_width, goal_height)) {
        return EXPLORE_RESULT_FAILSAFE;
      }
      continue;
    }

    /* UNKNOWN_WALL_FOUND：見ていない壁の手前の区画 (cell_x, cell_y) へ向かう。
     * その区画に入れば、センサでその壁を見られる */
    search_set_target(cell_x, cell_y, 1, 1);
    search_update_step_map();
    if (try_known_straight()) {
      if (failsafe_is_stopped()) {
        return EXPLORE_RESULT_FAILSAFE;
      }
      continue;  /* 見たことのある区画をまとめて走った。着いた区画で、また見に行く壁を確かめる */
    }
    action_t action = search_decide_action(s_x, s_y, s_dir);
    if (action == ACTION_STOP) {
      /* その区画へ行けない（普通は起きない）。
       * 止まっているなら、計算し直したばかりでも行けないので、未知壁探索をやめる
       * （この関数で止まるのは、計算し直すときだけ）。走っているなら、止まって計算し直す */
      if (s_stopped) {
        return EXPLORE_RESULT_DONE;
      }
      if (!stop_and_plan(goal_x, goal_y, goal_width, goal_height)) {
        return EXPLORE_RESULT_FAILSAFE;
      }
      continue;
    }
    if (action == ACTION_UTURN && !s_stopped) {
      /* U ターンでは止まるので、止まったついでに計算し直す（行き先が変わるかもしれない） */
      if (!stop_and_plan(goal_x, goal_y, goal_width, goal_height)) {
        return EXPLORE_RESULT_FAILSAFE;
      }
      continue;
    }

    move(action);
    if (failsafe_is_stopped()) {
      return EXPLORE_RESULT_FAILSAFE;
    }
    record_walls_here();
  }

  stop_here();  /* 動きすぎた：未知壁探索をやめる */
  return result_after_stop(EXPLORE_RESULT_DONE);
}

/* 途中で終わったときの後始末をして、result をそのまま返す。
 * フェイルセーフで止まったときは、最後に壁を記録した 5 区画ぶんを戻してから、迷路を Flash に保存する
 * （ぶつかる直前に読み間違えたかもしれない壁を残さないため。モーターは止まっている） */
static explore_result_t finish_early(explore_result_t result)
{
  if (result == EXPLORE_RESULT_FAILSAFE) {
    maze_map_undo_recent_cells();
    flash_save_maze();
  }
  return result;
}

/* 最後まで走ったときの後始末。今いる区画の真ん中で止まって、迷路を Flash に保存する */
static explore_result_t finish_done(void)
{
  stop_here();
  if (failsafe_is_stopped()) {
    return finish_early(EXPLORE_RESULT_FAILSAFE);  /* 止まる途中でぶつかった */
  }
  flash_save_maze();  /* 止まったあとはモーターの出力が 0 なので、CPU が止まっても動かない（run.h） */
  return EXPLORE_RESULT_DONE;
}

/* ---------------------------------------------------------------------------
 * 探索走行
 * ------------------------------------------------------------------------- */

/* 探索走行をする。機体をスタート区画に北向きに置いてから呼ぶ。走り終わるまで戻ってこない。
 * type で走り方を選ぶ（explore.h）。ゴールは、左下の区画 (goal_x, goal_y) と幅・高さ [区画] */
explore_result_t explore_run(explore_type_t type, int goal_x, int goal_y, int goal_width, int goal_height)
{
  explore_result_t result;

  /* 既知区間加速は No.4 だけ。No.4 は、ほかは未知壁探索（No.3）と同じ */
  s_use_known = false;
  if (type == EXPLORE_UNKNOWN_WALLS_KNOWN) {
    s_use_known = true;
    type = EXPLORE_UNKNOWN_WALLS;
  }

  /* この走行で記録した壁だけを、フェイルセーフのときに戻すようにする */
  maze_map_clear_recent_cells();

  /* 1. スタート区画から走り出し、北隣の区画 (0, 1) の入口の辺まで進む */
  run_search_start();
  s_x = 0;
  s_y = 1;
  s_dir = DIR_NORTH;
  s_stopped = false;
  if (failsafe_is_stopped()) {
    return finish_early(EXPLORE_RESULT_FAILSAFE);
  }
  record_walls_here();

  /* 2. 行き：ゴールまで */
  search_set_target(goal_x, goal_y, goal_width, goal_height);
  result = run_to_target();
  if (result != EXPLORE_RESULT_DONE) {
    return finish_early(result);
  }

  /* 3. 片道：ゴールの区画の真ん中で止まって終わり */
  if (type == EXPLORE_ONE_WAY) {
    return finish_done();
  }

  /* 4. 未知壁探索 */
  if (type == EXPLORE_UNKNOWN_WALLS) {
    result = scan_unknown_walls(goal_x, goal_y, goal_width, goal_height);
    if (result != EXPLORE_RESULT_DONE) {
      return finish_early(result);
    }
  }

  /* 5. 帰り：スタートまで戻って止まる */
  search_set_target(0, 0, 1, 1);
  result = run_to_target();
  if (result != EXPLORE_RESULT_DONE) {
    return finish_early(result);
  }
  return finish_done();
}
