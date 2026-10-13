/*
 * path.c
 * 経路（走る動きの並び）を作る
 */
#include "maze/path.h"

#include "maze/search.h"

/* ===========================================================================
 * 経路に動きを足す（共通の部品）
 * ======================================================================== */

/* 経路を空にする */
static void path_clear(path_t *path)
{
  path->count = 0;
}

/* 経路の最後に、直進を half_count 半区画足す。最後の動きが直進なら、その長さに足し込む */
static void path_add_straight(path_t *path, int half_count)
{
  if (half_count <= 0) {
    return;
  }
  int last = path->count - 1;  /* 最後の動きの番号 */
  if (last >= 0 && path->steps[last].motion == MOTION_STRAIGHT) {
    path->steps[last].half_count = path->steps[last].half_count + half_count;
    return;
  }
  if (path->count >= PATH_STEP_MAX) {
    return;  /* 入りきらない（迷路の区画数より長くはならないので、普通は起きない） */
  }
  path->steps[path->count].motion = MOTION_STRAIGHT;
  path->steps[path->count].half_count = half_count;
  path->count = path->count + 1;
}

/* 経路の最後に、直進以外の動きを 1 つ足す */
static void path_add_motion(path_t *path, motion_t motion)
{
  if (path->count >= PATH_STEP_MAX) {
    return;
  }
  path->steps[path->count].motion = motion;
  path->steps[path->count].half_count = 0;
  path->count = path->count + 1;
}

/* ===========================================================================
 * 大回りへの変換
 * ======================================================================== */

/* 動きが「探索用の左ターン」か「探索用の右ターン」か */
static bool is_small_turn(motion_t motion)
{
  if (motion == MOTION_TURN_LEFT || motion == MOTION_TURN_RIGHT) {
    return true;
  }
  return false;
}

/* 経路の i 番目が、長さのある直進か */
static bool is_straight_at(const path_t *path, int i)
{
  if (i < 0 || i >= path->count) {
    return false;
  }
  if (path->steps[i].motion != MOTION_STRAIGHT) {
    return false;
  }
  if (path->steps[i].half_count <= 0) {
    return false;
  }
  return true;
}

/* 経路の最後の直進を半区画短くする。長さが 0 になったら、その直進を消す */
static void shorten_last_straight(path_t *path)
{
  int last = path->count - 1;
  path->steps[last].half_count = path->steps[last].half_count - 1;
  if (path->steps[last].half_count == 0) {
    path->count = path->count - 1;
  }
}

/* 探索用のターンを、できるところだけ大回りに置き換えた経路を作る（src → dst）。
 *
 *   直進 → 左 → 直進          は  直進(半区画短く) → 大回り左 90° → 直進(半区画短く)
 *   直進 → 左 → 左 → 直進     は  直進(半区画短く) → 大回り左 180° → 直進(半区画短く)
 *   （右も同じ）
 *
 * 大回りは、前後の直進を半区画ずつ使って大きな半径で曲がる。
 * そのため、前と後ろの両方に直進があるところだけ置き換える。
 * 後ろの直進を半区画短くするために、src の中身も書き換える。 */
static void convert_to_big_turns(path_t *src, path_t *dst)
{
  path_clear(dst);

  int i = 0;
  while (i < src->count) {
    motion_t motion = src->steps[i].motion;

    if (motion == MOTION_STRAIGHT) {
      /* 直進はそのまま写す（前の大回りで半区画使われて 0 になっていたら写さない） */
      path_add_straight(dst, src->steps[i].half_count);
      i = i + 1;
      continue;
    }

    /* 写し先の最後が直進か（＝このターンの前に直進があるか） */
    bool has_straight_before = is_straight_at(dst, dst->count - 1);

    if (is_small_turn(motion) && has_straight_before && is_straight_at(src, i + 1)) {
      /* 直進 → ターン → 直進：大回り 90° */
      shorten_last_straight(dst);
      if (motion == MOTION_TURN_LEFT) {
        path_add_motion(dst, MOTION_BIG_TURN_LEFT_90);
      } else {
        path_add_motion(dst, MOTION_BIG_TURN_RIGHT_90);
      }
      src->steps[i + 1].half_count = src->steps[i + 1].half_count - 1;
      i = i + 1;
      continue;
    }

    if (is_small_turn(motion) && has_straight_before
        && i + 1 < src->count && src->steps[i + 1].motion == motion
        && is_straight_at(src, i + 2)) {
      /* 直進 → ターン → 同じ向きのターン → 直進：大回り 180° */
      shorten_last_straight(dst);
      if (motion == MOTION_TURN_LEFT) {
        path_add_motion(dst, MOTION_BIG_TURN_LEFT_180);
      } else {
        path_add_motion(dst, MOTION_BIG_TURN_RIGHT_180);
      }
      src->steps[i + 2].half_count = src->steps[i + 2].half_count - 1;
      i = i + 2;  /* ターン 2 つ分を使った */
      continue;
    }

    /* 置き換えられないものはそのまま写す */
    path_add_motion(dst, motion);
    i = i + 1;
  }
}

/* ===========================================================================
 * 既知区間の経路
 * ======================================================================== */

/* 既知区間の経路を作る（KOGUMA の Known_Pass_Generation）。
 *
 * 区画 (x, y) に向き dir で入るところから、探索と同じ決め方（歩数マップ）で進む先を決めていき、
 * 「まだ壁を見ていない区画」に入る手前で止める。壁を見たことのある区画では新しい情報が
 * 増えないので、1 区画ずつ止まって考えなくても、探索とまったく同じ道を通る。
 * それをまとめて 1 つの経路にして、速く走れるようにする（大回りにも変換する）。
 *
 * 先に maze_map_record_walls() で (x, y) の壁を記録し、search_update_step_map() で
 * 歩数マップを作っておくこと。
 * path->end_x, end_y, end_dir には、走り終わったときの位置（まだ見ていない区画の入口）が入る。
 * path->count が 0 なら、まとめて走れる区間はない。 */
void path_make_known(int x, int y, direction_t dir, path_t *path)
{
  static path_t s_raw;  /* 大回りにする前の経路（大きいので static に置く） */
  path_clear(&s_raw);

  while (s_raw.count < PATH_STEP_MAX) {
    if (!maze_map_is_visited(x, y)) {
      break;  /* まだ壁を見ていない区画。ここからは普通の探索で 1 区画ずつ進む */
    }
    if (search_is_in_target(x, y)) {
      break;  /* 目的地に着いた */
    }

    action_t action = search_decide_action(x, y, dir);
    if (action == ACTION_STOP) {
      break;
    } else if (action == ACTION_FORWARD) {
      path_add_straight(&s_raw, 2);  /* 1 区画 = 2 半区画 */
    } else if (action == ACTION_TURN_LEFT) {
      path_add_motion(&s_raw, MOTION_TURN_LEFT);
      dir = maze_turn_left(dir);
    } else if (action == ACTION_TURN_RIGHT) {
      path_add_motion(&s_raw, MOTION_TURN_RIGHT);
      dir = maze_turn_right(dir);
    } else {
      path_add_motion(&s_raw, MOTION_UTURN);
      dir = maze_turn_back(dir);
    }

    x = maze_next_x(x, dir);
    y = maze_next_y(y, dir);
  }

  convert_to_big_turns(&s_raw, path);
  path->end_x = x;
  path->end_y = y;
  path->end_dir = dir;
}
