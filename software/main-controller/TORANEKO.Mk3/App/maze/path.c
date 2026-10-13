/*
 * path.c
 * 経路（走る動きの並び）を作る
 */
#include "maze/path.h"

#include "maze/dijkstra.h"
#include "maze/search.h"

/* 経路を作る途中で使う作業用の経路（大きいので、関数ごとに持たずに 2 つを使い回す） */
static path_t s_work_a;
static path_t s_work_b;

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
    path->steps[last].half_count = (int16_t)(path->steps[last].half_count + half_count);
    return;
  }
  if (path->count >= PATH_STEP_MAX) {
    return;  /* 入りきらない（迷路の区画数より長くはならないので、普通は起きない） */
  }
  path->steps[path->count].motion = MOTION_STRAIGHT;
  path->steps[path->count].half_count = (int16_t)half_count;
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
  path->steps[last].half_count = (int16_t)(path->steps[last].half_count - 1);
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
      src->steps[i + 1].half_count = (int16_t)(src->steps[i + 1].half_count - 1);
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
      src->steps[i + 2].half_count = (int16_t)(src->steps[i + 2].half_count - 1);
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
  path_t *raw = &s_work_a;  /* 大回りにする前の経路 */
  path_clear(raw);

  while (raw->count < PATH_STEP_MAX) {
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
      path_add_straight(raw, 2);  /* 1 区画 = 2 半区画 */
    } else if (action == ACTION_TURN_LEFT) {
      path_add_motion(raw, MOTION_TURN_LEFT);
      dir = maze_turn_left(dir);
    } else if (action == ACTION_TURN_RIGHT) {
      path_add_motion(raw, MOTION_TURN_RIGHT);
      dir = maze_turn_right(dir);
    } else {
      path_add_motion(raw, MOTION_UTURN);
      dir = maze_turn_back(dir);
    }

    x = maze_next_x(x, dir);
    y = maze_next_y(y, dir);
  }

  convert_to_big_turns(raw, path);
  path->end_x = x;
  path->end_y = y;
  path->end_dir = dir;
}

/* ===========================================================================
 * 斜めへの変換（KOGUMA の Shortest_Pass_Compression_NANAME）
 * ======================================================================== */

/* 経路の最後に、斜めの直進を count 足す。最後の動きが斜めの直進なら、その長さに足し込む */
static void path_add_diag_straight(path_t *path, int count)
{
  int last = path->count - 1;
  if (last >= 0 && path->steps[last].motion == MOTION_DIAG_STRAIGHT) {
    path->steps[last].half_count = (int16_t)(path->steps[last].half_count + count);
    return;
  }
  if (path->count >= PATH_STEP_MAX) {
    return;
  }
  path->steps[path->count].motion = MOTION_DIAG_STRAIGHT;
  path->steps[path->count].half_count = (int16_t)count;
  path->count = path->count + 1;
}

/* 経路の i 番目の直進を半区画短くする（直進でなければ何もしない） */
static void shorten_straight_at(path_t *path, int i)
{
  if (is_straight_at(path, i)) {
    path->steps[i].half_count = (int16_t)(path->steps[i].half_count - 1);
  }
}

/* 経路の i 番目が「探索用のターン以外」か（直進、大回り、または経路の終わりの先） */
static bool is_not_small_turn_at(const path_t *path, int i)
{
  if (i >= path->count) {
    return true;  /* 経路の終わり */
  }
  if (is_small_turn(path->steps[i].motion)) {
    return false;
  }
  return true;
}

/* 左ターンなら left を、右ターンなら right を返す */
static motion_t pick_side(motion_t turn, motion_t left, motion_t right)
{
  if (turn == MOTION_TURN_LEFT) {
    return left;
  }
  return right;
}

/* 大回りにならずに残った探索用のターンのうち、ジグザグに続くところを斜めの動きに置き換える（src → dst）。
 *
 * 斜めに入る（直進のあとにターンが続くところ）
 *   直進 → 左 → 右 …   は  直進(半区画短く) → 左45°で斜めに入る → …
 *   直進 → 左 → 左 …   は  直進(半区画短く) → 左135°で斜めに入る → …
 * 斜めの中（左右が交互に続くところ）
 *   … 左 → 右 …       は  1 つ目のターンが斜めの直進 1 つ分
 *   … 左 → 左 …（あとにまだターンが続く）は  V90 左
 * 斜めから出る
 *   … 左 → 直進        は  左45°で直進に出る → 直進(半区画短く)
 *   … 左 → 左 → 直進   は  左135°で直進に出る → 直進(半区画短く)
 * （右も同じ）
 * 後ろの直進を半区画短くするために、src の中身も書き換える。 */
static void convert_to_diagonal(path_t *src, path_t *dst)
{
  path_clear(dst);
  bool in_diagonal = false;  /* 今、斜めを走っているか */

  int i = 0;
  while (i < src->count) {
    motion_t motion = src->steps[i].motion;

    /* 探索用のターン以外は、そのまま写す */
    if (!is_small_turn(motion)) {
      if (motion == MOTION_STRAIGHT) {
        path_add_straight(dst, src->steps[i].half_count);
      } else {
        path_add_motion(dst, motion);
      }
      i = i + 1;
      continue;
    }

    /* 次の動きが「同じ向きのターン」か「反対向きのターン」か */
    bool next_is_same_turn = false;
    bool next_is_opposite_turn = false;
    if (i + 1 < src->count && is_small_turn(src->steps[i + 1].motion)) {
      if (src->steps[i + 1].motion == motion) {
        next_is_same_turn = true;
      } else {
        next_is_opposite_turn = true;
      }
    }

    if (!in_diagonal) {
      /* 斜めに入れるのは、前に直進があって、すぐ次もターンのとき */
      bool has_straight_before = is_straight_at(dst, dst->count - 1);
      if (has_straight_before && next_is_opposite_turn) {
        shorten_last_straight(dst);
        path_add_motion(dst, pick_side(motion, MOTION_DIAG_IN45_LEFT, MOTION_DIAG_IN45_RIGHT));
        in_diagonal = true;
        i = i + 1;  /* 次の反対向きのターンは、斜めの中の動きとして次に調べる */
        continue;
      }
      if (has_straight_before && next_is_same_turn) {
        shorten_last_straight(dst);
        path_add_motion(dst, pick_side(motion, MOTION_DIAG_IN135_LEFT, MOTION_DIAG_IN135_RIGHT));
        in_diagonal = true;
        i = i + 2;  /* ターン 2 つ分を使った */
        continue;
      }
      /* 斜めにできないターンは、探索用のターンのまま残す */
      path_add_motion(dst, motion);
      i = i + 1;
      continue;
    }

    /* ここからは斜めの中 */
    if (is_not_small_turn_at(src, i + 1)) {
      /* 次が直進（または経路の終わり）：45° で直進に出る */
      path_add_motion(dst, pick_side(motion, MOTION_DIAG_OUT45_LEFT, MOTION_DIAG_OUT45_RIGHT));
      shorten_straight_at(src, i + 1);
      in_diagonal = false;
      i = i + 1;
      continue;
    }
    if (next_is_same_turn && is_not_small_turn_at(src, i + 2)) {
      /* 同じ向きのターンが 2 つ続いて、その次が直進：135° で直進に出る */
      path_add_motion(dst, pick_side(motion, MOTION_DIAG_OUT135_LEFT, MOTION_DIAG_OUT135_RIGHT));
      shorten_straight_at(src, i + 2);
      in_diagonal = false;
      i = i + 2;
      continue;
    }
    if (next_is_same_turn) {
      /* 同じ向きのターンが 2 つ続いて、その次もターン：V90 */
      path_add_motion(dst, pick_side(motion, MOTION_DIAG_V90_LEFT, MOTION_DIAG_V90_RIGHT));
      i = i + 2;
      continue;
    }
    /* 左右が交互に続く：斜めの直進 1 つ分 */
    path_add_diag_straight(dst, 1);
    i = i + 1;
  }
}

/* ===========================================================================
 * 最短経路
 * ======================================================================== */

/* 区画 (from_x, from_y) から隣の区画 (to_x, to_y) へ向かう向き */
static direction_t direction_between(int from_x, int from_y, int to_x, int to_y)
{
  if (to_y > from_y) {
    return DIR_NORTH;
  }
  if (to_y < from_y) {
    return DIR_SOUTH;
  }
  if (to_x > from_x) {
    return DIR_EAST;
  }
  return DIR_WEST;
}

/* 最短経路を作る（KOGUMA の Maze_Dijkstra_Calculation → Shortest_Pass_Compression → _NANAME）。
 *
 *   1. ダイクストラで、スタートからゴールまでの通る区画を探す（まだ見ていない壁は「ある」とみなす）
 *   2. 区画の並びを、1 区画ずつの動き（直進・左・右）にする
 *   3. 最後に、ゴールの最初の区画の真ん中まで半区画進んで止まる
 *   4. 大回りに置き換える → 斜めに置き換える
 *
 * 経路は、スタートの区画 (0,0) の南の辺（入口）から北向きに始まる。
 * 実際のロボットは区画の後ろの壁にお尻をつけて置くので、そのずれは走る側（ctrl/run.c）で足す。
 * ゴールにたどり着けなければ false。 */
bool path_make_shortest(int goal_x, int goal_y, int goal_width, int goal_height, path_t *path)
{
  path_t *raw = &s_work_a;       /* 1 区画ずつの動き */
  path_t *big_turn = &s_work_b;  /* 大回りに置き換えたもの */

  path_clear(path);
  if (!dijkstra_find_route(goal_x, goal_y, goal_width, goal_height, true)) {
    return false;
  }

  /* 2. 区画の並び → 1 区画ずつの動き */
  path_clear(raw);
  direction_t dir = DIR_NORTH;
  int length = dijkstra_get_route_length();
  for (int i = 0; i + 1 < length; i++) {
    int x = dijkstra_get_route_x(i);
    int y = dijkstra_get_route_y(i);
    int next_x = dijkstra_get_route_x(i + 1);
    int next_y = dijkstra_get_route_y(i + 1);
    direction_t next_dir = direction_between(x, y, next_x, next_y);

    if (next_dir == dir) {
      path_add_straight(raw, 2);
    } else if (next_dir == maze_turn_left(dir)) {
      path_add_motion(raw, MOTION_TURN_LEFT);
    } else if (next_dir == maze_turn_right(dir)) {
      path_add_motion(raw, MOTION_TURN_RIGHT);
    } else {
      return false;  /* 後ろに戻る道（ダイクストラの結果としてはありえない） */
    }
    dir = next_dir;
  }

  /* 3. ゴールの最初の区画の真ん中まで */
  path_add_straight(raw, 1);
  if (raw->count >= PATH_STEP_MAX) {
    return false;  /* 経路が長すぎて入りきらなかった（普通は起きない） */
  }

  /* 4. 大回り → 斜め */
  convert_to_big_turns(raw, big_turn);
  convert_to_diagonal(big_turn, path);

  path->end_x = dijkstra_get_route_x(length - 1);
  path->end_y = dijkstra_get_route_y(length - 1);
  path->end_dir = dir;
  return true;
}
