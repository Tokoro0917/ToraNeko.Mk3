/*
 * test_maze.c
 * 迷路（App/maze/）の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * テストの内容:
 *   1. 壁の地図（maze_map）の基本動作
 *   2. 歩数マップ（search）の基本動作
 *   3. 既知区間の経路（path_make_known）：小さな迷路で、大回りへの変換が正しいか
 *   4. 探索のシミュレーション：テスト用の迷路（mazes フォルダの .txt）を、壁を知らない状態から
 *      足立法で スタート → ゴール → スタート と走り、ちゃんと帰ってこられるかを調べる。
 *      既知区間の経路を使って走っても、1 区画ずつ走ったときと同じところに着くかも調べる
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "maze/maze_map.h"
#include "maze/path.h"
#include "maze/search.h"

/* ---------------------------------------------------------------------------
 * テスト結果の数え方
 * ------------------------------------------------------------------------- */

static int s_fail_count = 0;

/* ok が false なら失敗として数え、message を表示する */
static void check(bool ok, const char *message)
{
  if (!ok) {
    s_fail_count = s_fail_count + 1;
    printf("  失敗: %s\n", message);
  }
}

/* ---------------------------------------------------------------------------
 * 本当の迷路（テストの答え）
 * mazes フォルダの .txt を読み込んで、どこに壁があるかを覚えておく
 * ------------------------------------------------------------------------- */

static bool s_true_wall[MAZE_SIZE][MAZE_SIZE][4];  /* [x][y][向き] 壁があれば true */
static int s_goal_x;       /* ゴールの左下の x */
static int s_goal_y;       /* ゴールの左下の y */
static int s_goal_width;   /* ゴールの幅 [区画] */
static int s_goal_height;  /* ゴールの高さ [区画] */

/* テキストの迷路ファイルを読み込む。読めたら true */
static bool load_maze_file(const char *path)
{
  /* ファイルの 1 行は 「o---」が MAZE_SIZE 個 + 「o」= 4 * MAZE_SIZE + 1 文字 */
  static char lines[2 * MAZE_SIZE + 1][4 * MAZE_SIZE + 8];

  FILE *file = fopen(path, "r");
  if (file == NULL) {
    printf("  ファイルが開けない: %s\n", path);
    return false;
  }
  for (int row = 0; row < 2 * MAZE_SIZE + 1; row++) {
    if (fgets(lines[row], sizeof(lines[row]), file) == NULL) {
      printf("  ファイルの行が足りない: %s\n", path);
      fclose(file);
      return false;
    }
  }
  fclose(file);

  int goal_min_x = MAZE_SIZE;
  int goal_min_y = MAZE_SIZE;
  int goal_max_x = -1;
  int goal_max_y = -1;

  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      /* ファイルの上の行ほど y が大きい。区画 (x, y) の中身がある行と列 */
      int cell_row = 2 * (MAZE_SIZE - 1 - y) + 1;
      int cell_col = 4 * x + 2;

      s_true_wall[x][y][DIR_NORTH] = (lines[cell_row - 1][cell_col] == '-');
      s_true_wall[x][y][DIR_SOUTH] = (lines[cell_row + 1][cell_col] == '-');
      s_true_wall[x][y][DIR_WEST] = (lines[cell_row][cell_col - 2] == '|');
      s_true_wall[x][y][DIR_EAST] = (lines[cell_row][cell_col + 2] == '|');

      if (lines[cell_row][cell_col] == 'G') {
        if (x < goal_min_x) { goal_min_x = x; }
        if (y < goal_min_y) { goal_min_y = y; }
        if (x > goal_max_x) { goal_max_x = x; }
        if (y > goal_max_y) { goal_max_y = y; }
      }
    }
  }

  if (goal_max_x < 0) {
    printf("  ゴール（G）が見つからない: %s\n", path);
    return false;
  }
  s_goal_x = goal_min_x;
  s_goal_y = goal_min_y;
  s_goal_width = goal_max_x - goal_min_x + 1;
  s_goal_height = goal_max_y - goal_min_y + 1;
  return true;
}

/* ---------------------------------------------------------------------------
 * 1. 壁の地図のテスト
 * ------------------------------------------------------------------------- */

static void test_maze_map(void)
{
  printf("[1] 壁の地図\n");
  maze_map_init();

  check(maze_map_get_wall(0, 0, DIR_EAST) == WALL_EXIST, "スタートの東は壁");
  check(maze_map_get_wall(0, 0, DIR_NORTH) == WALL_NONE, "スタートの北は開いている");
  check(maze_map_get_wall(0, 0, DIR_SOUTH) == WALL_EXIST, "スタートの南（外周）は壁");
  check(maze_map_get_wall(5, MAZE_SIZE - 1, DIR_NORTH) == WALL_EXIST, "北の外周は壁");
  check(maze_map_get_wall(5, 5, DIR_NORTH) == WALL_UNKNOWN, "中の壁は最初は「まだ見ていない」");
  check(maze_map_is_visited(0, 0), "スタートは見たことにする");
  check(!maze_map_is_visited(5, 5), "ほかの区画はまだ見ていない");

  /* 片側から書くと、隣の区画から見ても同じになる */
  maze_map_set_wall(5, 5, DIR_NORTH, WALL_EXIST);
  check(maze_map_get_wall(5, 6, DIR_SOUTH) == WALL_EXIST, "(5,5) の北 = (5,6) の南");
  maze_map_set_wall(5, 5, DIR_WEST, WALL_NONE);
  check(maze_map_get_wall(4, 5, DIR_EAST) == WALL_NONE, "(5,5) の西 = (4,5) の東");

  /* 外周の壁は消せない */
  maze_map_set_wall(0, 0, DIR_WEST, WALL_NONE);
  check(maze_map_get_wall(0, 0, DIR_WEST) == WALL_EXIST, "外周の壁は消えない");

  /* 向きを変える */
  check(maze_turn_left(DIR_NORTH) == DIR_WEST, "北の左は西");
  check(maze_turn_right(DIR_WEST) == DIR_NORTH, "西の右は北");
  check(maze_turn_back(DIR_EAST) == DIR_WEST, "東の後ろは西");
}

/* ---------------------------------------------------------------------------
 * 2. 歩数マップのテスト
 * ------------------------------------------------------------------------- */

static void test_step_map(void)
{
  printf("[2] 歩数マップ\n");
  maze_map_init();

  /* 壁を何も見ていない迷路で、目的地 (3, 2) までの歩数は、縦横の距離の合計になる。
   * ただしスタートの東は壁なので、(0,0) からは北に回る必要がある（それでも距離は同じ） */
  search_set_target(3, 2, 1, 1);
  search_update_step_map();
  check(search_get_step(3, 2) == 0, "目的地は 0");
  check(search_get_step(0, 0) == 5, "(0,0) から (3,2) は 5 区画");
  check(search_get_step(3, 5) == 3, "(3,5) から (3,2) は 3 区画");

  /* 目的地の真下の道に壁を置くと、回り道の分だけ増える */
  maze_map_set_wall(3, 2, DIR_SOUTH, WALL_EXIST);
  search_update_step_map();
  check(search_get_step(3, 1) == 3, "(3,1) は真上に壁があるので、横から回って 3 区画");

  /* 歩数が小さい方へ進む。同じなら 前 → 左 → 右 の順 */
  check(search_decide_action(3, 0, DIR_NORTH) == ACTION_FORWARD, "前が近ければ直進");
  check(search_decide_action(0, 2, DIR_NORTH) == ACTION_TURN_RIGHT, "右が近ければ右");

  /* 囲まれていてたどり着けないときは止まる */
  maze_map_init();
  maze_map_set_wall(0, 0, DIR_NORTH, WALL_EXIST);
  search_set_target(10, 10, 1, 1);
  search_update_step_map();
  check(search_decide_action(0, 0, DIR_NORTH) == ACTION_STOP, "閉じ込められたら ACTION_STOP");
}

/* ---------------------------------------------------------------------------
 * 経路を読みやすく表示する（テストの結果を人が見るため）
 * ------------------------------------------------------------------------- */

static void print_path(const path_t *path)
{
  printf("  経路:");
  for (int i = 0; i < path->count; i++) {
    switch (path->steps[i].motion) {
    case MOTION_STRAIGHT:           printf(" 直進%d", path->steps[i].half_count); break;
    case MOTION_TURN_LEFT:          printf(" 左"); break;
    case MOTION_TURN_RIGHT:         printf(" 右"); break;
    case MOTION_UTURN:              printf(" Uターン"); break;
    case MOTION_BIG_TURN_LEFT_90:   printf(" 大回り左90"); break;
    case MOTION_BIG_TURN_RIGHT_90:  printf(" 大回り右90"); break;
    case MOTION_BIG_TURN_LEFT_180:  printf(" 大回り左180"); break;
    case MOTION_BIG_TURN_RIGHT_180: printf(" 大回り右180"); break;
    }
  }
  printf("  （直進の数字は半区画）\n");
}

/* 経路の i 番目が、motion で長さ half_count の動きか */
static bool step_is(const path_t *path, int i, motion_t motion, int half_count)
{
  if (i >= path->count) {
    return false;
  }
  if (path->steps[i].motion != motion) {
    return false;
  }
  if (path->steps[i].half_count != half_count) {
    return false;
  }
  return true;
}

/* ---------------------------------------------------------------------------
 * 3. 既知区間の経路のテスト
 * ------------------------------------------------------------------------- */

static void test_known_path(void)
{
  printf("[3] 既知区間の経路\n");
  path_t path;

  /* 例 1：北に 2 区画進んで右に曲がり、東に 2 区画で目的地
   *   (0,2)→(1,2)→(2,2)→(3,2)=目的地
   *     ↑
   *   (0,1)
   *     ↑
   *   (0,0)=スタート
   * 1 区画ずつだと「直進4 → 右 → 直進4」。大回りにすると「直進3 → 大回り右90 → 直進3」 */
  maze_map_init();
  maze_map_record_walls(0, 0, DIR_NORTH, false, true, true);
  maze_map_record_walls(0, 1, DIR_NORTH, false, true, true);
  maze_map_record_walls(0, 2, DIR_NORTH, true, true, false);
  maze_map_record_walls(1, 2, DIR_EAST, false, true, true);
  maze_map_record_walls(2, 2, DIR_EAST, false, true, true);
  maze_map_record_walls(3, 2, DIR_EAST, true, true, true);
  search_set_target(3, 2, 1, 1);
  search_update_step_map();
  path_make_known(0, 0, DIR_NORTH, &path);
  print_path(&path);
  check(path.count == 3, "例1：動きは 3 つ");
  check(step_is(&path, 0, MOTION_STRAIGHT, 3), "例1：最初は直進 3 半区画");
  check(step_is(&path, 1, MOTION_BIG_TURN_RIGHT_90, 0), "例1：次は大回り右 90°");
  check(step_is(&path, 2, MOTION_STRAIGHT, 3), "例1：最後は直進 3 半区画");
  check(path.end_x == 3 && path.end_y == 2 && path.end_dir == DIR_EAST, "例1：(3,2) で東向きに終わる");

  /* 例 2：北に 2 区画、右、右で折り返して南に 1 区画で目的地
   *   (0,2)→(1,2)
   *     ↑     ↓
   *   (0,1) (1,1)
   *     ↑     ↓
   *   (0,0) (1,0)=目的地
   * 1 区画ずつだと「直進4 → 右 → 右 → 直進2」。大回りにすると「直進3 → 大回り右180 → 直進1」 */
  maze_map_init();
  maze_map_record_walls(0, 0, DIR_NORTH, false, true, true);
  maze_map_record_walls(0, 1, DIR_NORTH, false, true, true);
  maze_map_record_walls(0, 2, DIR_NORTH, true, true, false);
  maze_map_record_walls(1, 2, DIR_EAST, true, true, false);
  maze_map_record_walls(1, 1, DIR_SOUTH, false, true, true);
  maze_map_record_walls(1, 0, DIR_SOUTH, true, true, true);
  search_set_target(1, 0, 1, 1);
  search_update_step_map();
  path_make_known(0, 0, DIR_NORTH, &path);
  print_path(&path);
  check(path.count == 3, "例2：動きは 3 つ");
  check(step_is(&path, 0, MOTION_STRAIGHT, 3), "例2：最初は直進 3 半区画");
  check(step_is(&path, 1, MOTION_BIG_TURN_RIGHT_180, 0), "例2：次は大回り右 180°");
  check(step_is(&path, 2, MOTION_STRAIGHT, 1), "例2：最後は直進 1 半区画");
  check(path.end_x == 1 && path.end_y == 0 && path.end_dir == DIR_SOUTH, "例2：(1,0) で南向きに終わる");

  /* 例 3：いきなり曲がるときは、前に直進がないので大回りにしない
   * (0,1) で左に曲がろうとしても西は外周なので、(0,1) で右に曲がる。前の直進は (0,0) の 2 半区画 */
  maze_map_init();
  maze_map_record_walls(0, 0, DIR_NORTH, false, true, true);
  search_set_target(0, 0, 1, 1);  /* 目的地が今いる区画なら、経路は空 */
  search_update_step_map();
  path_make_known(0, 0, DIR_NORTH, &path);
  check(path.count == 0, "例3：目的地にいるなら経路は空");

  /* 例 4：まだ見ていない区画に入るところで止まる */
  maze_map_init();
  maze_map_record_walls(0, 0, DIR_NORTH, false, true, true);
  search_set_target(0, 5, 1, 1);
  search_update_step_map();
  path_make_known(0, 0, DIR_NORTH, &path);
  check(path.count == 1 && step_is(&path, 0, MOTION_STRAIGHT, 2), "例4：見た区画 (0,0) だけ直進する");
  check(path.end_x == 0 && path.end_y == 1, "例4：まだ見ていない (0,1) の入口で止まる");
}

/* ---------------------------------------------------------------------------
 * 4. 探索のシミュレーション
 * ------------------------------------------------------------------------- */

#define MOVE_LIMIT 5000  /* これだけ動いても着かなければ失敗とする */

/* 経路を、1 区画ずつの動き（直進・左・右・Uターン）に戻しながら本当の迷路の上で走らせ、
 * 壁を通り抜けずに経路の終わりの位置まで行けるか調べる。行ければ true。
 *
 * 大回りは「半区画の直進 + ターン + 半区画の直進」に戻せる。直進の半区画を貯めておき、
 * ターンの手前で 2 半区画ずつ 1 区画の直進にする（半区画が余ったら経路がおかしい） */
static int s_pending_half = 0;     /* まだ走らせていない直進 [半区画] */
static int s_replayed_cells = 0;  /* 経路を走らせて進んだ区画数（シミュレーションの集計用） */

/* 1 区画ぶん走らせる。壁にぶつかったら false */
static bool replay_one_cell(int *x, int *y, direction_t *dir, action_t action)
{
  if (action == ACTION_TURN_LEFT) {
    *dir = maze_turn_left(*dir);
  } else if (action == ACTION_TURN_RIGHT) {
    *dir = maze_turn_right(*dir);
  } else if (action == ACTION_UTURN) {
    *dir = maze_turn_back(*dir);
  }
  if (s_true_wall[*x][*y][*dir]) {
    return false;
  }
  *x = maze_next_x(*x, *dir);
  *y = maze_next_y(*y, *dir);
  s_replayed_cells = s_replayed_cells + 1;
  return true;
}

/* 貯めていた直進を走らせる。半区画が余ったり壁にぶつかったら false */
static bool replay_pending_straight(int *x, int *y, direction_t *dir)
{
  if (s_pending_half % 2 != 0) {
    return false;  /* 奇数（2 で割り切れない）なら、半区画が余っている */
  }
  while (s_pending_half > 0) {
    if (!replay_one_cell(x, y, dir, ACTION_FORWARD)) {
      return false;
    }
    s_pending_half = s_pending_half - 2;
  }
  return true;
}

/* ターンを 1 つ走らせる（その前に貯めていた直進を走らせる） */
static bool replay_turn(int *x, int *y, direction_t *dir, action_t action)
{
  if (!replay_pending_straight(x, y, dir)) {
    return false;
  }
  return replay_one_cell(x, y, dir, action);
}

static bool replay_path(const path_t *path, int x, int y, direction_t dir)
{
  bool ok = true;
  s_pending_half = 0;

  for (int i = 0; i < path->count && ok; i++) {
    switch (path->steps[i].motion) {
    case MOTION_STRAIGHT:
      s_pending_half = s_pending_half + path->steps[i].half_count;
      break;
    case MOTION_TURN_LEFT:
      ok = replay_turn(&x, &y, &dir, ACTION_TURN_LEFT);
      break;
    case MOTION_TURN_RIGHT:
      ok = replay_turn(&x, &y, &dir, ACTION_TURN_RIGHT);
      break;
    case MOTION_UTURN:
      ok = replay_turn(&x, &y, &dir, ACTION_UTURN);
      break;
    case MOTION_BIG_TURN_LEFT_90:
      s_pending_half = s_pending_half + 1;
      ok = replay_turn(&x, &y, &dir, ACTION_TURN_LEFT);
      s_pending_half = 1;
      break;
    case MOTION_BIG_TURN_RIGHT_90:
      s_pending_half = s_pending_half + 1;
      ok = replay_turn(&x, &y, &dir, ACTION_TURN_RIGHT);
      s_pending_half = 1;
      break;
    case MOTION_BIG_TURN_LEFT_180:
      s_pending_half = s_pending_half + 1;
      ok = replay_turn(&x, &y, &dir, ACTION_TURN_LEFT);
      if (ok) {
        ok = replay_turn(&x, &y, &dir, ACTION_TURN_LEFT);
      }
      s_pending_half = 1;
      break;
    case MOTION_BIG_TURN_RIGHT_180:
      s_pending_half = s_pending_half + 1;
      ok = replay_turn(&x, &y, &dir, ACTION_TURN_RIGHT);
      if (ok) {
        ok = replay_turn(&x, &y, &dir, ACTION_TURN_RIGHT);
      }
      s_pending_half = 1;
      break;
    }
  }
  if (ok) {
    ok = replay_pending_straight(&x, &y, &dir);
  }
  if (!ok) {
    return false;
  }
  if (x != path->end_x || y != path->end_y || dir != path->end_dir) {
    return false;
  }
  return true;
}

/* シミュレーションで数えるもの */
static int s_known_path_count = 0;       /* 既知区間の経路でまとめて走った回数 */
static int s_big_turn_count = 0;         /* そのうち大回りの数 */
static int s_known_path_error_count = 0; /* 経路を走らせたら壁にぶつかった、または位置が合わなかった回数 */

/* 経路の中の大回りを数える */
static int count_big_turns(const path_t *path)
{
  int count = 0;
  for (int i = 0; i < path->count; i++) {
    motion_t motion = path->steps[i].motion;
    if (motion == MOTION_BIG_TURN_LEFT_90 || motion == MOTION_BIG_TURN_RIGHT_90
        || motion == MOTION_BIG_TURN_LEFT_180 || motion == MOTION_BIG_TURN_RIGHT_180) {
      count = count + 1;
    }
  }
  return count;
}

/* 今いる (x, y, dir) から、目的地に着くまで足立法で走る。
 * use_known_path が true なら、見たことのある区画が続くところは既知区間の経路でまとめて走る。
 * 進んだ区画数を返す（既知区間の経路でまとめて走った分は数えない）。着けなかったら -1。
 * 走り終わった位置と向きは x, y, dir に入る */
static int run_to_target(int *x, int *y, direction_t *dir, bool use_known_path)
{
  static path_t s_path;
  int move_count = 0;

  while (move_count < MOVE_LIMIT) {
    /* センサで前・左・右の壁を見る（本当の迷路から読む） */
    bool wall_front = s_true_wall[*x][*y][*dir];
    bool wall_left = s_true_wall[*x][*y][maze_turn_left(*dir)];
    bool wall_right = s_true_wall[*x][*y][maze_turn_right(*dir)];
    maze_map_record_walls(*x, *y, *dir, wall_front, wall_left, wall_right);

    if (search_is_in_target(*x, *y)) {
      return move_count;
    }

    search_update_step_map();

    if (use_known_path) {
      path_make_known(*x, *y, *dir, &s_path);
      /* 2 区画以上まとめて走れるときだけ使う（1 区画なら普通の探索と同じ） */
      bool is_long = true;
      if (s_path.count == 0) {
        is_long = false;
      }
      if (s_path.count == 1 && s_path.steps[0].motion == MOTION_STRAIGHT && s_path.steps[0].half_count <= 2) {
        is_long = false;
      }
      if (s_path.count == 1 && s_path.steps[0].motion != MOTION_STRAIGHT) {
        is_long = false;
      }
      if (is_long) {
        if (!replay_path(&s_path, *x, *y, *dir)) {
          s_known_path_error_count = s_known_path_error_count + 1;
        }
        s_known_path_count = s_known_path_count + 1;
        s_big_turn_count = s_big_turn_count + count_big_turns(&s_path);
        *x = s_path.end_x;
        *y = s_path.end_y;
        *dir = s_path.end_dir;
        continue;
      }
    }

    action_t action = search_decide_action(*x, *y, *dir);

    if (action == ACTION_STOP) {
      printf("  (%d,%d) で進めなくなった\n", *x, *y);
      return -1;
    } else if (action == ACTION_TURN_LEFT) {
      *dir = maze_turn_left(*dir);
    } else if (action == ACTION_TURN_RIGHT) {
      *dir = maze_turn_right(*dir);
    } else if (action == ACTION_UTURN) {
      *dir = maze_turn_back(*dir);
    }

    /* 進む先に本当に壁があったら、探索が壁を無視したことになる */
    if (s_true_wall[*x][*y][*dir]) {
      printf("  (%d,%d) で壁に突っ込んだ\n", *x, *y);
      return -1;
    }
    *x = maze_next_x(*x, *dir);
    *y = maze_next_y(*y, *dir);
    move_count = move_count + 1;
  }

  printf("  %d 区画動いても着かなかった\n", MOVE_LIMIT);
  return -1;
}

/* 記録した壁が、本当の迷路と食い違っていないか調べる。食い違いの数を返す */
static int count_wrong_walls(void)
{
  int wrong_count = 0;
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int i = 0; i < 4; i++) {
        direction_t dir = (direction_t)i;
        wall_state_t recorded = maze_map_get_wall(x, y, dir);
        if (recorded == WALL_EXIST && !s_true_wall[x][y][dir]) {
          wrong_count = wrong_count + 1;
        }
        if (recorded == WALL_NONE && s_true_wall[x][y][dir]) {
          wrong_count = wrong_count + 1;
        }
      }
    }
  }
  return wrong_count;
}

/* スタート → ゴール → スタート を走る。進んだ区画数を moves_to_goal, moves_to_start に入れる */
static void explore_round_trip(bool use_known_path, int *moves_to_goal, int *moves_to_start)
{
  maze_map_init();
  int x = 0;
  int y = 0;
  direction_t dir = DIR_NORTH;

  /* 行き：スタート → ゴール */
  search_set_target(s_goal_x, s_goal_y, s_goal_width, s_goal_height);
  *moves_to_goal = run_to_target(&x, &y, &dir, use_known_path);
  check(*moves_to_goal >= 0, "ゴールに着く");

  /* 帰り：ゴール → スタート */
  search_set_target(0, 0, 1, 1);
  *moves_to_start = run_to_target(&x, &y, &dir, use_known_path);
  check(*moves_to_start >= 0, "スタートに戻る");
  check(x == 0 && y == 0, "最後はスタートにいる");

  check(count_wrong_walls() == 0, "記録した壁が本当の迷路と同じ");
}

static void test_explore(const char *file_path)
{
  printf("[4] 探索のシミュレーション: %s\n", file_path);
  if (!load_maze_file(file_path)) {
    check(false, "迷路ファイルが読めない");
    return;
  }
  printf("  ゴール: (%d,%d) から %dx%d 区画\n", s_goal_x, s_goal_y, s_goal_width, s_goal_height);

  /* 1 区画ずつ探索する */
  int goal_moves = 0;
  int start_moves = 0;
  explore_round_trip(false, &goal_moves, &start_moves);
  printf("  1 区画ずつ:       行き %d 区画、帰り %d 区画\n", goal_moves, start_moves);

  /* 既知区間の経路を使って探索する */
  s_known_path_count = 0;
  s_big_turn_count = 0;
  s_known_path_error_count = 0;
  s_replayed_cells = 0;
  int goal_moves_known = 0;
  int start_moves_known = 0;
  explore_round_trip(true, &goal_moves_known, &start_moves_known);
  printf("  既知区間を使う:   行き %d 区画、帰り %d 区画（このほかに既知区間を %d 回まとめて走った。大回り %d 回）\n",
         goal_moves_known, start_moves_known, s_known_path_count, s_big_turn_count);

  check(s_known_path_error_count == 0, "既知区間の経路が壁を通らず、経路の終わりの位置に着く");
  check(s_known_path_count > 0, "既知区間の経路が使われている");

  /* 見たことのある区画では新しい情報が増えないので、まとめて走っても 1 区画ずつ走っても
   * 通る道は同じはず。進んだ区画数の合計が同じかで確かめる */
  int total_one_by_one = goal_moves + start_moves;
  int total_known = goal_moves_known + start_moves_known + s_replayed_cells;
  printf("  進んだ区画数の合計: 1 区画ずつ %d、既知区間を使う %d\n", total_one_by_one, total_known);
  check(total_one_by_one == total_known, "既知区間を使っても、1 区画ずつと同じ道を通る");
}

/* ---------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(void)
{
  test_maze_map();
  test_step_map();
  test_known_path();
  test_explore("mazes/maze32_center2x2.txt");
  test_explore("mazes/maze32_offset3x3.txt");

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗: %d 件\n", s_fail_count);
  return 1;
}
