/*
 * test_maze.c
 * 迷路（App/maze/）の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * テストの内容:
 *   1. 壁の地図（maze_map）の基本動作。フェイルセーフで止まったときに、最後の 5 区画ぶんを戻せるか
 *   2. 歩数マップ（search）の基本動作
 *   3. 既知区間の経路（path_make_known）：小さな迷路で、正しい動きの並びになるか
 *   6. 未知壁探索（KOGUMA のモード0 No.7）：ゴールに着いたあと最短経路の上の見ていない壁を見に行く。
 *      見に行く壁がなくなったらその場で止まって最短経路を計算し、全部の壁を知っているときと
 *      同じ速さの最短経路になるか
 *   7. ランダムな迷路（たくさん）：全部の壁を知っているときの最短経路が、どれも壁を通らずにゴールに着くか
 *   8. ダイクストラの答えが本当の最短か：時間はかかるが確実な方法（総当たりに近い計算）で求めたコストと比べる
 *   5. 最短経路（path_make_shortest）：何もない迷路と、探索したあとの迷路で、
 *      大回り・斜めを使った経路が壁を通らずにゴールに着くか
 *   4. 探索のシミュレーション：テスト用の迷路（mazes フォルダの .txt）を、壁を知らない状態から
 *      足立法で スタート → ゴール → スタート と走り、ちゃんと帰ってこられるかを調べる。
 *      既知区間の経路を使って走っても、1 区画ずつ走ったときと同じところに着くかも調べる
 *   9. 探索走行（sys/explore.c）：本当の迷路の上を動く「仮の機体」で、往復・片道・未知壁探索を走らせる。
 *      止まる場所・Flash に保存する場所・最後の位置・記録した壁が正しいか、未知壁探索のあとの最短経路が
 *      全部の壁を知っているときと同じ速さか、フェイルセーフで止まったときに最後の 5 区画ぶんを戻して保存し、
 *      続きから探索できるかを調べる
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "maze/dijkstra.h"
#include "maze/maze_map.h"
#include "maze/path.h"
#include "maze/search.h"
#include "ctrl/run.h"
#include "ctrl/wall_ctrl.h"
#include "sys/explore.h"
#include "sys/failsafe.h"
#include "sys/flash.h"

/* ダイクストラの中だけで使う関数（move_from など）を、8 のテストで使うため、dijkstra.c をファイルごと取り込む。
 * そのため Makefile では dijkstra.c を別にコンパイルしない */
#include "maze/dijkstra.c"

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
        if (x < goal_min_x) {
          goal_min_x = x;
        }
        if (y < goal_min_y) {
          goal_min_y = y;
        }
        if (x > goal_max_x) {
          goal_max_x = x;
        }
        if (y > goal_max_y) {
          goal_max_y = y;
        }
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

  /* スタート区画の東の壁（規則で必ずある）も消せない */
  maze_map_set_wall(0, 0, DIR_EAST, WALL_NONE);
  check(maze_map_get_wall(0, 0, DIR_EAST) == WALL_EXIST, "スタートの東の壁は消えない");
  check(maze_map_get_wall(1, 0, DIR_WEST) == WALL_EXIST, "(1,0) から見たスタートの東の壁も消えない");

  /* センサで見た壁は足すだけで、1 回「見えなかった」だけでは消えない（KOGUMA と同じ） */
  maze_map_record_walls(3, 3, DIR_NORTH, true, false, false);
  check(maze_map_get_wall(3, 3, DIR_NORTH) == WALL_EXIST, "見えた前の壁は「あり」");
  check(maze_map_get_wall(3, 3, DIR_WEST) == WALL_NONE, "見えなかった左の壁は「なし」");
  maze_map_record_walls(3, 3, DIR_NORTH, false, true, false);
  check(maze_map_get_wall(3, 3, DIR_NORTH) == WALL_EXIST, "一度見えた壁は、あとで見えなくても消えない");
  check(maze_map_get_wall(3, 3, DIR_WEST) == WALL_EXIST, "「なし」だった壁も、あとで見えたら「あり」になる");

  /* 向きを変える */
  check(maze_turn_left(DIR_NORTH) == DIR_WEST, "北の左は西");
  check(maze_turn_right(DIR_WEST) == DIR_NORTH, "西の右は北");
  check(maze_turn_back(DIR_EAST) == DIR_WEST, "東の後ろは西");
}

/* 最後に壁を記録した 5 区画ぶんを戻す（フェイルセーフで止まったとき） */
static void test_undo_recent_cells(void)
{
  printf("[1] 壁の地図：最後の %d 区画ぶんを戻す\n", MAZE_UNDO_CELL_COUNT);

  /* 1. スタートから北へ 7 区画進んだとする。左（西）は外周、右（東）は壁なし、前も壁なし */
  maze_map_init();
  for (int y = 1; y <= 7; y++) {
    maze_map_record_walls(0, y, DIR_NORTH, false, true, false);
  }
  maze_map_undo_recent_cells();
  /* 古い 2 区画 (0,1)・(0,2) は残り、新しい 5 区画 (0,3)〜(0,7) は記録する前に戻る */
  check(maze_map_is_visited(0, 1) && maze_map_is_visited(0, 2), "古い 2 区画は入ったことのあるまま");
  check(maze_map_get_wall(0, 2, DIR_EAST) == WALL_NONE, "古い区画の壁は残る");
  check(maze_map_get_wall(0, 2, DIR_NORTH) == WALL_NONE, "古い区画 (0,2) で見た前の壁（= (0,3) の南）は残る");
  check(!maze_map_is_visited(0, 3) && !maze_map_is_visited(0, 7), "新しい 5 区画は入ったことのない区画に戻る");
  check(maze_map_get_wall(0, 3, DIR_EAST) == WALL_UNKNOWN, "新しい区画の壁は「まだ見ていない」に戻る");
  check(maze_map_get_wall(1, 5, DIR_WEST) == WALL_UNKNOWN, "隣の区画から見た同じ壁も戻る");
  check(maze_map_get_wall(0, 7, DIR_NORTH) == WALL_UNKNOWN, "いちばん新しい区画の前の壁も戻る");
  check(maze_map_get_wall(0, 5, DIR_WEST) == WALL_EXIST, "外周の壁は「あり」のまま");

  /* 2. 続けてもう一度戻しても、何も変わらない（覚えている区画はもうない） */
  maze_map_undo_recent_cells();
  check(maze_map_is_visited(0, 2), "2 回目は何も戻さない");

  /* 3. 同じ区画を 2 回記録した（2 回目で前の壁が見えた）ときは、1 回目の前の状態まで戻る */
  maze_map_init();
  maze_map_record_walls(5, 5, DIR_NORTH, false, false, false);
  maze_map_record_walls(5, 5, DIR_NORTH, true, false, false);
  check(maze_map_get_wall(5, 5, DIR_NORTH) == WALL_EXIST, "2 回目の記録で前の壁が「あり」");
  maze_map_undo_recent_cells();
  check(maze_map_get_wall(5, 5, DIR_NORTH) == WALL_UNKNOWN, "2 回とも戻り、「まだ見ていない」になる");
  check(maze_map_get_wall(5, 6, DIR_SOUTH) == WALL_UNKNOWN, "(5,6) から見た同じ壁も戻る");
  check(!maze_map_is_visited(5, 5), "入ったことのない区画に戻る");

  /* 4. 探索を始めるときに忘れた区画は、戻さない */
  maze_map_init();
  maze_map_record_walls(0, 1, DIR_NORTH, false, true, true);
  maze_map_clear_recent_cells();
  maze_map_record_walls(0, 2, DIR_NORTH, true, true, false);
  maze_map_undo_recent_cells();
  check(maze_map_get_wall(0, 1, DIR_EAST) == WALL_EXIST, "忘れる前に記録した区画は戻さない");
  check(maze_map_is_visited(0, 1), "忘れる前に記録した区画は入ったことのあるまま");
  check(maze_map_get_wall(0, 2, DIR_NORTH) == WALL_UNKNOWN, "忘れたあとに記録した区画は戻す");
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

  /* 未知壁探索の経路を計算していないときは「全部見た」ではなく「経路がない」 */
  int cell_x;
  int cell_y;
  direction_t wall_dir;
  maze_map_init();
  maze_map_set_wall(0, 0, DIR_NORTH, WALL_EXIST);
  check(!search_plan_unknown_walls(10, 10, 1, 1), "閉じ込められていたら未知壁探索の経路は計算できない");
  check(search_check_unknown_walls(&cell_x, &cell_y, &wall_dir) == UNKNOWN_WALL_NO_PLAN,
        "経路が計算できなかったら UNKNOWN_WALL_NO_PLAN（「全部見た」と区別する）");

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
    case MOTION_DIAG_IN45_LEFT:     printf(" 斜め入り左45"); break;
    case MOTION_DIAG_IN45_RIGHT:    printf(" 斜め入り右45"); break;
    case MOTION_DIAG_IN135_LEFT:    printf(" 斜め入り左135"); break;
    case MOTION_DIAG_IN135_RIGHT:   printf(" 斜め入り右135"); break;
    case MOTION_DIAG_STRAIGHT:      printf(" 斜め直進%d", path->steps[i].half_count); break;
    case MOTION_DIAG_V90_LEFT:      printf(" V90左"); break;
    case MOTION_DIAG_V90_RIGHT:     printf(" V90右"); break;
    case MOTION_DIAG_OUT45_LEFT:    printf(" 斜め出左45"); break;
    case MOTION_DIAG_OUT45_RIGHT:   printf(" 斜め出右45"); break;
    case MOTION_DIAG_OUT135_LEFT:   printf(" 斜め出左135"); break;
    case MOTION_DIAG_OUT135_RIGHT:  printf(" 斜め出右135"); break;
    }
  }
  printf("\n");
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
   * 既知区間では大回りにしないので「直進4 → 右 → 直進4」 */
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
  check(step_is(&path, 0, MOTION_STRAIGHT, 4), "例1：最初は直進 4 半区画");
  check(step_is(&path, 1, MOTION_TURN_RIGHT, 0), "例1：次は右（小回り）");
  check(step_is(&path, 2, MOTION_STRAIGHT, 4), "例1：最後は直進 4 半区画");
  check(path.end_x == 3 && path.end_y == 2 && path.end_dir == DIR_EAST, "例1：(3,2) で東向きに終わる");

  /* 例 2：北に 2 区画、右、右で折り返して南に 1 区画で目的地
   *   (0,2)→(1,2)
   *     ↑     ↓
   *   (0,1) (1,1)
   *     ↑     ↓
   *   (0,0) (1,0)=目的地
   * 「直進4 → 右 → 右 → 直進2」 */
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
  check(path.count == 4, "例2：動きは 4 つ");
  check(step_is(&path, 0, MOTION_STRAIGHT, 4), "例2：最初は直進 4 半区画");
  check(step_is(&path, 1, MOTION_TURN_RIGHT, 0), "例2：次は右");
  check(step_is(&path, 2, MOTION_TURN_RIGHT, 0), "例2：もう一度右");
  check(step_is(&path, 3, MOTION_STRAIGHT, 2), "例2：最後は直進 2 半区画");
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

/* 動きが左向きのものか */
static bool is_left_motion(motion_t motion)
{
  switch (motion) {
  case MOTION_TURN_LEFT:
  case MOTION_BIG_TURN_LEFT_90:
  case MOTION_BIG_TURN_LEFT_180:
  case MOTION_DIAG_IN45_LEFT:
  case MOTION_DIAG_IN135_LEFT:
  case MOTION_DIAG_V90_LEFT:
  case MOTION_DIAG_OUT45_LEFT:
  case MOTION_DIAG_OUT135_LEFT:
    return true;
  default:
    return false;
  }
}

/* 経路を本当の迷路の上で走らせる。壁を通らずに経路の終わりの位置まで行ければ true。
 * ends_at_cell_center：最短経路のように、最後に区画の真ん中まで半区画進んで終わる経路か
 *
 * 各動きを、1 区画ずつの動き（直進・左・右・Uターン）に戻して走らせる。
 *   大回り 90°      = 半区画 + ターン + 半区画
 *   大回り 180°     = 半区画 + ターン + 同じターン + 半区画
 *   斜め入り 45°    = 半区画 + ターン（このあと斜めの中は、左右が交互のターンが続く）
 *   斜め入り 135°   = 半区画 + ターン + 同じターン
 *   斜めの直進 n    = 左右交互のターン n 個（前のターンの反対から始まる）
 *   V90             = ターン + 同じターン
 *   斜め出 45°      = ターン + 半区画
 *   斜め出 135°     = ターン + 同じターン + 半区画 */
static bool replay_path(const path_t *path, int x, int y, direction_t dir, bool ends_at_cell_center)
{
  bool ok = true;
  bool last_turn_left = false;  /* 最後のターンが左だったか（斜めの直進で交互にするため） */
  s_pending_half = 0;

  for (int i = 0; i < path->count && ok; i++) {
    motion_t motion = path->steps[i].motion;
    action_t turn = ACTION_TURN_RIGHT;
    if (is_left_motion(motion)) {
      turn = ACTION_TURN_LEFT;
    }

    switch (motion) {
    case MOTION_STRAIGHT:
      s_pending_half = s_pending_half + path->steps[i].half_count;
      break;
    case MOTION_TURN_LEFT:
    case MOTION_TURN_RIGHT:
      ok = replay_turn(&x, &y, &dir, turn);
      break;
    case MOTION_UTURN:
      ok = replay_turn(&x, &y, &dir, ACTION_UTURN);
      break;
    case MOTION_BIG_TURN_LEFT_90:
    case MOTION_BIG_TURN_RIGHT_90:
      s_pending_half = s_pending_half + 1;
      ok = replay_turn(&x, &y, &dir, turn);
      s_pending_half = 1;
      break;
    case MOTION_BIG_TURN_LEFT_180:
    case MOTION_BIG_TURN_RIGHT_180:
      s_pending_half = s_pending_half + 1;
      ok = replay_turn(&x, &y, &dir, turn);
      if (ok) {
        ok = replay_turn(&x, &y, &dir, turn);
      }
      s_pending_half = 1;
      break;
    case MOTION_DIAG_IN45_LEFT:
    case MOTION_DIAG_IN45_RIGHT:
      s_pending_half = s_pending_half + 1;
      ok = replay_turn(&x, &y, &dir, turn);
      break;
    case MOTION_DIAG_IN135_LEFT:
    case MOTION_DIAG_IN135_RIGHT:
    case MOTION_DIAG_V90_LEFT:
    case MOTION_DIAG_V90_RIGHT:
      if (motion == MOTION_DIAG_IN135_LEFT || motion == MOTION_DIAG_IN135_RIGHT) {
        s_pending_half = s_pending_half + 1;
      }
      ok = replay_turn(&x, &y, &dir, turn);
      if (ok) {
        ok = replay_turn(&x, &y, &dir, turn);
      }
      break;
    case MOTION_DIAG_STRAIGHT:
      for (int turn_count = 0; turn_count < path->steps[i].half_count && ok; turn_count++) {
        if (last_turn_left) {
          ok = replay_turn(&x, &y, &dir, ACTION_TURN_RIGHT);
          last_turn_left = false;
        } else {
          ok = replay_turn(&x, &y, &dir, ACTION_TURN_LEFT);
          last_turn_left = true;
        }
      }
      break;
    case MOTION_DIAG_OUT45_LEFT:
    case MOTION_DIAG_OUT45_RIGHT:
      ok = replay_turn(&x, &y, &dir, turn);
      s_pending_half = 1;
      break;
    case MOTION_DIAG_OUT135_LEFT:
    case MOTION_DIAG_OUT135_RIGHT:
      ok = replay_turn(&x, &y, &dir, turn);
      if (ok) {
        ok = replay_turn(&x, &y, &dir, turn);
      }
      s_pending_half = 1;
      break;
    }

    if (motion != MOTION_STRAIGHT && motion != MOTION_DIAG_STRAIGHT && motion != MOTION_UTURN) {
      last_turn_left = is_left_motion(motion);
    }
  }

  /* 最後に区画の真ん中まで進む経路なら、半区画が 1 つ余るのが正しい */
  if (ok && ends_at_cell_center) {
    if (s_pending_half % 2 == 1) {
      s_pending_half = s_pending_half - 1;
    } else {
      ok = false;
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

/* 未知壁探索で見に行く壁。s_watch_enabled が true なら、run_to_target() はこの壁を見た時点でも止まる */
static bool s_watch_enabled = false;
static int s_watch_x = 0;
static int s_watch_y = 0;
static direction_t s_watch_dir = DIR_NORTH;

/* s_stop_on_uturn が true なら、run_to_target() は U ターンする手前で止まり、s_stopped_for_uturn を true にする
 * （未知壁探索では、U ターンで止まるついでにダイクストラを計算し直すため）。
 * 走り出してすぐの U ターン（止まって計算したあとの最初の動き）では止まらない */
static bool s_stop_on_uturn = false;
static bool s_stopped_for_uturn = false;

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
    if (s_watch_enabled && maze_map_get_wall(s_watch_x, s_watch_y, s_watch_dir) != WALL_UNKNOWN) {
      return move_count;  /* 見に行く壁を見た */
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
        if (!replay_path(&s_path, *x, *y, *dir, false)) {
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

    if (s_stop_on_uturn && action == ACTION_UTURN && move_count > 0) {
      s_stopped_for_uturn = true;
      return move_count;  /* U ターンの前に止まる */
    }

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

/* 迷路ファイルの迷路を、1 区画ずつ探索した場合と、既知区間をまとめて走った場合で比べる */
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
  printf("  既知区間を使う:   行き %d 区画、帰り %d 区画（このほかに既知区間を %d 回まとめて走った）\n",
         goal_moves_known, start_moves_known, s_known_path_count);
  check(s_big_turn_count == 0, "既知区間の経路には大回りが入らない");

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
 * 5. 最短経路のテスト
 * ------------------------------------------------------------------------- */

/* 経路の中に motion があるか */
static bool path_has(const path_t *path, motion_t motion)
{
  for (int i = 0; i < path->count; i++) {
    if (path->steps[i].motion == motion) {
      return true;
    }
  }
  return false;
}

/* 経路の中に斜めの動きがあるか */
static bool path_has_diagonal(const path_t *path)
{
  if (path_has(path, MOTION_DIAG_IN45_LEFT) || path_has(path, MOTION_DIAG_IN45_RIGHT)) {
    return true;
  }
  if (path_has(path, MOTION_DIAG_IN135_LEFT) || path_has(path, MOTION_DIAG_IN135_RIGHT)) {
    return true;
  }
  return false;
}

/* 区画 (x, y) がゴールの中か */
static bool is_in_goal(int x, int y)
{
  if (x < s_goal_x || x >= s_goal_x + s_goal_width) {
    return false;
  }
  if (y < s_goal_y || y >= s_goal_y + s_goal_height) {
    return false;
  }
  return true;
}

/* 本当の迷路の壁を、全部「見た」ことにして地図に写す */
static void copy_true_maze_to_map(void)
{
  maze_map_init();
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int i = 0; i < 4; i++) {
        direction_t dir = (direction_t)i;
        if (s_true_wall[x][y][dir]) {
          maze_map_set_wall(x, y, dir, WALL_EXIST);
        } else {
          maze_map_set_wall(x, y, dir, WALL_NONE);
        }
      }
    }
  }
}

/* 中に壁が 1 枚もない迷路（スタートの東の壁だけある）を、本当の迷路にする */
static void make_open_true_maze(void)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      s_true_wall[x][y][DIR_NORTH] = (y == MAZE_SIZE - 1);
      s_true_wall[x][y][DIR_SOUTH] = (y == 0);
      s_true_wall[x][y][DIR_EAST] = (x == MAZE_SIZE - 1);
      s_true_wall[x][y][DIR_WEST] = (x == 0);
    }
  }
  s_true_wall[0][0][DIR_EAST] = true;
  s_true_wall[1][0][DIR_WEST] = true;
}

/* 最短経路を作って、本当の迷路の上で走らせて確かめる */
static void check_shortest(const char *name)
{
  static path_t s_path;
  bool found = path_make_shortest(s_goal_x, s_goal_y, s_goal_width, s_goal_height, &s_path);
  check(found, "最短経路が見つかる");
  if (!found) {
    return;
  }
  printf("  %s: 区画 %d 個を通る、動き %d 個\n", name, dijkstra_get_route_length(), s_path.count);
  print_path(&s_path);
  check(replay_path(&s_path, 0, 0, DIR_NORTH, true), "最短経路が壁を通らず、経路の終わりの位置に着く");
  check(is_in_goal(s_path.end_x, s_path.end_y), "最短経路の終わりはゴールの中");
}

/* 中に壁がない迷路で、最短経路が斜めを使うかを調べる */
static void test_shortest_open(void)
{
  printf("[5] 最短経路：中に壁がない迷路で、(0,0) → (6,6)\n");
  make_open_true_maze();
  copy_true_maze_to_map();
  s_goal_x = 6;
  s_goal_y = 6;
  s_goal_width = 1;
  s_goal_height = 1;
  check_shortest("壁なし");

  static path_t s_path;
  path_make_shortest(s_goal_x, s_goal_y, s_goal_width, s_goal_height, &s_path);
  check(path_has_diagonal(&s_path), "壁がなければ斜めに走る");
  check(path_has(&s_path, MOTION_DIAG_STRAIGHT), "斜めの直進がある");

  /* まだ見ていない壁は「ある」とみなすので、何も見ていなければ道はない */
  maze_map_init();
  check(!path_make_shortest(s_goal_x, s_goal_y, s_goal_width, s_goal_height, &s_path),
        "何も見ていない迷路では最短経路は見つからない");
}

/* 迷路ファイルの迷路で、探索のあとと、全部の壁を知っているときの最短経路を調べる */
static void test_shortest_maze(const char *file_path)
{
  printf("[5] 最短経路: %s\n", file_path);
  if (!load_maze_file(file_path)) {
    check(false, "迷路ファイルが読めない");
    return;
  }

  /* 探索（行き・帰り）で見た壁だけを使う */
  int goal_moves = 0;
  int start_moves = 0;
  explore_round_trip(true, &goal_moves, &start_moves);
  check_shortest("探索のあと");

  /* 迷路の壁を全部知っているとき */
  copy_true_maze_to_map();
  check_shortest("全部の壁を知っているとき");
}

/* ---------------------------------------------------------------------------
 * 6. 未知壁探索のテスト
 * ------------------------------------------------------------------------- */

/* 今の本当の迷路で、未知壁探索の流れ（行き → 見ていない壁を見に行く → 止まって最短経路を計算 → 帰り）を走らせる。
 * 最後の最短経路が、全部の壁を知っているときと同じ速さなら true。verbose が true なら途中経過を表示する */
static bool run_unknown_wall_flow(bool verbose)
{
  /* 全部の壁を知っているときの最短経路のコスト（これが目標） */
  copy_true_maze_to_map();
  bool found_best = dijkstra_find_route(s_goal_x, s_goal_y, s_goal_width, s_goal_height, true);
  check(found_best, "全部の壁を知っているとき、ゴールへの道がある");
  int best_cost = dijkstra_get_route_cost();

  /* 1. 行き：スタート → ゴール */
  maze_map_init();
  int x = 0;
  int y = 0;
  direction_t dir = DIR_NORTH;
  search_set_target(s_goal_x, s_goal_y, s_goal_width, s_goal_height);
  int moves_to_goal = run_to_target(&x, &y, &dir, true);
  check(moves_to_goal >= 0, "ゴールに着く");

  bool found_after_goal = dijkstra_find_route(s_goal_x, s_goal_y, s_goal_width, s_goal_height, true);
  check(found_after_goal, "ゴールに着いた時点で、見た壁だけでゴールへの道がある");
  int cost_after_goal = dijkstra_get_route_cost();

  /* 2. ゴールで止まって、未知壁探索の経路を計算する（ダイクストラは止まっているときだけ） */
  int plan_count = 1;     /* 止まってダイクストラを計算した回数 */
  int blocked_count = 0;  /* そのうち、経路がふさがれたため */
  int uturn_count = 0;    /* そのうち、U ターンのついで */
  check(search_plan_unknown_walls(s_goal_x, s_goal_y, s_goal_width, s_goal_height), "ゴールで未知壁探索の経路を計算できる");

  /* 3. 経路の上の見ていない壁を、なくなるまで見に行く */
  int wall_count = 0;
  int moves_for_walls = 0;
  while (true) {
    if (wall_count >= 300) {
      check(false, "見に行く壁が 300 枚を超えた（終わらない）");
      break;
    }

    int cell_x;
    int cell_y;
    direction_t wall_dir;
    unknown_wall_result_t result = search_check_unknown_walls(&cell_x, &cell_y, &wall_dir);
    if (result == UNKNOWN_WALL_NONE) {
      break;  /* 経路の壁を全部見た */
    }
    if (result == UNKNOWN_WALL_NO_PLAN) {
      check(false, "未知壁探索の経路がない");
      break;
    }
    if (result == UNKNOWN_WALL_BLOCKED) {
      /* 経路がふさがれた：止まって計算し直す */
      plan_count = plan_count + 1;
      blocked_count = blocked_count + 1;
      search_plan_unknown_walls(s_goal_x, s_goal_y, s_goal_width, s_goal_height);
      continue;
    }

    /* 見ていない壁の手前の区画へ、止まらずに走る */
    wall_count = wall_count + 1;
    s_watch_enabled = true;
    s_watch_x = cell_x;
    s_watch_y = cell_y;
    s_watch_dir = wall_dir;
    s_stop_on_uturn = true;
    s_stopped_for_uturn = false;
    search_set_target(cell_x, cell_y, 1, 1);
    int moves = run_to_target(&x, &y, &dir, true);
    s_watch_enabled = false;
    s_stop_on_uturn = false;

    check(moves >= 0, "見に行く壁の手前の区画に着く");
    if (moves < 0) {
      break;
    }
    moves_for_walls = moves_for_walls + moves;

    if (s_stopped_for_uturn) {
      /* U ターンで止まったついでに計算し直す */
      plan_count = plan_count + 1;
      uturn_count = uturn_count + 1;
      search_plan_unknown_walls(s_goal_x, s_goal_y, s_goal_width, s_goal_height);
    }
  }

  /* 4. 見に行く壁がなくなったら、その場で止まって最短経路を計算する
   *    （走りながら計算しない。計算が終わってから動き出す） */
  if (verbose) {
    printf("  見に行く壁がなくなった：(%d,%d) で止まって最短経路を計算する\n", x, y);
  }
  static path_t s_shortest;
  bool found = path_make_shortest(s_goal_x, s_goal_y, s_goal_width, s_goal_height, &s_shortest);
  check(found, "止まった場所で最短経路を計算できる");
  int cost_final = dijkstra_get_route_cost();

  /* 5. 帰り：スタートへ（最短走行はスタートから始まるので） */
  search_set_target(0, 0, 1, 1);
  int moves_to_start = run_to_target(&x, &y, &dir, true);
  check(moves_to_start >= 0, "スタートに戻る");
  check(count_wrong_walls() == 0, "記録した壁が本当の迷路と同じ");

  /* 結果：止まった場所で計算した最短経路が、全部の壁を知っているときと同じ速さか。
   * 帰りに新しい壁を見ても、最短経路は変わらない（もう一度計算して確かめる） */
  bool found_after_return = dijkstra_find_route(s_goal_x, s_goal_y, s_goal_width, s_goal_height, true);
  check(found_after_return, "帰ったあとも、ゴールへの道がある");
  check(dijkstra_get_route_cost() == cost_final, "帰りに壁を見ても、止まった場所で計算した最短経路は変わらない");
  if (verbose) {
    printf("  ゴールに着いた時点の最短経路のコスト %d → 止まった場所で計算 %d（全部の壁を知っているとき %d）\n",
           cost_after_goal, cost_final, best_cost);
  }
  if (verbose) {
    printf("  見に行った壁 %d 枚、そのために進んだ区画 %d（既知区間でまとめて走った分を除く）\n",
           wall_count, moves_for_walls);
  }
  if (verbose) {
    printf("  止まってダイクストラを計算した回数 %d（ゴール 1、経路がふさがれた %d、U ターンのついで %d）\n",
           plan_count, blocked_count, uturn_count);
  }
  return cost_final == best_cost;
}

/* 迷路ファイルの迷路で、未知壁探索の流れを調べる */
static void test_unknown_wall_explore(const char *file_path)
{
  printf("[6] 未知壁探索: %s\n", file_path);
  if (!load_maze_file(file_path)) {
    check(false, "迷路ファイルが読めない");
    return;
  }
  check(run_unknown_wall_flow(true), "未知壁探索のあとの最短経路は、全部の壁を知っているときと同じ速さ");
}

/* ---------------------------------------------------------------------------
 * 7. ランダムな迷路のテスト
 * ------------------------------------------------------------------------- */

/* 決まった順に乱数を出す（毎回同じ迷路になるように、自分で作る） */
static unsigned int s_random_seed = 1;

static int random_below(int limit)
{
  s_random_seed = s_random_seed * 1103515245u + 12345u;
  return (int)((s_random_seed / 65536u) % (unsigned int)limit);
}

/* 本当の迷路の、(x, y) の dir 側の壁を取る（隣の区画の側も） */
static void open_true_wall(int x, int y, direction_t dir)
{
  s_true_wall[x][y][dir] = false;
  s_true_wall[maze_next_x(x, dir)][maze_next_y(y, dir)][maze_turn_back(dir)] = false;
}

/* ランダムな迷路を作って、本当の迷路にする（gen_maze.py と同じ作り方）。
 * ゴールは 2x2〜3x3 で、場所もランダム */
static void make_random_true_maze(void)
{
  static bool s_visited_cell[MAZE_SIZE][MAZE_SIZE];
  static int s_stack_x[MAZE_SIZE * MAZE_SIZE];
  static int s_stack_y[MAZE_SIZE * MAZE_SIZE];

  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int i = 0; i < 4; i++) {
        s_true_wall[x][y][i] = true;
      }
      s_visited_cell[x][y] = false;
    }
  }

  /* 1. 穴掘り法で、全部の区画をつなぐ */
  int stack_count = 1;
  s_stack_x[0] = 0;
  s_stack_y[0] = 0;
  s_visited_cell[0][0] = true;
  while (stack_count > 0) {
    int x = s_stack_x[stack_count - 1];
    int y = s_stack_y[stack_count - 1];
    direction_t candidates[4];
    int candidate_count = 0;
    for (int i = 0; i < 4; i++) {
      direction_t dir = (direction_t)i;
      int next_x = maze_next_x(x, dir);
      int next_y = maze_next_y(y, dir);
      if (maze_is_inside(next_x, next_y) && !s_visited_cell[next_x][next_y]) {
        candidates[candidate_count] = dir;
        candidate_count = candidate_count + 1;
      }
    }
    if (candidate_count == 0) {
      stack_count = stack_count - 1;
      continue;
    }
    direction_t dir = candidates[random_below(candidate_count)];
    open_true_wall(x, y, dir);
    int next_x = maze_next_x(x, dir);
    int next_y = maze_next_y(y, dir);
    s_visited_cell[next_x][next_y] = true;
    s_stack_x[stack_count] = next_x;
    s_stack_y[stack_count] = next_y;
    stack_count = stack_count + 1;
  }

  /* 2. 壁を 15% くらい壊して、道が何通りもある迷路にする */
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      if (y + 1 < MAZE_SIZE && random_below(100) < 15) {
        open_true_wall(x, y, DIR_NORTH);
      }
      if (x + 1 < MAZE_SIZE && random_below(100) < 15) {
        open_true_wall(x, y, DIR_EAST);
      }
    }
  }

  /* 3. ゴール（中の壁はなし） */
  s_goal_width = 2 + random_below(2);
  s_goal_height = s_goal_width;
  s_goal_x = 4 + random_below(MAZE_SIZE - 8);
  s_goal_y = 4 + random_below(MAZE_SIZE - 8);
  for (int x = s_goal_x; x < s_goal_x + s_goal_width; x++) {
    for (int y = s_goal_y; y < s_goal_y + s_goal_height; y++) {
      if (y + 1 < s_goal_y + s_goal_height) {
        open_true_wall(x, y, DIR_NORTH);
      }
      if (x + 1 < s_goal_x + s_goal_width) {
        open_true_wall(x, y, DIR_EAST);
      }
    }
  }

  /* 4. スタートの東は必ず壁、北は必ず開いている（規則。東を閉じても閉じ込められないように北を開ける） */
  s_true_wall[0][0][DIR_EAST] = true;
  s_true_wall[1][0][DIR_WEST] = true;
  open_true_wall(0, 0, DIR_NORTH);
}

/* ランダムな迷路 maze_count 個で、全部の壁を知っているときの最短経路が走れるかを調べる */
static void test_random_mazes(int maze_count)
{
  printf("[7] ランダムな迷路 %d 個で、全部の壁を知っているときの最短経路\n", maze_count);
  static path_t s_path;
  int fail_count = 0;
  for (int maze_index = 0; maze_index < maze_count; maze_index++) {
    s_random_seed = (unsigned int)(maze_index + 1);
    make_random_true_maze();
    copy_true_maze_to_map();
    bool found = path_make_shortest(s_goal_x, s_goal_y, s_goal_width, s_goal_height, &s_path);
    bool ok = found;
    if (ok) {
      ok = replay_path(&s_path, 0, 0, DIR_NORTH, true);
    }
    if (ok) {
      ok = is_in_goal(s_path.end_x, s_path.end_y);
    }
    if (!ok) {
      fail_count = fail_count + 1;
      if (fail_count <= 3) {
        printf("  迷路 %d 番（乱数の種 %d）で失敗\n", maze_index, maze_index + 1);
      }
    }
  }
  printf("  失敗した迷路: %d 個\n", fail_count);
  check(fail_count == 0, "ランダムな迷路の最短経路が、どれも壁を通らずにゴールに着く");
}

/* ランダムな迷路 maze_count 個で、未知壁探索のあとの最短経路が、全部の壁を知っているときと同じ速さかを調べる */
static void test_unknown_wall_random(int maze_count)
{
  printf("[6] 未知壁探索：ランダムな迷路 %d 個\n", maze_count);
  int fail_count = 0;
  for (int maze_index = 0; maze_index < maze_count; maze_index++) {
    s_random_seed = (unsigned int)(maze_index + 10001);
    make_random_true_maze();
    if (!run_unknown_wall_flow(false)) {
      fail_count = fail_count + 1;
    }
  }
  printf("  全部の壁を知っているときより遅い最短経路になった迷路: %d 個\n", fail_count);
  check(fail_count == 0, "ランダムな迷路でも、未知壁探索のあとの最短経路は全部の壁を知っているときと同じ速さ");
}

/* ---------------------------------------------------------------------------
 * 8. ダイクストラの答えが本当の最短かのテスト
 * ------------------------------------------------------------------------- */

#define REF_COST_MAX 1000000  /* まだたどり着いていない */

/* 確認用の計算で使う、状態「点 + その点に来た向き（8 方向）」ごとのコスト [横の壁か][x][y][向き] */
static int s_ref_cost[2][MAZE_SIZE + 1][MAZE_SIZE + 1][8];

/* 確認用：ダイクストラとは別のやり方で、本当の最短のコストを求める。
 * 状態に「来た向き」まで含め、コストが変わらなくなるまで全部の状態を何度も見直す（遅いが確実）。
 * まだ見ていない壁は「ある」とみなす。道がなければ REF_COST_MAX */
static int calc_reference_cost(int goal_x, int goal_y, int goal_width, int goal_height)
{
  for (int row = 0; row < 2; row++) {
    for (int x = 0; x <= MAZE_SIZE; x++) {
      for (int y = 0; y <= MAZE_SIZE; y++) {
        for (int i = 0; i < 8; i++) {
          s_ref_cost[row][x][y][i] = REF_COST_MAX;
        }
      }
    }
  }

  /* ゴールの入口を、ゴールから外へ向かう向きでコスト 0 にする（dijkstra.c の add_goal_entrance と同じ） */
  for (int x = goal_x; x < goal_x + goal_width; x++) {
    for (int y = goal_y; y < goal_y + goal_height; y++) {
      for (int i = 0; i < 4; i++) {
        direction_t dir = (direction_t)i;
        if (maze_is_in_area(maze_next_x(x, dir), maze_next_y(y, dir), goal_x, goal_y, goal_width, goal_height)) {
          continue;
        }
        edge_t edge;
        dir8_t outward;
        if (dir == DIR_NORTH) {
          edge = make_edge(true, x, y + 1);
          outward = DIR8_NORTH;
        } else if (dir == DIR_SOUTH) {
          edge = make_edge(true, x, y);
          outward = DIR8_SOUTH;
        } else if (dir == DIR_EAST) {
          edge = make_edge(false, x + 1, y);
          outward = DIR8_EAST;
        } else {
          edge = make_edge(false, x, y);
          outward = DIR8_WEST;
        }
        if (is_open(edge, true)) {
          s_ref_cost[edge.is_row][edge.x][edge.y][outward] = 0;
        }
      }
    }
  }

  /* コストが変わらなくなるまで見直す */
  bool changed = true;
  while (changed) {
    changed = false;
    for (int row = 0; row < 2; row++) {
      for (int x = 0; x <= MAZE_SIZE; x++) {
        for (int y = 0; y <= MAZE_SIZE; y++) {
          for (int i = 0; i < 8; i++) {
            int cost = s_ref_cost[row][x][y][i];
            if (cost == REF_COST_MAX) {
              continue;
            }
            dir8_t came_dir = (dir8_t)i;
            edge_t from = make_edge(row == 1, x, y);
            direction_t crossing = crossing_after(from, came_dir);
            for (int j = 0; j < 8; j++) {
              dir8_t dir = (dir8_t)j;
              if (!is_forward_move(crossing, dir)) {
                continue;
              }
              edge_t to;
              int cell_x;
              int cell_y;
              if (!move_from(from, dir, &to, &cell_x, &cell_y)) {
                continue;
              }
              if (!maze_is_inside(cell_x, cell_y) || !is_open(to, true)) {
                continue;
              }
              int step_cost;
              if (dir == came_dir) {
                step_cost = COST_CONTINUE;
              } else if (is_orthogonal(dir)) {
                step_cost = COST_STRAIGHT;
              } else {
                step_cost = COST_DIAGONAL;
              }
              if (cost + step_cost < s_ref_cost[to.is_row][to.x][to.y][dir]) {
                s_ref_cost[to.is_row][to.x][to.y][dir] = cost + step_cost;
                changed = true;
              }
            }
          }
        }
      }
    }
  }

  /* スタートの区画の北の壁を、ゴール側から見て南向き（南・南東・南西）に通る状態のうち、いちばん安いもの */
  int best = s_ref_cost[1][0][1][DIR8_SOUTH];
  if (s_ref_cost[1][0][1][DIR8_SOUTH_EAST] < best) {
    best = s_ref_cost[1][0][1][DIR8_SOUTH_EAST];
  }
  if (s_ref_cost[1][0][1][DIR8_SOUTH_WEST] < best) {
    best = s_ref_cost[1][0][1][DIR8_SOUTH_WEST];
  }
  return best;
}

/* 区画に向き heading_in で入り、向き heading_out で出るときの、点から点への向き（8 方向）。
 * 同じなら縦横の向き、違えば 2 つを合わせた斜めの向き（例：北から入って東へ出るなら北東） */
static dir8_t hop_direction(direction_t heading_in, direction_t heading_out)
{
  bool north = (heading_in == DIR_NORTH || heading_out == DIR_NORTH);
  bool south = (heading_in == DIR_SOUTH || heading_out == DIR_SOUTH);
  bool east = (heading_in == DIR_EAST || heading_out == DIR_EAST);
  bool west = (heading_in == DIR_WEST || heading_out == DIR_WEST);
  if (north && east) {
    return DIR8_NORTH_EAST;
  }
  if (north && west) {
    return DIR8_NORTH_WEST;
  }
  if (south && east) {
    return DIR8_SOUTH_EAST;
  }
  if (south && west) {
    return DIR8_SOUTH_WEST;
  }
  if (north) {
    return DIR8_NORTH;
  }
  if (south) {
    return DIR8_SOUTH;
  }
  if (east) {
    return DIR8_EAST;
  }
  return DIR8_WEST;
}

/* dijkstra_find_route() が返した区画の並びを実際にたどったときのコストを、区画の並びから数え直す。
 * コストはゴール側から数える（ゴールに入る向きにそのまま進むところは COST_CONTINUE） */
static int count_route_cost(void)
{
  int length = dijkstra_get_route_length();
  int last = length - 1;  /* ゴールの区画の番号 */
  direction_t into_goal = maze_direction_between(dijkstra_get_route_x(last - 1), dijkstra_get_route_y(last - 1),
                                                 dijkstra_get_route_x(last), dijkstra_get_route_y(last));
  dir8_t previous = hop_direction(into_goal, into_goal);
  int cost = 0;
  /* スタートの区画（0 番）とゴールの区画（last 番）の間の区画を、ゴール側から順に */
  for (int i = last - 1; i >= 1; i--) {
    direction_t heading_in = maze_direction_between(dijkstra_get_route_x(i - 1), dijkstra_get_route_y(i - 1),
                                                    dijkstra_get_route_x(i), dijkstra_get_route_y(i));
    direction_t heading_out = maze_direction_between(dijkstra_get_route_x(i), dijkstra_get_route_y(i),
                                                     dijkstra_get_route_x(i + 1), dijkstra_get_route_y(i + 1));
    dir8_t hop = reverse_dir8(hop_direction(heading_in, heading_out));  /* ゴール側から見た向き */
    if (hop == reverse_dir8(previous)) {
      cost = cost + COST_CONTINUE;
    } else if (is_orthogonal(hop)) {
      cost = cost + COST_STRAIGHT;
    } else {
      cost = cost + COST_DIAGONAL;
    }
    previous = reverse_dir8(hop);
  }
  return cost;
}

/* ランダムな迷路 maze_count 個で、ダイクストラの答えのコストが本当の最短と同じか、
 * 返した区画の並びのコストが報告したコストと同じかを調べる */
static void test_dijkstra_exact(int maze_count)
{
  printf("[8] ダイクストラの答えが本当の最短か：ランダムな迷路 %d 個\n", maze_count);
  int not_shortest_count = 0;
  int wrong_cost_count = 0;
  for (int maze_index = 0; maze_index < maze_count; maze_index++) {
    s_random_seed = (unsigned int)(maze_index + 20001);
    make_random_true_maze();
    copy_true_maze_to_map();
    if (!dijkstra_find_route(s_goal_x, s_goal_y, s_goal_width, s_goal_height, true)) {
      not_shortest_count = not_shortest_count + 1;
      continue;
    }
    if (dijkstra_get_route_cost() != calc_reference_cost(s_goal_x, s_goal_y, s_goal_width, s_goal_height)) {
      not_shortest_count = not_shortest_count + 1;
    }
    if (dijkstra_get_route_cost() != count_route_cost()) {
      wrong_cost_count = wrong_cost_count + 1;
    }
  }
  printf("  本当の最短でなかった迷路: %d 個、返した道のコストと報告したコストが違った迷路: %d 個\n",
         not_shortest_count, wrong_cost_count);
  check(not_shortest_count == 0, "ダイクストラの答えは本当の最短");
  check(wrong_cost_count == 0, "返した道のコストと、報告したコストが同じ");
}

/* ---------------------------------------------------------------------------
 * 9. 探索走行（sys/explore.c）のテスト
 *
 * explore.c が使う「機体を動かす」（ctrl/run.h）、「壁を見る」（ctrl/wall_ctrl.h）、
 * 「フェイルセーフ」（sys/failsafe.h）、「Flash に保存」（sys/flash.h）の関数を、ここで仮に作る（仮の機体）。
 * 仮の機体は、本当の迷路（s_true_wall）の上を 1 区画ずつ動き、おかしな使い方をされたら数える。
 *   - 壁に突っ込んだ
 *   - 走っているのに「発進」した、止まっているのに「直進」した など、状態に合わない動きを頼まれた
 *   - 止まっているのに壁を見た（壁は区画の入口の辺で見る）
 *   - 走っているあいだに Flash に保存した（保存中は CPU が止まるので、止まっているときだけ）
 * ------------------------------------------------------------------------- */

static int s_sim_x = 0;                /* 仮の機体の位置と向き */
static int s_sim_y = 0;
static direction_t s_sim_dir = DIR_NORTH;
static bool s_sim_stopped = true;      /* 区画の真ん中で止まっているか */
static bool s_sim_at_start = false;    /* スタート区画に置かれて、まだ走り出していないか */
static int s_sim_motion_count = 0;     /* 動きを頼まれた回数 */
static int s_sim_failsafe_at = -1;     /* この回数目の動きでフェイルセーフを起こす（-1 なら起こさない） */
static bool s_sim_failsafe = false;    /* フェイルセーフで止まっているか */
static bool s_sim_all_walls = false;   /* true なら、センサがいつも「前・左・右とも壁あり」と答える（読み間違いの試験） */
static int s_sim_error_count = 0;      /* おかしな使い方をされた回数 */
static int s_sim_stop_count = 0;       /* 止まった回数（U ターンと、未知壁探索で計算するときなど） */
static int s_sim_save_count = 0;       /* Flash に保存した回数 */

/* 迷路の地図の写し（フェイルセーフで戻した結果と比べるため） */
typedef struct {
  wall_state_t walls[MAZE_SIZE][MAZE_SIZE][4];
  bool visited[MAZE_SIZE][MAZE_SIZE];
} map_copy_t;

/* 壁を記録する直前の地図の写し。新しい MAZE_UNDO_CELL_COUNT 回ぶん（[0] がいちばん古い） */
static map_copy_t s_sim_before_record[MAZE_UNDO_CELL_COUNT];
static int s_sim_before_record_count = 0;
static map_copy_t s_sim_at_place;  /* スタートに置いたときの地図 */
static map_copy_t s_sim_flash;     /* 最後に Flash に保存した地図 */

/* 今の地図を写す */
static void copy_map(map_copy_t *copy)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int i = 0; i < 4; i++) {
        copy->walls[x][y][i] = maze_map_get_wall(x, y, (direction_t)i);
      }
      copy->visited[x][y] = maze_map_is_visited(x, y);
    }
  }
}

/* 2 つの地図の写しが同じか */
static bool same_map(const map_copy_t *a, const map_copy_t *b)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int i = 0; i < 4; i++) {
        if (a->walls[x][y][i] != b->walls[x][y][i]) {
          return false;
        }
      }
      if (a->visited[x][y] != b->visited[x][y]) {
        return false;
      }
    }
  }
  return true;
}

/* おかしな使い方をされたことを数える。最初の数回だけ表示する */
static void sim_error(const char *message)
{
  if (s_sim_error_count < 5) {
    printf("  仮の機体: %s（(%d,%d) 向き %d）\n", message, s_sim_x, s_sim_y, s_sim_dir);
  }
  s_sim_error_count = s_sim_error_count + 1;
}

/* 機体をスタート区画に北向きに置く（走らせる前に呼ぶ）。数えたものも 0 に戻す */
static void sim_place_at_start(int failsafe_at)
{
  s_sim_x = 0;
  s_sim_y = 0;
  s_sim_dir = DIR_NORTH;
  s_sim_stopped = true;
  s_sim_at_start = true;
  s_sim_motion_count = 0;
  s_sim_failsafe_at = failsafe_at;
  s_sim_failsafe = false;
  s_sim_error_count = 0;
  s_sim_stop_count = 0;
  s_sim_save_count = 0;
  s_sim_before_record_count = 0;
  copy_map(&s_sim_at_place);
}

/* 動きを 1 つ頼まれた。フェイルセーフを起こす回数になったら起こす。
 * フェイルセーフで止まっていれば false（動かない。本物の機体も、モーターが止まっているので動かない） */
static bool sim_begin_motion(void)
{
  if (s_sim_failsafe) {
    return false;
  }
  s_sim_motion_count = s_sim_motion_count + 1;
  if (s_sim_motion_count == s_sim_failsafe_at) {
    s_sim_failsafe = true;
    s_sim_stopped = true;
    return false;
  }
  return true;
}

/* 向き dir に 1 区画進み、次の区画の入口の辺へ行く。壁があれば突っ込んだとして数える */
static void sim_go(direction_t dir)
{
  s_sim_dir = dir;
  if (s_true_wall[s_sim_x][s_sim_y][dir]) {
    sim_error("壁に突っ込んだ");
    return;
  }
  s_sim_x = maze_next_x(s_sim_x, dir);
  s_sim_y = maze_next_y(s_sim_y, dir);
  s_sim_stopped = false;
}

/* 走っているときにだけ頼んでよい動きか調べる */
static void sim_must_be_moving(const char *name)
{
  if (s_sim_stopped) {
    sim_error(name);
  }
}

void run_search_start(void)
{
  if (!sim_begin_motion()) {
    return;
  }
  if (!s_sim_at_start) {
    sim_error("スタートに置いていないのに走り出した");
  }
  s_sim_at_start = false;
  sim_go(DIR_NORTH);
}

void run_search_straight(void)
{
  if (!sim_begin_motion()) {
    return;
  }
  sim_must_be_moving("止まっているのに直進した");
  sim_go(s_sim_dir);
}

void run_search_turn_left(void)
{
  if (!sim_begin_motion()) {
    return;
  }
  sim_must_be_moving("止まっているのにスラロームした");
  sim_go(maze_turn_left(s_sim_dir));
}

void run_search_turn_right(void)
{
  if (!sim_begin_motion()) {
    return;
  }
  sim_must_be_moving("止まっているのにスラロームした");
  sim_go(maze_turn_right(s_sim_dir));
}

void run_search_stop_at_center(void)
{
  if (!sim_begin_motion()) {
    return;
  }
  sim_must_be_moving("止まっているのに、また止まった");
  s_sim_stopped = true;
  s_sim_stop_count = s_sim_stop_count + 1;
}

void run_search_go_from_center(run_turn_t turn)
{
  if (!sim_begin_motion()) {
    return;
  }
  if (!s_sim_stopped) {
    sim_error("走っているのに、止まったところからの発進をした");
  }
  direction_t dir = s_sim_dir;
  if (turn == RUN_TURN_LEFT) {
    dir = maze_turn_left(dir);
  } else if (turn == RUN_TURN_RIGHT) {
    dir = maze_turn_right(dir);
  } else if (turn == RUN_TURN_BACK) {
    dir = maze_turn_back(dir);
  }
  sim_go(dir);
}

/* 壁を見る。explore.c は前・左・右の順に見てから記録するので、前を見たときに「記録する直前の地図」を写しておく */
bool wall_ctrl_has_wall_front(void)
{
  if (s_sim_stopped) {
    sim_error("止まっているのに壁を見た");
  }
  if (s_sim_before_record_count == MAZE_UNDO_CELL_COUNT) {
    for (int i = 0; i + 1 < MAZE_UNDO_CELL_COUNT; i++) {
      s_sim_before_record[i] = s_sim_before_record[i + 1];
    }
    s_sim_before_record_count = MAZE_UNDO_CELL_COUNT - 1;
  }
  copy_map(&s_sim_before_record[s_sim_before_record_count]);
  s_sim_before_record_count = s_sim_before_record_count + 1;

  if (s_sim_all_walls) {
    return true;
  }
  return s_true_wall[s_sim_x][s_sim_y][s_sim_dir];
}

bool wall_ctrl_has_wall_left(void)
{
  if (s_sim_all_walls) {
    return true;
  }
  return s_true_wall[s_sim_x][s_sim_y][maze_turn_left(s_sim_dir)];
}

bool wall_ctrl_has_wall_right(void)
{
  if (s_sim_all_walls) {
    return true;
  }
  return s_true_wall[s_sim_x][s_sim_y][maze_turn_right(s_sim_dir)];
}

bool failsafe_is_stopped(void)
{
  return s_sim_failsafe;
}

void flash_save_maze(void)
{
  if (!s_sim_stopped) {
    sim_error("走っているあいだに Flash に保存した");
  }
  s_sim_save_count = s_sim_save_count + 1;
  copy_map(&s_sim_flash);
}

/* 全部の壁を知っているときの最短経路のコスト（未知壁探索の目標） */
static int best_route_cost(void)
{
  copy_true_maze_to_map();
  if (!dijkstra_find_route(s_goal_x, s_goal_y, s_goal_width, s_goal_height, true)) {
    return -1;
  }
  return dijkstra_get_route_cost();
}

/* 今の地図（探索で見た壁）での最短経路のコスト */
static int recorded_route_cost(void)
{
  if (!dijkstra_find_route(s_goal_x, s_goal_y, s_goal_width, s_goal_height, true)) {
    return -1;
  }
  return dijkstra_get_route_cost();
}

/* 走り方の名前（表示用） */
static const char *explore_type_name(explore_type_t type)
{
  if (type == EXPLORE_ONE_WAY) {
    return "片道";
  }
  if (type == EXPLORE_UNKNOWN_WALLS) {
    return "未知壁探索";
  }
  return "往復";
}

/* 空の迷路から、走り方 type で探索させる。最後まで走り、終わりの場所・保存・壁が正しければ true。
 * 未知壁探索なら、最短経路が全部の壁を知っているときと同じ速さかも調べる。verbose なら結果を表示する */
static bool explore_and_check(explore_type_t type, bool verbose)
{
  int best_cost = best_route_cost();

  maze_map_init();
  sim_place_at_start(-1);
  explore_result_t result = explore_run(type, s_goal_x, s_goal_y, s_goal_width, s_goal_height);

  bool ok = true;
  if (result != EXPLORE_RESULT_DONE) {
    ok = false;
  }
  if (s_sim_error_count != 0) {
    ok = false;
  }
  if (!s_sim_stopped) {
    ok = false;  /* 最後は止まっている */
  }
  if (s_sim_save_count != 1) {
    ok = false;  /* 最後に 1 回だけ保存する */
  }
  if (count_wrong_walls() != 0) {
    ok = false;
  }
  if (type == EXPLORE_ONE_WAY) {
    if (!is_in_goal(s_sim_x, s_sim_y)) {
      ok = false;
    }
  } else {
    if (s_sim_x != 0 || s_sim_y != 0) {
      ok = false;
    }
  }
  int cost = recorded_route_cost();
  if (type == EXPLORE_UNKNOWN_WALLS && cost != best_cost) {
    ok = false;
  }

  if (verbose) {
    printf("  %s: 動き %d 回、止まった %d 回、最後の位置 (%d,%d)、最短経路のコスト %d（全部の壁を知っているとき %d）\n",
           explore_type_name(type), s_sim_motion_count, s_sim_stop_count, s_sim_x, s_sim_y, cost, best_cost);
  }
  return ok;
}

/* 迷路ファイルの迷路で、3 つの走り方を試す */
static void test_explore_types(const char *file_path)
{
  printf("[9] 探索走行（explore.c）: %s\n", file_path);
  if (!load_maze_file(file_path)) {
    check(false, "迷路ファイルが読めない");
    return;
  }
  check(explore_and_check(EXPLORE_ROUND_TRIP, true), "往復：スタートに戻って止まり、保存する");
  check(explore_and_check(EXPLORE_ONE_WAY, true), "片道：ゴールで止まり、保存する");
  check(explore_and_check(EXPLORE_UNKNOWN_WALLS, true), "未知壁探索：スタートに戻り、最短経路が全部の壁を知っているときと同じ");
}

/* 走り方 type で、failsafe_at 回目の動きでフェイルセーフを起こす。
 * 最後の 5 区画ぶんが戻って保存されるか、そのあとスタートに置き直して続きから探索できるかを調べる */
static bool failsafe_and_continue(explore_type_t type, int failsafe_at)
{
  int best_cost = best_route_cost();
  bool ok = true;

  /* 1. 空の迷路から走らせて、途中でフェイルセーフを起こす */
  maze_map_init();
  sim_place_at_start(failsafe_at);
  explore_result_t result = explore_run(type, s_goal_x, s_goal_y, s_goal_width, s_goal_height);
  if (result != EXPLORE_RESULT_FAILSAFE) {
    return true;  /* フェイルセーフを起こす前に走り終わった（この回数では試せない） */
  }

  /* 2. 最後に壁を記録した 5 区画ぶんが、記録する前に戻っているか（1 回も記録していなければ、置いたときのまま） */
  map_copy_t now;
  copy_map(&now);
  if (s_sim_before_record_count > 0) {
    if (!same_map(&now, &s_sim_before_record[0])) {
      ok = false;
    }
  } else {
    if (!same_map(&now, &s_sim_at_place)) {
      ok = false;
    }
  }
  if (s_sim_save_count != 1 || !same_map(&s_sim_flash, &now)) {
    ok = false;  /* 戻したあとの迷路を 1 回保存する */
  }
  if (s_sim_error_count != 0) {
    ok = false;
  }

  /* 3. スタートに置き直して、続きから同じ走り方で探索する */
  sim_place_at_start(-1);
  result = explore_run(type, s_goal_x, s_goal_y, s_goal_width, s_goal_height);
  if (result != EXPLORE_RESULT_DONE || s_sim_error_count != 0 || count_wrong_walls() != 0) {
    ok = false;
  }
  if (type == EXPLORE_UNKNOWN_WALLS && recorded_route_cost() != best_cost) {
    ok = false;
  }
  return ok;
}

/* フェイルセーフで止まったときのテスト（迷路ファイルの迷路で、起こす時刻をいろいろ変える） */
static void test_explore_failsafe(const char *file_path)
{
  printf("[9] 探索走行：フェイルセーフで止まったとき: %s\n", file_path);
  if (!load_maze_file(file_path)) {
    check(false, "迷路ファイルが読めない");
    return;
  }
  int fail_count = 0;
  int tried_count = 0;
  for (int failsafe_at = 1; failsafe_at <= 400; failsafe_at = failsafe_at + 7) {
    if (!failsafe_and_continue(EXPLORE_ROUND_TRIP, failsafe_at)) {
      fail_count = fail_count + 1;
    }
    if (!failsafe_and_continue(EXPLORE_UNKNOWN_WALLS, failsafe_at)) {
      fail_count = fail_count + 1;
    }
    tried_count = tried_count + 2;
  }
  printf("  試した回数 %d、うまくいかなかった回数 %d\n", tried_count, fail_count);
  check(fail_count == 0, "最後の 5 区画ぶんを戻して保存し、続きから探索できる");
}

/* センサが読み間違えて、ゴールへの道がなくなったとき */
static void test_explore_no_route(void)
{
  printf("[9] 探索走行：壁を読み間違えて、ゴールへの道がなくなったとき\n");
  load_maze_file("mazes/maze32_center2x2.txt");
  maze_map_init();
  sim_place_at_start(-1);
  s_sim_all_walls = true;  /* どこを見ても「壁あり」と答える */
  explore_result_t result = explore_run(EXPLORE_ROUND_TRIP, s_goal_x, s_goal_y, s_goal_width, s_goal_height);
  s_sim_all_walls = false;
  check(result == EXPLORE_RESULT_NO_ROUTE, "道がなくなったと答える");
  check(s_sim_stopped, "その場で止まっている");
  check(s_sim_save_count == 0, "保存しない");
  check(s_sim_error_count == 0, "おかしな動きをしない");
}

/* ランダムな迷路で、3 つの走り方と、フェイルセーフのあとの続きを試す */
static void test_explore_random(int maze_count)
{
  printf("[9] 探索走行：ランダムな迷路 %d 個\n", maze_count);
  int fail_count = 0;
  int failsafe_fail_count = 0;
  for (int maze_index = 0; maze_index < maze_count; maze_index++) {
    s_random_seed = (unsigned int)(maze_index + 20001);
    make_random_true_maze();
    if (!explore_and_check(EXPLORE_ROUND_TRIP, false)) {
      fail_count = fail_count + 1;
    }
    if (!explore_and_check(EXPLORE_ONE_WAY, false)) {
      fail_count = fail_count + 1;
    }
    if (!explore_and_check(EXPLORE_UNKNOWN_WALLS, false)) {
      fail_count = fail_count + 1;
    }
    int failsafe_at = 1 + random_below(300);
    if (!failsafe_and_continue(EXPLORE_UNKNOWN_WALLS, failsafe_at)) {
      failsafe_fail_count = failsafe_fail_count + 1;
    }
  }
  printf("  うまくいかなかった走行 %d 回、フェイルセーフのあとうまくいかなかった %d 回\n", fail_count, failsafe_fail_count);
  check(fail_count == 0, "ランダムな迷路でも、3 つの走り方が正しく終わる");
  check(failsafe_fail_count == 0, "ランダムな迷路でも、フェイルセーフのあと続きから探索できる");
}

/* ---------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(void)
{
  test_maze_map();
  test_undo_recent_cells();
  test_step_map();
  test_known_path();
  test_explore("mazes/maze32_center2x2.txt");
  test_explore("mazes/maze32_offset3x3.txt");
  test_shortest_open();
  test_shortest_maze("mazes/maze32_center2x2.txt");
  test_shortest_maze("mazes/maze32_offset3x3.txt");
  test_unknown_wall_explore("mazes/maze32_center2x2.txt");
  test_unknown_wall_explore("mazes/maze32_offset3x3.txt");
  test_random_mazes(2000);
  test_unknown_wall_random(300);
  test_dijkstra_exact(500);
  test_explore_types("mazes/maze32_center2x2.txt");
  test_explore_types("mazes/maze32_offset3x3.txt");
  test_explore_failsafe("mazes/maze32_center2x2.txt");
  test_explore_failsafe("mazes/maze32_offset3x3.txt");
  test_explore_no_route();
  test_explore_random(300);

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗: %d 件\n", s_fail_count);
  return 1;
}
