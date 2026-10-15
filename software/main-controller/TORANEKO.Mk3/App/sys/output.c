/*
 * output.c
 * 出力（MODES.md のモード7 No.1、KOGUMA_PORTING.md の B4・B5）
 *
 * RAM に残っているデータを、迷路 → 走行ログ の順に出し、最後に #DONE を出す（ないものは出さない）。
 * どれも KOGUMA のビューア（tools/viewer.html）で読める形にする。
 *
 * ■ 迷路の形（KOGUMA の Maze_Debug_Dump と同じ。ビューアが迷路の図にする）
 *   #BEGIN MAZE size=32 goal_x=ゴールの左下の x goal_y=ゴールの左下の y
 *   MAZE_DUMP_BEGIN 32
 *   ROW k 値    k = 0〜32。横の線 y = k の上の壁（x 番目の区画の辺に壁があれば、値の x 番目の桁が 1）
 *   COL k 値    k = 0〜32。縦の線 x = k の上の壁（y 番目の区画の辺に壁があれば、値の y 番目の桁が 1）
 *   LROW k 値   ROW と同じ並びで、見たことのある壁なら 1
 *   LCOL k 値   COL と同じ並びで、見たことのある壁なら 1
 *   MAZE_DUMP_END
 *   unknown walls: まだ見ていない壁の数
 *   DIJK_COST コスト
 *   DIJK_PATH: 最短走行で走る動きの並び（まだ見ていない壁は「あり」とみなす。最短走行と同じ）
 *   #END MAZE
 * 値は、32 桁の 2 進数を 16 進数 8 桁で書いたもの（例：0 番目と 2 番目に壁があれば 1 + 4 = 5 → 00000005）。
 * 線の番号は、いちばん下（南）の外周が y = 0、いちばん左（西）の外周が x = 0。
 *
 * ■ たくさんの文字を送るとき
 * UART の送信待ちバッファ（2KB）が一杯になると文字を捨てるので、少しずつ、空きができるのを待ってから出す。
 */
#include "sys/output.h"

#include <stdbool.h>
#include <stdio.h>

#include "drv/uart.h"
#include "maze/dijkstra.h"
#include "maze/maze_map.h"
#include "maze/path.h"
#include "sys/goal.h"
#include "sys/log.h"

#define LINE_LENGTH_MAX 120  /* 1 行の長さの上限 [文字]。出す前に、送信待ちバッファにこれだけの空きを待つ */
#define WORD_LENGTH_MAX 32   /* 動きの名前 1 つの長さの上限 [文字] */

/* 最短経路（DIJK_PATH を出すときに使う） */
static path_t s_path;

/* ---------------------------------------------------------------------------
 * 迷路
 * ------------------------------------------------------------------------- */

/* 横の線 y = line_y の、x 番目の区画の辺の壁（line_y = MAZE_SIZE は、いちばん上の外周） */
static wall_state_t get_row_wall(int line_y, int x)
{
  if (line_y < MAZE_SIZE) {
    return maze_map_get_wall(x, line_y, DIR_SOUTH);
  }
  return maze_map_get_wall(x, MAZE_SIZE - 1, DIR_NORTH);
}

/* 縦の線 x = line_x の、y 番目の区画の辺の壁（line_x = MAZE_SIZE は、いちばん右の外周） */
static wall_state_t get_col_wall(int line_x, int y)
{
  if (line_x < MAZE_SIZE) {
    return maze_map_get_wall(line_x, y, DIR_WEST);
  }
  return maze_map_get_wall(MAZE_SIZE - 1, y, DIR_EAST);
}

/* 壁の状態 state を 1 にするか。want_seen が true なら「見たことがある」、false なら「壁がある」を 1 にする */
static bool is_marked(wall_state_t state, bool want_seen)
{
  if (want_seen) {
    if (state != WALL_UNKNOWN) {
      return true;
    }
    return false;
  }
  if (state == WALL_EXIST) {
    return true;
  }
  return false;
}

/* 横の線 y = line_y の 32 枚の壁を、32 桁の 2 進数にする（x 番目の壁が当てはまれば、x 番目の桁を 1 にする）。
 * 桁の重み digit を 1、2、4、8、… と 2 倍していき、当てはまる壁の重みを足す。
 * unsigned long は、ARM でも PC でも 32 桁以上入り、printf の %lX でそのまま出せる */
static unsigned long row_bits(int line_y, bool want_seen)
{
  unsigned long value = 0;
  unsigned long digit = 1;
  for (int x = 0; x < MAZE_SIZE; x++) {
    if (is_marked(get_row_wall(line_y, x), want_seen)) {
      value = value + digit;
    }
    digit = digit * 2;
  }
  return value;
}

/* 縦の線 x = line_x の 32 枚の壁を、32 桁の 2 進数にする（y 番目の壁が当てはまれば、y 番目の桁を 1 にする） */
static unsigned long col_bits(int line_x, bool want_seen)
{
  unsigned long value = 0;
  unsigned long digit = 1;
  for (int y = 0; y < MAZE_SIZE; y++) {
    if (is_marked(get_col_wall(line_x, y), want_seen)) {
      value = value + digit;
    }
    digit = digit * 2;
  }
  return value;
}

/* まだ見ていない壁の数 */
static int count_unknown_walls(void)
{
  int count = 0;
  for (int line = 0; line <= MAZE_SIZE; line++) {
    for (int i = 0; i < MAZE_SIZE; i++) {
      if (get_row_wall(line, i) == WALL_UNKNOWN) {
        count = count + 1;
      }
      if (get_col_wall(line, i) == WALL_UNKNOWN) {
        count = count + 1;
      }
    }
  }
  return count;
}

/* 動きの名前（PC テストの表示と同じ） */
static const char *motion_name(motion_t motion)
{
  switch (motion) {
  case MOTION_STRAIGHT:           return "直進";
  case MOTION_TURN_LEFT:          return "左";
  case MOTION_TURN_RIGHT:         return "右";
  case MOTION_UTURN:              return "Uターン";
  case MOTION_BIG_TURN_LEFT_90:   return "大回り左90";
  case MOTION_BIG_TURN_RIGHT_90:  return "大回り右90";
  case MOTION_BIG_TURN_LEFT_180:  return "大回り左180";
  case MOTION_BIG_TURN_RIGHT_180: return "大回り右180";
  case MOTION_DIAG_IN45_LEFT:     return "斜め入り左45";
  case MOTION_DIAG_IN45_RIGHT:    return "斜め入り右45";
  case MOTION_DIAG_IN135_LEFT:    return "斜め入り左135";
  case MOTION_DIAG_IN135_RIGHT:   return "斜め入り右135";
  case MOTION_DIAG_STRAIGHT:      return "斜め直進";
  case MOTION_DIAG_V90_LEFT:      return "V90左";
  case MOTION_DIAG_V90_RIGHT:     return "V90右";
  case MOTION_DIAG_OUT45_LEFT:    return "斜め出左45";
  case MOTION_DIAG_OUT45_RIGHT:   return "斜め出右45";
  case MOTION_DIAG_OUT135_LEFT:   return "斜め出左135";
  case MOTION_DIAG_OUT135_RIGHT:  return "斜め出右135";
  }
  return "?";
}

/* 最短走行で走る動きの並びを出す（まだ見ていない壁は「あり」とみなす。最短走行と同じ）。
 * 直進と斜めの直進は、名前のあとに長さ（半区画の数、斜めの区画の数）を付ける */
static void print_shortest_path(int goal_x, int goal_y)
{
  if (!path_make_shortest(goal_x, goal_y, GOAL_WIDTH, GOAL_HEIGHT, &s_path)) {
    uart_wait_free(LINE_LENGTH_MAX);
    printf("NO PATH: start cannot reach goal (unknown walls count as walls)\r\n");
    return;
  }

  uart_wait_free(LINE_LENGTH_MAX);
  printf("DIJK_COST %d\r\n", dijkstra_get_route_cost());
  printf("DIJK_PATH:");
  for (int i = 0; i < s_path.count; i++) {
    motion_t motion = s_path.steps[i].motion;
    uart_wait_free(WORD_LENGTH_MAX);
    if (motion == MOTION_STRAIGHT || motion == MOTION_DIAG_STRAIGHT) {
      printf(" %s%d", motion_name(motion), s_path.steps[i].half_count);
    } else {
      printf(" %s", motion_name(motion));
    }
  }
  printf("\r\n");
}

/* 迷路（壁と、見たことのある壁）と最短経路を出す。ゴールは左下の区画 (goal_x, goal_y)、大きさは 2×2 */
void output_maze(int goal_x, int goal_y)
{
  uart_wait_free(LINE_LENGTH_MAX);
  printf("\r\n#BEGIN MAZE size=%d goal_x=%d goal_y=%d\r\n", MAZE_SIZE, goal_x, goal_y);
  printf("MAZE_DUMP_BEGIN %d\r\n", MAZE_SIZE);

  for (int line = 0; line <= MAZE_SIZE; line++) {
    uart_wait_free(LINE_LENGTH_MAX);
    printf("ROW %d %08lX\r\n", line, row_bits(line, false));
  }
  for (int line = 0; line <= MAZE_SIZE; line++) {
    uart_wait_free(LINE_LENGTH_MAX);
    printf("COL %d %08lX\r\n", line, col_bits(line, false));
  }
  for (int line = 0; line <= MAZE_SIZE; line++) {
    uart_wait_free(LINE_LENGTH_MAX);
    printf("LROW %d %08lX\r\n", line, row_bits(line, true));
  }
  for (int line = 0; line <= MAZE_SIZE; line++) {
    uart_wait_free(LINE_LENGTH_MAX);
    printf("LCOL %d %08lX\r\n", line, col_bits(line, true));
  }

  uart_wait_free(LINE_LENGTH_MAX);
  printf("MAZE_DUMP_END\r\n");
  printf("unknown walls: %d\r\n", count_unknown_walls());

  print_shortest_path(goal_x, goal_y);

  uart_wait_free(LINE_LENGTH_MAX);
  printf("#END MAZE\r\n");
}

/* ---------------------------------------------------------------------------
 * 全部
 * ------------------------------------------------------------------------- */

/* RAM に残っているデータを全部出す（迷路 → 走行ログ → #DONE）。走行ログは、記録していなければ出さない */
void output_all(int goal_x, int goal_y)
{
  output_maze(goal_x, goal_y);
  if (log_has_data()) {
    log_print();
  }
  uart_wait_free(LINE_LENGTH_MAX);
  printf("\r\n#DONE\r\n");
}
