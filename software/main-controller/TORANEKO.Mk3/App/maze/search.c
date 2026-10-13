/*
 * search.c
 * 探索（足立法）：歩数マップを作って、次にどちらへ進むかを決める
 *
 * ■ 歩数マップ
 * 各区画に「目的地まであと何区画か」を書いた地図。目的地の区画が 0 で、
 * 1 区画離れるごとに 1 ずつ増える。まだ見ていない壁は「ない」ものとして数える
 * （足立法。見ていない場所は通れると考えて、近そうな道から試す）。
 *
 * ■ 次の動きの決め方
 * 前・左・右・後ろの区画のうち、壁がなくて歩数がいちばん小さい区画へ進む。
 * 同じ歩数なら 前 → 左 → 右 → 後ろ の順に選ぶ（KOGUMA と同じ。直進を優先すると速い）。
 */
#include "maze/search.h"

/* 目的地（長方形の範囲）。(左下の x, 左下の y) と 幅・高さ [区画] */
static int s_target_x = 0;
static int s_target_y = 0;
static int s_target_width = 1;
static int s_target_height = 1;

static uint16_t s_step_map[MAZE_SIZE][MAZE_SIZE];

/* 歩数マップを作るときの「次に調べる区画」の列（待ち行列）。
 * 1 つの区画は 1 回しか入らないので、大きさは区画の数あれば足りる */
static int s_queue_x[MAZE_SIZE * MAZE_SIZE];
static int s_queue_y[MAZE_SIZE * MAZE_SIZE];

/* 目的地を決める。ゴールなら大会のゴールの範囲、帰りならスタート (0, 0, 1, 1) を渡す */
void search_set_target(int x, int y, int width, int height)
{
  s_target_x = x;
  s_target_y = y;
  s_target_width = width;
  s_target_height = height;
}

/* (x, y) が目的地の範囲の中か */
bool search_is_in_target(int x, int y)
{
  if (x < s_target_x || x >= s_target_x + s_target_width) {
    return false;
  }
  if (y < s_target_y || y >= s_target_y + s_target_height) {
    return false;
  }
  return true;
}

/* (x, y) から向き dir へ進めるか（足立法なので、まだ見ていない壁は通れるとみなす） */
static bool can_pass(int x, int y, direction_t dir)
{
  if (maze_map_get_wall(x, y, dir) == WALL_EXIST) {
    return false;
  }
  return true;
}

/* 歩数マップを作り直す。
 * 目的地の区画を 0 にして、そこから壁のないところへ 1 区画ずつ広げていく（幅優先探索） */
void search_update_step_map(void)
{
  int queue_read = 0;   /* 次に取り出す位置 */
  int queue_write = 0;  /* 次に入れる位置 */

  /* 1. 全部「たどり着けない」にしてから、目的地の区画を 0 にして待ち行列に入れる */
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      if (search_is_in_target(x, y)) {
        s_step_map[x][y] = 0;
        s_queue_x[queue_write] = x;
        s_queue_y[queue_write] = y;
        queue_write = queue_write + 1;
      } else {
        s_step_map[x][y] = STEP_MAX;
      }
    }
  }

  /* 2. 待ち行列から 1 つ取り出し、壁のない隣の区画で、まだ歩数が決まっていないところに
   *    「自分の歩数 + 1」を書いて待ち行列に入れる。待ち行列が空になるまで繰り返す */
  while (queue_read < queue_write) {
    int x = s_queue_x[queue_read];
    int y = s_queue_y[queue_read];
    queue_read = queue_read + 1;

    for (int i = 0; i < 4; i++) {
      direction_t dir = (direction_t)i;
      if (!can_pass(x, y, dir)) {
        continue;
      }
      int next_x = maze_next_x(x, dir);
      int next_y = maze_next_y(y, dir);
      if (!maze_is_inside(next_x, next_y)) {
        continue;
      }
      if (s_step_map[next_x][next_y] != STEP_MAX) {
        continue;  /* もう歩数が決まっている（もっと近い道がある） */
      }
      s_step_map[next_x][next_y] = s_step_map[x][y] + 1;
      s_queue_x[queue_write] = next_x;
      s_queue_y[queue_write] = next_y;
      queue_write = queue_write + 1;
    }
  }
}

/* 歩数マップの値を読む。迷路の外なら STEP_MAX */
uint16_t search_get_step(int x, int y)
{
  if (!maze_is_inside(x, y)) {
    return STEP_MAX;
  }
  return s_step_map[x][y];
}

/* (x, y) から向き dir に進んだ先の区画の歩数。壁があって進めなければ STEP_MAX */
static uint16_t get_step_toward(int x, int y, direction_t dir)
{
  if (!can_pass(x, y, dir)) {
    return STEP_MAX;
  }
  return search_get_step(maze_next_x(x, dir), maze_next_y(y, dir));
}

/* 区画 (x, y) で向き dir を向いているときの、次の動きを決める。
 * 先に search_update_step_map() で歩数マップを作っておくこと */
action_t search_decide_action(int x, int y, direction_t dir)
{
  uint16_t step_front = get_step_toward(x, y, dir);
  uint16_t step_left = get_step_toward(x, y, maze_turn_left(dir));
  uint16_t step_right = get_step_toward(x, y, maze_turn_right(dir));
  uint16_t step_back = get_step_toward(x, y, maze_turn_back(dir));

  /* いちばん小さい歩数を探す */
  uint16_t step_min = step_front;
  if (step_left < step_min) {
    step_min = step_left;
  }
  if (step_right < step_min) {
    step_min = step_right;
  }
  if (step_back < step_min) {
    step_min = step_back;
  }

  if (step_min == STEP_MAX) {
    return ACTION_STOP;  /* どの方向からも目的地にたどり着けない */
  }

  /* 同じ歩数なら 前 → 左 → 右 → 後ろ の順に選ぶ */
  if (step_front == step_min) {
    return ACTION_FORWARD;
  }
  if (step_left == step_min) {
    return ACTION_TURN_LEFT;
  }
  if (step_right == step_min) {
    return ACTION_TURN_RIGHT;
  }
  return ACTION_UTURN;
}
