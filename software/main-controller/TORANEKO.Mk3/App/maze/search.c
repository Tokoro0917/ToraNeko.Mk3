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

#include "maze/dijkstra.h"

/* 目的地（長方形の範囲）。(左下の x, 左下の y) と 幅・高さ [区画] */
static int s_target_x = 0;
static int s_target_y = 0;
static int s_target_width = 1;
static int s_target_height = 1;

static uint16_t s_step_map[MAZE_SIZE][MAZE_SIZE];

/* 歩数マップを作るときの「次に調べる区画」の列（待ち行列）。
 * 1 つの区画は 1 回しか入らないので、大きさは区画の数あれば足りる。
 * 座標は 0〜31 なので、メモリを節約するため 1 バイトの整数にする */
static uint8_t s_queue_x[MAZE_SIZE * MAZE_SIZE];
static uint8_t s_queue_y[MAZE_SIZE * MAZE_SIZE];

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
  return maze_is_in_area(x, y, s_target_x, s_target_y, s_target_width, s_target_height);
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
        s_queue_x[queue_write] = (uint8_t)x;
        s_queue_y[queue_write] = (uint8_t)y;
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
      s_queue_x[queue_write] = (uint8_t)next_x;
      s_queue_y[queue_write] = (uint8_t)next_y;
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

/* 未知壁探索で使う経路（ダイクストラで計算した、スタートからゴールまでの区画の並び）。
 * 座標は 0〜31 なので、メモリを節約するため 1 バイトの整数にする */
static uint8_t s_plan_x[MAZE_SIZE * MAZE_SIZE];
static uint8_t s_plan_y[MAZE_SIZE * MAZE_SIZE];
static int s_plan_length = 0;

/* 未知壁探索の経路を計算して覚える（KOGUMA の Maze_Unknown_Walls_Plan）。重いので、止まっているときに呼ぶ。
 *
 * まだ見ていない壁を「ない」とみなしてダイクストラで最短経路を探す。
 * 見ていない壁は通れるとみなす、いちばん都合のいい考え方なので、この経路の壁を全部見て
 * 全部なかったら、本当の迷路でもこれより速い道はない。
 * ゴールへの道がなければ false（普通は起きない）。 */
bool search_plan_unknown_walls(int goal_x, int goal_y, int goal_width, int goal_height)
{
  s_plan_length = 0;
  if (!dijkstra_find_route(goal_x, goal_y, goal_width, goal_height, false)) {
    return false;
  }
  int length = dijkstra_get_route_length();
  for (int i = 0; i < length; i++) {
    s_plan_x[i] = (uint8_t)dijkstra_get_route_x(i);
    s_plan_y[i] = (uint8_t)dijkstra_get_route_y(i);
  }
  s_plan_length = length;
  return true;
}

/* 覚えた経路の i 番目の区画から i+1 番目の区画へ向かう向き */
static direction_t plan_direction(int i)
{
  return maze_direction_between(s_plan_x[i], s_plan_y[i], s_plan_x[i + 1], s_plan_y[i + 1]);
}

/* 覚えた経路の壁を見直す（KOGUMA の Maze_Unknown_Wall_Scan と Maze_Unknown_Wall_Is_Wall）。軽いので、走りながら呼んでよい。
 *
 *   1. 経路の上の壁に「ある」ものが 1 枚でもあれば UNKNOWN_WALL_BLOCKED（止まって計算し直す）。
 *      見に行った壁だけでなく、途中で見えた経路の上の壁も調べる
 *   2. そうでなく、「まだ見ていない」壁があれば UNKNOWN_WALL_FOUND。スタート側から見て最初の 1 枚について、
 *      手前の区画を (cell_x, cell_y) に、その区画から見た壁の向きを wall_dir に入れる。
 *      その区画まで行けば、センサでその壁を見られる（まだ入ったことのない区画なので、入ると前・左・右の壁を見る）
 *   3. どちらでもなければ UNKNOWN_WALL_NONE（経路の壁は全部見てあり、全部なかった）。
 * 経路を覚えていなければ UNKNOWN_WALL_NO_PLAN（「全部見た」と区別するため）。 */
unknown_wall_result_t search_check_unknown_walls(int *cell_x, int *cell_y, direction_t *wall_dir)
{
  /* 0. 経路を覚えていない（search_plan_unknown_walls をまだ呼んでいないか、ゴールへの道がなかった） */
  if (s_plan_length == 0) {
    return UNKNOWN_WALL_NO_PLAN;
  }

  /* 1. ふさがれていないか */
  for (int i = 0; i + 1 < s_plan_length; i++) {
    if (maze_map_get_wall(s_plan_x[i], s_plan_y[i], plan_direction(i)) == WALL_EXIST) {
      return UNKNOWN_WALL_BLOCKED;
    }
  }

  /* 2. まだ見ていない壁 */
  for (int i = 0; i + 1 < s_plan_length; i++) {
    direction_t dir = plan_direction(i);
    if (maze_map_get_wall(s_plan_x[i], s_plan_y[i], dir) == WALL_UNKNOWN) {
      *cell_x = s_plan_x[i];
      *cell_y = s_plan_y[i];
      *wall_dir = dir;
      return UNKNOWN_WALL_FOUND;
    }
  }

  return UNKNOWN_WALL_NONE;
}
