/*
 * maze_map.c
 * 迷路の壁の情報（どこに壁があるか、まだ見ていないか）を持っておくところ
 *
 * ■ 壁の持ち方
 * 区画ごとに、北・東・南・西の 4 枚の壁の状態を持つ（s_walls[x][y][向き]）。
 * 1 枚の壁は隣り合う 2 つの区画から見えるので（例：(0,0) の北の壁 = (0,1) の南の壁）、
 * 書き込むときは maze_map_set_wall() で必ず両側をそろえて書く。
 *
 * ■ 最後の 5 区画ぶんを戻す（フェイルセーフで止まったとき。MODES.md の 5）
 * ぶつかる直前は機体がずれていて、壁を読み間違えているかもしれない。そこで、壁を記録するたびに
 * 「その区画と、記録する前の 4 枚の壁」を覚えておき（新しい 5 区画ぶん）、フェイルセーフで止まったら
 * 新しいほうから順に、記録する前の状態に戻す。
 */
#include "maze/maze_map.h"

static wall_state_t s_walls[MAZE_SIZE][MAZE_SIZE][4];  /* [x][y][向き] */
static bool s_visited[MAZE_SIZE][MAZE_SIZE];           /* その区画に入って壁を見たか */

/* 壁を記録する前の、1 つの区画の状態 */
typedef struct {
  int x;
  int y;
  wall_state_t walls[4];  /* 記録する前の 4 枚の壁 [向き] */
  bool visited;           /* 記録する前に、その区画に入ったことがあったか */
} undo_cell_t;

/* 覚えている区画。s_undo_cells[0] がいちばん古く、s_undo_cells[s_undo_count - 1] がいちばん新しい */
static undo_cell_t s_undo_cells[MAZE_UNDO_CELL_COUNT];
static int s_undo_count = 0;

/* 迷路の情報を消して、最初の状態にする。
 * 外周の壁と、スタート区画の東の壁（規則で必ずある）は最初から「あり」にしておく */
void maze_map_init(void)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      s_walls[x][y][DIR_NORTH] = WALL_UNKNOWN;
      s_walls[x][y][DIR_EAST] = WALL_UNKNOWN;
      s_walls[x][y][DIR_SOUTH] = WALL_UNKNOWN;
      s_walls[x][y][DIR_WEST] = WALL_UNKNOWN;
      s_visited[x][y] = false;
    }
  }

  /* 外周 */
  for (int i = 0; i < MAZE_SIZE; i++) {
    maze_map_set_wall(i, MAZE_SIZE - 1, DIR_NORTH, WALL_EXIST);  /* 上の辺 */
    maze_map_set_wall(i, 0, DIR_SOUTH, WALL_EXIST);              /* 下の辺 */
    maze_map_set_wall(MAZE_SIZE - 1, i, DIR_EAST, WALL_EXIST);   /* 右の辺 */
    maze_map_set_wall(0, i, DIR_WEST, WALL_EXIST);               /* 左の辺 */
  }

  /* スタート区画：東は壁、北は開いている（スタートの向き） */
  maze_map_set_wall(0, 0, DIR_EAST, WALL_EXIST);
  maze_map_set_wall(0, 0, DIR_NORTH, WALL_NONE);
  s_visited[0][0] = true;

  maze_map_clear_recent_cells();
}

/* (x, y) の dir 側の壁が、規則で必ずある壁か（外周と、スタート区画の東の壁） */
static bool is_fixed_wall(int x, int y, direction_t dir)
{
  int next_x = maze_next_x(x, dir);
  int next_y = maze_next_y(y, dir);
  if (!maze_is_inside(next_x, next_y)) {
    return true;  /* 外周 */
  }
  if (x == 0 && y == 0 && dir == DIR_EAST) {
    return true;  /* スタート区画の東 */
  }
  if (x == 1 && y == 0 && dir == DIR_WEST) {
    return true;  /* 同じ壁を (1,0) から見たもの */
  }
  return false;
}

/* 壁の状態を書き込む。隣の区画から見た同じ壁も一緒に書き込む。
 * 規則で必ずある壁（外周、スタート区画の東）は、何を書き込もうとしても「あり」のまま */
void maze_map_set_wall(int x, int y, direction_t dir, wall_state_t state)
{
  if (!maze_is_inside(x, y)) {
    return;
  }

  if (is_fixed_wall(x, y, dir)) {
    state = WALL_EXIST;
  }

  int next_x = maze_next_x(x, dir);
  int next_y = maze_next_y(y, dir);
  s_walls[x][y][dir] = state;
  if (!maze_is_inside(next_x, next_y)) {
    return;  /* 外周なので、隣の区画はない */
  }
  /* 隣の区画から見ると、同じ壁は反対向きになる */
  s_walls[next_x][next_y][maze_turn_back(dir)] = state;
}

/* 壁の状態を読む。迷路の外を聞かれたら「壁あり」と答える */
wall_state_t maze_map_get_wall(int x, int y, direction_t dir)
{
  if (!maze_is_inside(x, y)) {
    return WALL_EXIST;
  }
  return s_walls[x][y][dir];
}

/* センサで見た 1 枚の壁を記録する（KOGUMA と同じく、壁は足すだけで消さない）。
 *   - 壁が見えた           → 「あり」にする
 *   - 壁が見えなかった     → まだ見ていなかった壁だけ「なし」にする。
 *                             すでに「あり」の壁は、1 回の読み間違いで消えないようにそのままにする */
static void record_one_wall(int x, int y, direction_t dir, bool wall_seen)
{
  if (wall_seen) {
    maze_map_set_wall(x, y, dir, WALL_EXIST);
    return;
  }
  if (maze_map_get_wall(x, y, dir) == WALL_UNKNOWN) {
    maze_map_set_wall(x, y, dir, WALL_NONE);
  }
}

/* 区画 (x, y) の今の状態（4 枚の壁と、入ったことがあるか）を、戻すときのために覚えておく。
 * 覚えられるのは新しい MAZE_UNDO_CELL_COUNT 区画ぶんだけ。いっぱいなら、いちばん古いものを捨てる */
static void remember_cell(int x, int y)
{
  /* いっぱいなら、全部を 1 つずつ前へずらして、いちばん古いもの（[0]）を捨てる */
  if (s_undo_count == MAZE_UNDO_CELL_COUNT) {
    for (int i = 0; i + 1 < MAZE_UNDO_CELL_COUNT; i++) {
      s_undo_cells[i] = s_undo_cells[i + 1];
    }
    s_undo_count = MAZE_UNDO_CELL_COUNT - 1;
  }

  /* いちばん後ろに足す */
  int last = s_undo_count;
  s_undo_cells[last].x = x;
  s_undo_cells[last].y = y;
  s_undo_cells[last].walls[DIR_NORTH] = s_walls[x][y][DIR_NORTH];
  s_undo_cells[last].walls[DIR_EAST] = s_walls[x][y][DIR_EAST];
  s_undo_cells[last].walls[DIR_SOUTH] = s_walls[x][y][DIR_SOUTH];
  s_undo_cells[last].walls[DIR_WEST] = s_walls[x][y][DIR_WEST];
  s_undo_cells[last].visited = s_visited[x][y];
  s_undo_count = s_undo_count + 1;
}

/* 区画 (x, y) に向き dir で入ったときに、センサで見た前・左・右の壁を記録する。
 * 後ろの壁は、通ってきたところなので記録済み（1 つ前の区画で、前・左・右のどれかの「壁なし」として書いている。
 * (0,1) の南の壁は、maze_map_init() で「壁なし」にしてある）。
 * フェイルセーフで止まったときに戻せるように、記録する前の状態を覚えておく */
void maze_map_record_walls(int x, int y, direction_t dir, bool wall_front, bool wall_left, bool wall_right)
{
  if (!maze_is_inside(x, y)) {
    return;
  }

  remember_cell(x, y);

  record_one_wall(x, y, dir, wall_front);
  record_one_wall(x, y, maze_turn_left(dir), wall_left);
  record_one_wall(x, y, maze_turn_right(dir), wall_right);

  s_visited[x][y] = true;
}

/* 最後に壁を記録した区画（MAZE_UNDO_CELL_COUNT 区画まで）を、記録する前の状態に戻す。
 * フェイルセーフで止まったときに呼ぶ（ぶつかる直前に読み間違えたかもしれない壁を残さないため）。
 * 同じ壁を 2 回記録していることもあるので、新しいほうから順に戻す（最後に、いちばん古い記録の前の状態になる）。
 * 戻したあとは、覚えている区画はなくなる */
void maze_map_undo_recent_cells(void)
{
  for (int i = s_undo_count - 1; i >= 0; i--) {
    int x = s_undo_cells[i].x;
    int y = s_undo_cells[i].y;
    /* maze_map_set_wall() で書くので、隣の区画から見た同じ壁もそろって戻る */
    maze_map_set_wall(x, y, DIR_NORTH, s_undo_cells[i].walls[DIR_NORTH]);
    maze_map_set_wall(x, y, DIR_EAST, s_undo_cells[i].walls[DIR_EAST]);
    maze_map_set_wall(x, y, DIR_SOUTH, s_undo_cells[i].walls[DIR_SOUTH]);
    maze_map_set_wall(x, y, DIR_WEST, s_undo_cells[i].walls[DIR_WEST]);
    s_visited[x][y] = s_undo_cells[i].visited;
  }
  s_undo_count = 0;
}

/* 覚えている区画を忘れる（戻せなくする）。探索を始めるときに呼ぶ。
 * 前の走行で記録して、もう Flash に保存した壁まで戻してしまわないようにするため */
void maze_map_clear_recent_cells(void)
{
  s_undo_count = 0;
}

/* その区画に入って壁を見たことがあるか */
bool maze_map_is_visited(int x, int y)
{
  if (!maze_is_inside(x, y)) {
    return false;
  }
  return s_visited[x][y];
}

/* その区画に入ったことがあるかを書き込む（Flash に保存した迷路を読み込むときに使う。
 * 探索中は maze_map_record_walls() が書き込むので、これは使わない） */
void maze_map_set_visited(int x, int y, bool visited)
{
  if (!maze_is_inside(x, y)) {
    return;
  }
  s_visited[x][y] = visited;
}

/* (x, y) が迷路の中か */
bool maze_is_inside(int x, int y)
{
  if (x < 0 || x >= MAZE_SIZE) {
    return false;
  }
  if (y < 0 || y >= MAZE_SIZE) {
    return false;
  }
  return true;
}

/* (x, y) が、左下 (area_x, area_y)・幅 width・高さ height の長方形の中か（ゴールの範囲などに使う） */
bool maze_is_in_area(int x, int y, int area_x, int area_y, int width, int height)
{
  if (x < area_x || x >= area_x + width) {
    return false;
  }
  if (y < area_y || y >= area_y + height) {
    return false;
  }
  return true;
}

/* 隣り合う区画 (from_x, from_y) から (to_x, to_y) へ向かう向き */
direction_t maze_direction_between(int from_x, int from_y, int to_x, int to_y)
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

/* 向き dir に 1 区画進んだときの x */
int maze_next_x(int x, direction_t dir)
{
  if (dir == DIR_EAST) {
    return x + 1;
  }
  if (dir == DIR_WEST) {
    return x - 1;
  }
  return x;
}

/* 向き dir に 1 区画進んだときの y */
int maze_next_y(int y, direction_t dir)
{
  if (dir == DIR_NORTH) {
    return y + 1;
  }
  if (dir == DIR_SOUTH) {
    return y - 1;
  }
  return y;
}

/* 左を向いたときの向き */
direction_t maze_turn_left(direction_t dir)
{
  switch (dir) {
  case DIR_NORTH: return DIR_WEST;
  case DIR_EAST:  return DIR_NORTH;
  case DIR_SOUTH: return DIR_EAST;
  default:        return DIR_SOUTH;  /* DIR_WEST */
  }
}

/* 右を向いたときの向き */
direction_t maze_turn_right(direction_t dir)
{
  switch (dir) {
  case DIR_NORTH: return DIR_EAST;
  case DIR_EAST:  return DIR_SOUTH;
  case DIR_SOUTH: return DIR_WEST;
  default:        return DIR_NORTH;  /* DIR_WEST */
  }
}

/* 後ろを向いたときの向き */
direction_t maze_turn_back(direction_t dir)
{
  switch (dir) {
  case DIR_NORTH: return DIR_SOUTH;
  case DIR_EAST:  return DIR_WEST;
  case DIR_SOUTH: return DIR_NORTH;
  default:        return DIR_EAST;  /* DIR_WEST */
  }
}
