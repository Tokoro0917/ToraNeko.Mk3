/*
 * dijkstra.c
 * ダイクストラ法で、スタートからゴールまでの「速く走れる道」を探す（KOGUMA の Maze_Dijkstra_Calculation）
 *
 * ■ 点（ノード）は「壁の真ん中」
 * 区画の真ん中ではなく、区画と区画の境目（壁がある・ないの位置）の真ん中を点にする。
 * 点から点へは、区画を 1 つ通って 8 方向（縦横 4 つ + 斜め 4 つ）に進める。
 *   例：横の壁の点から北へ進むと、区画を縦に通り抜けて、1 つ上の横の壁の点へ
 *       横の壁の点から北東へ進むと、区画の中で右に曲がって、その区画の東の縦の壁の点へ
 * 斜めに続けて進むと、ジグザグに曲がる道（＝斜め走行）になる。
 *
 * ■ コスト（かかる時間の目安）
 *   - 同じ向きに続けて進む ：COST_CONTINUE（速度が乗るので安い）
 *   - 縦横に向きを変えて進む：COST_STRAIGHT
 *   - 斜めに向きを変えて進む：COST_DIAGONAL
 * 区画の数が同じでも、曲がる回数が少ない道・斜めで速く走れる道が選ばれる。
 *
 * ■ 状態は「点」と「その点をどちら向きに通り抜けているか」の組
 * 同じ点でも、北向きに通るときと南向きに通るときでは、次に進める先が違う
 * （北向きなら北側の区画を通る 3 本：直進・左斜め・右斜め）。そこで、点ごとに
 * 「北向き／南向き」（縦の壁なら「東向き／西向き」）の 2 つの状態に分けてコストを持つ。
 * 分けないと「区画に入って横に抜け、同じ区画に戻る」走れない道ができてしまう
 * （KOGUMA で 2026-10-13 に直された不具合。ToraNeko のランダム迷路のテストでも 2000 個中 7 個で起きた）。
 *
 * ■ 探し方
 * ゴールの入口の点を、ゴールから外へ向かう向きでコスト 0 にして、そこから広げていく。
 * 状態のコストが下がったら、その状態を「もう一度調べる」待ち行列に入れ直す（KOGUMA と同じ。SPFA と呼ばれる方法）。
 * 最後に、スタートの区画の北の壁の点（ゴール側から見ると南向きに通る状態）から、
 * 来た向きを逆にたどるとゴールに着く。
 */
#include "maze/dijkstra.h"

#include <stdint.h>

/* コスト。KOGUMA の ST_COST, DIAG_COST, CON_COST と同じ値 */
#define COST_STRAIGHT 5
#define COST_DIAGONAL 3
#define COST_CONTINUE 2
#define COST_MAX 60000  /* まだたどり着いていない */

/* 8 方向 */
typedef enum {
  DIR8_NORTH,
  DIR8_NORTH_EAST,
  DIR8_EAST,
  DIR8_SOUTH_EAST,
  DIR8_SOUTH,
  DIR8_SOUTH_WEST,
  DIR8_WEST,
  DIR8_NORTH_WEST,
  DIR8_NONE,  /* ゴールの入口（まだどちらにも進んでいない） */
} dir8_t;

/* 壁の位置（点）
 *   is_row が true ：横の壁。区画 (x, y-1) と (x, y) の間。x は 0〜MAZE_SIZE-1、y は 0〜MAZE_SIZE
 *   is_row が false：縦の壁。区画 (x-1, y) と (x, y) の間。x は 0〜MAZE_SIZE、y は 0〜MAZE_SIZE-1 */
typedef struct {
  bool is_row;
  int8_t x;  /* 値は -1〜32 なので、メモリを節約するため 1 バイトの整数にする */
  int8_t y;
} edge_t;

/* 点を作る */
static edge_t make_edge(bool is_row, int x, int y)
{
  edge_t edge;
  edge.is_row = is_row;
  edge.x = (int8_t)x;
  edge.y = (int8_t)y;
  return edge;
}

/* 状態：点と、その点を通り抜ける向き。
 *   横の壁の点なら crossing は DIR_NORTH か DIR_SOUTH、縦の壁の点なら DIR_EAST か DIR_WEST */
typedef struct {
  edge_t edge;
  direction_t crossing;
} state_t;

/* 状態を作る */
static state_t make_state(edge_t edge, direction_t crossing)
{
  state_t state;
  state.edge = edge;
  state.crossing = crossing;
  return state;
}

/* 状態の情報を入れる配列の [x][y][向き] の「向き」の番号。北向き・東向きが 0、南向き・西向きが 1 */
static int crossing_index(direction_t crossing)
{
  if (crossing == DIR_NORTH || crossing == DIR_EAST) {
    return 0;
  }
  return 1;
}

/* 横の壁の点の状態の情報 [x][y][向き] */
static uint16_t s_row_cost[MAZE_SIZE][MAZE_SIZE + 1][2];
static dir8_t s_row_dir[MAZE_SIZE][MAZE_SIZE + 1][2];      /* その状態に来たときの向き（ゴール側から見て） */
static bool s_row_in_queue[MAZE_SIZE][MAZE_SIZE + 1][2];   /* 待ち行列に入っているか */
/* 縦の壁の点の状態の情報 [x][y][向き] */
static uint16_t s_col_cost[MAZE_SIZE + 1][MAZE_SIZE][2];
static dir8_t s_col_dir[MAZE_SIZE + 1][MAZE_SIZE][2];
static bool s_col_in_queue[MAZE_SIZE + 1][MAZE_SIZE][2];

/* 待ち行列（輪のように使う。1 つの状態は同時に 1 回しか入らないので、状態の数あれば足りる） */
#define QUEUE_SIZE (2 * (MAZE_SIZE + 1) * (MAZE_SIZE + 1) * 2)
static state_t s_queue[QUEUE_SIZE];
static int s_queue_read = 0;
static int s_queue_write = 0;

/* 見つけた道（通る区画の並び） */
static uint8_t s_route_x[ROUTE_CELL_MAX];  /* 区画の座標は 0〜31 なので 1 バイトで足りる */
static uint8_t s_route_y[ROUTE_CELL_MAX];
static int s_route_length = 0;
static int s_route_cost = 0;  /* 見つけた道のコストの合計 */

/* ===========================================================================
 * 状態の情報の読み書き（横の壁か縦の壁かで配列が違うので、ここでまとめて振り分ける）
 * ======================================================================== */

static uint16_t get_cost(state_t state)
{
  int i = crossing_index(state.crossing);
  if (state.edge.is_row) {
    return s_row_cost[state.edge.x][state.edge.y][i];
  }
  return s_col_cost[state.edge.x][state.edge.y][i];
}

static void set_cost(state_t state, uint16_t cost)
{
  int i = crossing_index(state.crossing);
  if (state.edge.is_row) {
    s_row_cost[state.edge.x][state.edge.y][i] = cost;
  } else {
    s_col_cost[state.edge.x][state.edge.y][i] = cost;
  }
}

static dir8_t get_dir(state_t state)
{
  int i = crossing_index(state.crossing);
  if (state.edge.is_row) {
    return s_row_dir[state.edge.x][state.edge.y][i];
  }
  return s_col_dir[state.edge.x][state.edge.y][i];
}

static void set_dir(state_t state, dir8_t dir)
{
  int i = crossing_index(state.crossing);
  if (state.edge.is_row) {
    s_row_dir[state.edge.x][state.edge.y][i] = dir;
  } else {
    s_col_dir[state.edge.x][state.edge.y][i] = dir;
  }
}

static bool get_in_queue(state_t state)
{
  int i = crossing_index(state.crossing);
  if (state.edge.is_row) {
    return s_row_in_queue[state.edge.x][state.edge.y][i];
  }
  return s_col_in_queue[state.edge.x][state.edge.y][i];
}

static void set_in_queue(state_t state, bool in_queue)
{
  int i = crossing_index(state.crossing);
  if (state.edge.is_row) {
    s_row_in_queue[state.edge.x][state.edge.y][i] = in_queue;
  } else {
    s_col_in_queue[state.edge.x][state.edge.y][i] = in_queue;
  }
}

/* 待ち行列に入れる（もう入っていれば何もしない） */
static void queue_push(state_t state)
{
  if (get_in_queue(state)) {
    return;
  }
  set_in_queue(state, true);
  s_queue[s_queue_write] = state;
  s_queue_write = s_queue_write + 1;
  if (s_queue_write >= QUEUE_SIZE) {
    s_queue_write = 0;
  }
}

/* 待ち行列から取り出す。空なら false */
static bool queue_pop(state_t *state)
{
  if (s_queue_read == s_queue_write) {
    return false;
  }
  *state = s_queue[s_queue_read];
  s_queue_read = s_queue_read + 1;
  if (s_queue_read >= QUEUE_SIZE) {
    s_queue_read = 0;
  }
  set_in_queue(*state, false);
  return true;
}

/* ===========================================================================
 * 迷路の形
 * ======================================================================== */

/* その点（壁の位置）を通り抜けられるか。外周は通れない。
 * unknown_as_wall が true なら、まだ見ていない壁は「ある」とみなす（最短走行で安全に走るため） */
static bool is_open(edge_t edge, bool unknown_as_wall)
{
  wall_state_t state;
  if (edge.is_row) {
    if (edge.y <= 0 || edge.y >= MAZE_SIZE) {
      return false;  /* 南と北の外周 */
    }
    state = maze_map_get_wall(edge.x, edge.y, DIR_SOUTH);
  } else {
    if (edge.x <= 0 || edge.x >= MAZE_SIZE) {
      return false;  /* 西と東の外周 */
    }
    state = maze_map_get_wall(edge.x, edge.y, DIR_WEST);
  }

  if (state == WALL_EXIST) {
    return false;
  }
  if (state == WALL_UNKNOWN && unknown_as_wall) {
    return false;
  }
  return true;
}

/* 点 from から向き dir に進んだ先の点を to に、途中で通る区画を cell_x, cell_y に入れる。
 * 横の壁の点からは東・西に、縦の壁の点からは北・南に進めない（壁に沿って進むことになる）ので false */
static bool move_from(edge_t from, dir8_t dir, edge_t *to, int *cell_x, int *cell_y)
{
  int x = from.x;
  int y = from.y;

  if (from.is_row) {
    /* 横の壁 (x, y) から。北へ向かうときは北側の区画 (x, y) を、南へ向かうときは南側の区画 (x, y-1) を通る */
    if (dir == DIR8_NORTH) {
      *to = make_edge(true, x, y + 1);        /* まっすぐ北：1 つ上の横の壁 */
      *cell_x = x;
      *cell_y = y;
      return true;
    }
    if (dir == DIR8_NORTH_EAST) {
      *to = make_edge(false, x + 1, y);       /* 北東：区画 (x, y) の東の縦の壁 */
      *cell_x = x;
      *cell_y = y;
      return true;
    }
    if (dir == DIR8_NORTH_WEST) {
      *to = make_edge(false, x, y);           /* 北西：区画 (x, y) の西の縦の壁 */
      *cell_x = x;
      *cell_y = y;
      return true;
    }
    if (dir == DIR8_SOUTH) {
      *to = make_edge(true, x, y - 1);        /* まっすぐ南：1 つ下の横の壁 */
      *cell_x = x;
      *cell_y = y - 1;
      return true;
    }
    if (dir == DIR8_SOUTH_EAST) {
      *to = make_edge(false, x + 1, y - 1);   /* 南東：区画 (x, y-1) の東の縦の壁 */
      *cell_x = x;
      *cell_y = y - 1;
      return true;
    }
    if (dir == DIR8_SOUTH_WEST) {
      *to = make_edge(false, x, y - 1);       /* 南西：区画 (x, y-1) の西の縦の壁 */
      *cell_x = x;
      *cell_y = y - 1;
      return true;
    }
    return false;  /* 東・西には進めない */
  }

  /* 縦の壁 (x, y) から。東へ向かうときは東側の区画 (x, y) を、西へ向かうときは西側の区画 (x-1, y) を通る */
  if (dir == DIR8_EAST) {
    *to = make_edge(false, x + 1, y);         /* まっすぐ東：1 つ右の縦の壁 */
    *cell_x = x;
    *cell_y = y;
    return true;
  }
  if (dir == DIR8_NORTH_EAST) {
    *to = make_edge(true, x, y + 1);          /* 北東：区画 (x, y) の北の横の壁 */
    *cell_x = x;
    *cell_y = y;
    return true;
  }
  if (dir == DIR8_SOUTH_EAST) {
    *to = make_edge(true, x, y);              /* 南東：区画 (x, y) の南の横の壁 */
    *cell_x = x;
    *cell_y = y;
    return true;
  }
  if (dir == DIR8_WEST) {
    *to = make_edge(false, x - 1, y);         /* まっすぐ西：1 つ左の縦の壁 */
    *cell_x = x - 1;
    *cell_y = y;
    return true;
  }
  if (dir == DIR8_NORTH_WEST) {
    *to = make_edge(true, x - 1, y + 1);      /* 北西：区画 (x-1, y) の北の横の壁 */
    *cell_x = x - 1;
    *cell_y = y;
    return true;
  }
  if (dir == DIR8_SOUTH_WEST) {
    *to = make_edge(true, x - 1, y);          /* 南西：区画 (x-1, y) の南の横の壁 */
    *cell_x = x - 1;
    *cell_y = y;
    return true;
  }
  return false;  /* 北・南には進めない */
}

/* 反対の向き */
static dir8_t reverse_dir8(dir8_t dir)
{
  switch (dir) {
  case DIR8_NORTH:      return DIR8_SOUTH;
  case DIR8_NORTH_EAST: return DIR8_SOUTH_WEST;
  case DIR8_EAST:       return DIR8_WEST;
  case DIR8_SOUTH_EAST: return DIR8_NORTH_WEST;
  case DIR8_SOUTH:      return DIR8_NORTH;
  case DIR8_SOUTH_WEST: return DIR8_NORTH_EAST;
  case DIR8_WEST:       return DIR8_EAST;
  case DIR8_NORTH_WEST: return DIR8_SOUTH_EAST;
  default:              return DIR8_NONE;
  }
}

/* 縦横の向きか（斜めでないか） */
static bool is_orthogonal(dir8_t dir)
{
  if (dir == DIR8_NORTH || dir == DIR8_EAST || dir == DIR8_SOUTH || dir == DIR8_WEST) {
    return true;
  }
  return false;
}

/* 点を crossing の向きに通り抜けているときに、向き dir に進めるか。
 * 進めるのは、通り抜けた先の区画を通る 3 本（直進と、左右の斜め）だけ。
 * 例：横の壁を北向きに通っているなら、北・北東・北西 */
static bool is_forward_move(direction_t crossing, dir8_t dir)
{
  if (crossing == DIR_NORTH) {
    return (dir == DIR8_NORTH || dir == DIR8_NORTH_EAST || dir == DIR8_NORTH_WEST);
  }
  if (crossing == DIR_SOUTH) {
    return (dir == DIR8_SOUTH || dir == DIR8_SOUTH_EAST || dir == DIR8_SOUTH_WEST);
  }
  if (crossing == DIR_EAST) {
    return (dir == DIR8_EAST || dir == DIR8_NORTH_EAST || dir == DIR8_SOUTH_EAST);
  }
  return (dir == DIR8_WEST || dir == DIR8_NORTH_WEST || dir == DIR8_SOUTH_WEST);
}

/* 向き dir で点 edge に着いたとき、その点を通り抜ける向き */
static direction_t crossing_after(edge_t edge, dir8_t dir)
{
  if (edge.is_row) {
    if (dir == DIR8_NORTH || dir == DIR8_NORTH_EAST || dir == DIR8_NORTH_WEST) {
      return DIR_NORTH;
    }
    return DIR_SOUTH;
  }
  if (dir == DIR8_EAST || dir == DIR8_NORTH_EAST || dir == DIR8_SOUTH_EAST) {
    return DIR_EAST;
  }
  return DIR_WEST;
}

/* 点 edge を向き dir で通り抜けたときに入る区画 */
static void get_cell_beyond(edge_t edge, dir8_t dir, int *cell_x, int *cell_y)
{
  *cell_x = edge.x;
  *cell_y = edge.y;
  if (edge.is_row) {
    /* 南向きの成分があれば南側の区画 */
    if (dir == DIR8_SOUTH || dir == DIR8_SOUTH_EAST || dir == DIR8_SOUTH_WEST) {
      *cell_y = edge.y - 1;
    }
  } else {
    /* 西向きの成分があれば西側の区画 */
    if (dir == DIR8_WEST || dir == DIR8_NORTH_WEST || dir == DIR8_SOUTH_WEST) {
      *cell_x = edge.x - 1;
    }
  }
}

/* ===========================================================================
 * 探す
 * ======================================================================== */

/* 全部の状態を「まだたどり着いていない」にする */
static void reset_all(void)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE + 1; y++) {
      for (int i = 0; i < 2; i++) {
        s_row_cost[x][y][i] = COST_MAX;
        s_row_dir[x][y][i] = DIR8_NONE;
        s_row_in_queue[x][y][i] = false;
      }
    }
  }
  for (int x = 0; x < MAZE_SIZE + 1; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int i = 0; i < 2; i++) {
        s_col_cost[x][y][i] = COST_MAX;
        s_col_dir[x][y][i] = DIR8_NONE;
        s_col_in_queue[x][y][i] = false;
      }
    }
  }
  s_queue_read = 0;
  s_queue_write = 0;
}

/* 区画 (x, y) がゴールの範囲の中か */
static bool is_goal_cell(int x, int y, int goal_x, int goal_y, int goal_width, int goal_height)
{
  if (x < goal_x || x >= goal_x + goal_width) {
    return false;
  }
  if (y < goal_y || y >= goal_y + goal_height) {
    return false;
  }
  return true;
}

/* ゴールの区画 (x, y) の dir 側の辺が、ゴールの外との境目で通れるなら、
 * そこを「ゴールから外へ向かって通り抜ける」状態としてコスト 0 にし、待ち行列に入れる */
static void add_goal_entrance(int x, int y, direction_t dir, int goal_x, int goal_y, int goal_width, int goal_height,
                              bool unknown_as_wall)
{
  if (is_goal_cell(maze_next_x(x, dir), maze_next_y(y, dir), goal_x, goal_y, goal_width, goal_height)) {
    return;  /* 隣もゴール（ゴールの中の辺） */
  }

  edge_t edge;
  dir8_t outward;  /* ゴールから外へ向かう向き */
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
  if (!is_open(edge, unknown_as_wall)) {
    return;
  }

  state_t state = make_state(edge, dir);
  set_cost(state, 0);
  set_dir(state, outward);
  queue_push(state);
}

/* ゴールの入口（ゴールの範囲の外側との境目で、壁がないところ）を全部コスト 0 にする */
static void add_goal_entrances(int goal_x, int goal_y, int goal_width, int goal_height, bool unknown_as_wall)
{
  for (int x = goal_x; x < goal_x + goal_width; x++) {
    for (int y = goal_y; y < goal_y + goal_height; y++) {
      for (int i = 0; i < 4; i++) {
        add_goal_entrance(x, y, (direction_t)i, goal_x, goal_y, goal_width, goal_height, unknown_as_wall);
      }
    }
  }
}

/* 待ち行列が空になるまで、状態のコストを広げていく */
static void spread_costs(bool unknown_as_wall)
{
  state_t from;
  while (queue_pop(&from)) {
    for (int i = 0; i < 8; i++) {
      dir8_t dir = (dir8_t)i;
      if (!is_forward_move(from.crossing, dir)) {
        continue;  /* 通り抜けた先の区画を通らない向きには進めない */
      }
      edge_t to_edge;
      int cell_x;
      int cell_y;
      if (!move_from(from.edge, dir, &to_edge, &cell_x, &cell_y)) {
        continue;
      }
      if (!maze_is_inside(cell_x, cell_y)) {
        continue;
      }
      if (!is_open(to_edge, unknown_as_wall)) {
        continue;  /* 壁があって通れない */
      }

      /* この向きに進むときのコスト */
      uint16_t step_cost;
      if (dir == get_dir(from)) {
        step_cost = COST_CONTINUE;
      } else if (is_orthogonal(dir)) {
        step_cost = COST_STRAIGHT;
      } else {
        step_cost = COST_DIAGONAL;
      }

      state_t to = make_state(to_edge, crossing_after(to_edge, dir));
      uint16_t new_cost = get_cost(from) + step_cost;
      if (new_cost < get_cost(to)) {
        set_cost(to, new_cost);
        set_dir(to, dir);
        queue_push(to);  /* コストが下がったので、その先ももう一度調べる */
      }
    }
  }
}

/* 道に区画を 1 つ足す */
static void route_add(int x, int y)
{
  if (s_route_length >= ROUTE_CELL_MAX) {
    return;
  }
  s_route_x[s_route_length] = (uint8_t)x;
  s_route_y[s_route_length] = (uint8_t)y;
  s_route_length = s_route_length + 1;
}

/* スタートからゴールへの道を探す。見つかったら true。
 * 結果は dijkstra_get_route_x/y() で、スタート (0,0) からゴールの最初の区画まで順に読める。
 * unknown_as_wall：まだ見ていない壁を「ある」とみなすか（最短走行では true） */
bool dijkstra_find_route(int goal_x, int goal_y, int goal_width, int goal_height, bool unknown_as_wall)
{
  s_route_length = 0;

  reset_all();
  add_goal_entrances(goal_x, goal_y, goal_width, goal_height, unknown_as_wall);
  spread_costs(unknown_as_wall);

  /* スタートの区画 (0,0) の北の壁の点から、ゴール側へたどる。
   * 探すときはゴールから外へ広げたので、この点はゴール側から見ると南向きに通り抜けている */
  state_t state = make_state(make_edge(true, 0, 1), DIR_SOUTH);
  if (get_cost(state) == COST_MAX) {
    return false;  /* ゴールにたどり着けない */
  }

  s_route_cost = get_cost(state);
  route_add(0, 0);
  dir8_t move_dir = DIR8_NORTH;  /* スタートの区画は北へ抜ける */

  while (get_cost(state) != 0) {
    if (s_route_length >= ROUTE_CELL_MAX) {
      return false;  /* 長すぎる（普通は起きない） */
    }
    /* この状態にはゴール側から get_dir の向きで来たので、ゴールへはその反対向きに進む */
    dir8_t came_dir = get_dir(state);
    move_dir = reverse_dir8(came_dir);
    edge_t next_edge;
    int cell_x;
    int cell_y;
    if (!move_from(state.edge, move_dir, &next_edge, &cell_x, &cell_y)) {
      return false;  /* 向きの記録がおかしい（普通は起きない） */
    }
    route_add(cell_x, cell_y);
    /* 1 つ前（ゴール側）の状態。ゴール側から見ると、came_dir の向きで next_edge を通り抜けていた */
    state = make_state(next_edge, crossing_after(next_edge, came_dir));
  }

  /* 最後の点（ゴールの入口）を通り抜けた先が、ゴールの最初の区画 */
  int goal_cell_x;
  int goal_cell_y;
  get_cell_beyond(state.edge, move_dir, &goal_cell_x, &goal_cell_y);
  route_add(goal_cell_x, goal_cell_y);
  return true;
}

/* 見つけた道の区画の数 */
int dijkstra_get_route_length(void)
{
  return s_route_length;
}

/* 見つけた道のコストの合計（スタートの区画の北の壁からゴールの入口まで）。道の速さを比べるのに使う */
int dijkstra_get_route_cost(void)
{
  return s_route_cost;
}

/* 見つけた道の index 番目（0 がスタート）の区画の x */
int dijkstra_get_route_x(int index)
{
  return s_route_x[index];
}

/* 見つけた道の index 番目の区画の y */
int dijkstra_get_route_y(int index)
{
  return s_route_y[index];
}
