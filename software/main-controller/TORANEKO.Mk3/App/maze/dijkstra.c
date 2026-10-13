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
 * ■ 探し方
 * ゴールの入口の点をコスト 0 にして、そこから広げていく。点のコストが下がったら、
 * その点を「もう一度調べる」待ち行列に入れ直す（KOGUMA と同じ。SPFA と呼ばれる方法）。
 * 最後に、スタートの区画の北の壁の点から、コストが下がる向きに逆にたどるとゴールに着く。
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

/* 横の壁の点の情報 [x][y] */
static uint16_t s_row_cost[MAZE_SIZE][MAZE_SIZE + 1];
static dir8_t s_row_dir[MAZE_SIZE][MAZE_SIZE + 1];      /* その点に来たときの向き（ゴール側から見て） */
static bool s_row_in_queue[MAZE_SIZE][MAZE_SIZE + 1];   /* 待ち行列に入っているか */
/* 縦の壁の点の情報 [x][y] */
static uint16_t s_col_cost[MAZE_SIZE + 1][MAZE_SIZE];
static dir8_t s_col_dir[MAZE_SIZE + 1][MAZE_SIZE];
static bool s_col_in_queue[MAZE_SIZE + 1][MAZE_SIZE];

/* 待ち行列（輪のように使う。1 つの点は同時に 1 回しか入らないので、点の数あれば足りる） */
#define QUEUE_SIZE (2 * (MAZE_SIZE + 1) * (MAZE_SIZE + 1))
static edge_t s_queue[QUEUE_SIZE];
static int s_queue_read = 0;
static int s_queue_write = 0;

/* 見つけた道（通る区画の並び） */
static uint8_t s_route_x[ROUTE_CELL_MAX];  /* 区画の座標は 0〜31 なので 1 バイトで足りる */
static uint8_t s_route_y[ROUTE_CELL_MAX];
static int s_route_length = 0;
static int s_route_cost = 0;  /* 見つけた道のコストの合計 */

/* ===========================================================================
 * 点の情報の読み書き（横の壁か縦の壁かで配列が違うので、ここでまとめて振り分ける）
 * ======================================================================== */

static uint16_t get_cost(edge_t edge)
{
  if (edge.is_row) {
    return s_row_cost[edge.x][edge.y];
  }
  return s_col_cost[edge.x][edge.y];
}

static void set_cost(edge_t edge, uint16_t cost)
{
  if (edge.is_row) {
    s_row_cost[edge.x][edge.y] = cost;
  } else {
    s_col_cost[edge.x][edge.y] = cost;
  }
}

static dir8_t get_dir(edge_t edge)
{
  if (edge.is_row) {
    return s_row_dir[edge.x][edge.y];
  }
  return s_col_dir[edge.x][edge.y];
}

static void set_dir(edge_t edge, dir8_t dir)
{
  if (edge.is_row) {
    s_row_dir[edge.x][edge.y] = dir;
  } else {
    s_col_dir[edge.x][edge.y] = dir;
  }
}

static bool get_in_queue(edge_t edge)
{
  if (edge.is_row) {
    return s_row_in_queue[edge.x][edge.y];
  }
  return s_col_in_queue[edge.x][edge.y];
}

static void set_in_queue(edge_t edge, bool in_queue)
{
  if (edge.is_row) {
    s_row_in_queue[edge.x][edge.y] = in_queue;
  } else {
    s_col_in_queue[edge.x][edge.y] = in_queue;
  }
}

/* 待ち行列に入れる（もう入っていれば何もしない） */
static void queue_push(edge_t edge)
{
  if (get_in_queue(edge)) {
    return;
  }
  set_in_queue(edge, true);
  s_queue[s_queue_write] = edge;
  s_queue_write = s_queue_write + 1;
  if (s_queue_write >= QUEUE_SIZE) {
    s_queue_write = 0;
  }
}

/* 待ち行列から取り出す。空なら false */
static bool queue_pop(edge_t *edge)
{
  if (s_queue_read == s_queue_write) {
    return false;
  }
  *edge = s_queue[s_queue_read];
  s_queue_read = s_queue_read + 1;
  if (s_queue_read >= QUEUE_SIZE) {
    s_queue_read = 0;
  }
  set_in_queue(*edge, false);
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

/* 全部の点を「まだたどり着いていない」にする */
static void reset_all(void)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE + 1; y++) {
      s_row_cost[x][y] = COST_MAX;
      s_row_dir[x][y] = DIR8_NONE;
      s_row_in_queue[x][y] = false;
    }
  }
  for (int x = 0; x < MAZE_SIZE + 1; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      s_col_cost[x][y] = COST_MAX;
      s_col_dir[x][y] = DIR8_NONE;
      s_col_in_queue[x][y] = false;
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

/* ゴールの入口（ゴールの範囲の外側との境目で、壁がないところ）をコスト 0 にして待ち行列に入れる */
static void add_goal_entrances(int goal_x, int goal_y, int goal_width, int goal_height, bool unknown_as_wall)
{
  for (int x = goal_x; x < goal_x + goal_width; x++) {
    for (int y = goal_y; y < goal_y + goal_height; y++) {
      /* ゴールの区画 (x, y) の 4 つの辺のうち、隣がゴールの外のもの */
      edge_t sides[4];
      bool outside[4];
      sides[0] = make_edge(true, x, y + 1);   /* 北 */
      outside[0] = !is_goal_cell(x, y + 1, goal_x, goal_y, goal_width, goal_height);
      sides[1] = make_edge(true, x, y);       /* 南 */
      outside[1] = !is_goal_cell(x, y - 1, goal_x, goal_y, goal_width, goal_height);
      sides[2] = make_edge(false, x + 1, y);  /* 東 */
      outside[2] = !is_goal_cell(x + 1, y, goal_x, goal_y, goal_width, goal_height);
      sides[3] = make_edge(false, x, y);      /* 西 */
      outside[3] = !is_goal_cell(x - 1, y, goal_x, goal_y, goal_width, goal_height);

      for (int i = 0; i < 4; i++) {
        if (outside[i] && is_open(sides[i], unknown_as_wall)) {
          set_cost(sides[i], 0);
          set_dir(sides[i], DIR8_NONE);
          queue_push(sides[i]);
        }
      }
    }
  }
}

/* 待ち行列が空になるまで、点のコストを広げていく */
static void spread_costs(bool unknown_as_wall)
{
  edge_t from;
  while (queue_pop(&from)) {
    for (int i = 0; i < 8; i++) {
      dir8_t dir = (dir8_t)i;
      edge_t to;
      int cell_x;
      int cell_y;
      if (!move_from(from, dir, &to, &cell_x, &cell_y)) {
        continue;  /* この種類の点からは進めない向き */
      }
      if (!maze_is_inside(cell_x, cell_y)) {
        continue;
      }
      if (!is_open(to, unknown_as_wall)) {
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

  /* スタートの区画 (0,0) の北の壁の点から、ゴール側へたどる */
  edge_t edge = make_edge(true, 0, 1);
  if (get_cost(edge) == COST_MAX) {
    return false;  /* ゴールにたどり着けない */
  }

  s_route_cost = get_cost(edge);
  route_add(0, 0);
  dir8_t move_dir = DIR8_NORTH;  /* スタートの区画は北へ抜ける */

  while (get_cost(edge) != 0) {
    if (s_route_length >= ROUTE_CELL_MAX) {
      return false;  /* 長すぎる（普通は起きない） */
    }
    /* この点にはゴール側から get_dir の向きで来たので、ゴールへはその反対向きに進む */
    move_dir = reverse_dir8(get_dir(edge));
    edge_t next;
    int cell_x;
    int cell_y;
    if (!move_from(edge, move_dir, &next, &cell_x, &cell_y)) {
      return false;  /* 向きの記録がおかしい（普通は起きない） */
    }
    route_add(cell_x, cell_y);
    edge = next;
  }

  /* 最後の点（ゴールの入口）を通り抜けた先が、ゴールの最初の区画 */
  int goal_cell_x;
  int goal_cell_y;
  get_cell_beyond(edge, move_dir, &goal_cell_x, &goal_cell_y);
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
