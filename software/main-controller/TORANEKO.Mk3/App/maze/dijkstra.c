/*
 * dijkstra.c
 * ダイクストラ法で、スタートからゴールまでの「速く走れる道」を探す（KOGUMA の Maze_Dijkstra_Calculation）
 *
 * ■ 点（ノード）は「壁の真ん中」
 * 区画の真ん中ではなく、区画と区画の境目（壁がある・ないの位置）の真ん中を点にする。
 * 点から点へは、区画を 1 つ通って 8 方向（縦横 4 つ + 斜め 4 つ）に進める。これを「1 ホップ」と呼ぶ。
 *   例：横の壁の点から北へ進むと、区画を縦に通り抜けて、1 つ上の横の壁の点へ
 *       横の壁の点から北東へ進むと、区画の中で右に曲がって、その区画の東の縦の壁の点へ
 * 斜めのホップは、その区画で小回りのターンを 1 回すること（経路の 1 つの区画）と同じ。
 * 斜めに続けて進むと、ジグザグに曲がる道（＝斜め走行）になる。
 *
 * ■ 状態は「点」と「その点を通るときの向き（8 方向）」の組
 * 横の壁の点なら 北・南・北東・北西・南東・南西 の 6 つ、縦の壁の点なら 東・西・北東・南東・北西・南西 の 6 つ。
 * 状態ごとに「スタートからその状態までのコスト（かかる時間の目安）」を持つ。
 *
 * ■ コストの数え方は 2 とおり（どちらを使うかは dijkstra_set_... で選ぶ）
 * (A) 区間ごとに数える（何も呼ばない・dijkstra_set_step_cost()・dijkstra_set_time_cost_small_turn()）
 *     「同じ向きに k ホップ続けて進む区間」を 1 回の移動にして、区間のコストを足していく。
 *     区間のコストは直前の向きに関係なく決まるので、状態は「点と、その点を通り抜ける向き（縦横の 2 つ）」だけでよい
 *     （6 つの向きのうち、縦横の向きの所だけを使う）。
 *       - 3 つの数のコスト（KOGUMA の ST_COST 5・DIAG_COST 3・CON_COST 2）：
 *           区間のコスト = 最初の 1 歩（縦横なら COST_STRAIGHT、斜めなら COST_DIAGONAL）＋ COST_CONTINUE ×（k − 1）
 *         ゴールに縦横の直線のまま入る区間だけは、全部 COST_CONTINUE にする（KOGUMA と同じ）。
 *         探索（maze/search.c の未知壁探索）で使う。
 *       - 走行時間の表（小回りだけで走る最短走行。モード1 No.1〜3）：区間の k 歩目のコストを表にして、
 *           区間のコスト = 表[1] + 表[2] + … + 表[k]  ≒  その区間を走る時間（単位 5ms）
 *         長い直線ほど速度が乗って 1 歩あたりが速くなることを表せる。表の k 歩目は k−1 歩目より大きくしない
 *         （大きいと「区間を途中で切って 2 つにする」ほうが安く見えて、本当の最短にならない。KOGUMA で確かめたこと）
 * (B) ターンごとの時間で数える（dijkstra_set_time_cost_diagonal()。大回り・斜めで走る最短走行。モード1 No.4・5）
 *     KOGUMA の #100（2026-10-27）を持ち込んだもの。1 回の移動を「直線 ＋ ターン 1 つ」にして、
 *     その直線とターンを本当に走る時間（単位 2ms）を足していく。ターンの種類ごとの時間の違いまで表せる。
 *     状態の向きは「この点から進む直線の向き」（縦横か斜め）。移動とホップの並び（path.c の大回り・斜めへの置き換えと同じ）：
 *       大回り 90°    縦横 k ホップ → 斜め 1（45°）                 → 縦横の状態（90° 曲がった向き）
 *       大回り 180°   縦横 k ホップ → 斜め 1（45°）→ 斜め 1（さらに 90°） → 縦横の状態（反対向き）
 *       斜め入り 45°  縦横 k ホップ → 斜め 1（45°）                 → 斜めの状態（その斜めの向き）
 *       斜め入り 135° 縦横 k ホップ → 斜め 1（45°）→ 斜め 1（さらに 90°） → 斜めの状態
 *       斜め出 45°    斜め m ホップ（最後のホップがターン）          → 縦横の状態
 *       斜め出 135°   斜め m ホップ → 斜め 1（90° 向きを変える）       → 縦横の状態
 *       V90°          斜め m ホップ → 斜め 1（90° 向きを変える）       → 斜めの状態（新しい斜めの向き）
 *     縦横の直線の長さは 2k − 2 半区画（前後のターンが半区画ずつ使う）。k は 1 以上（スタートだけは 0 でもよい）。
 *     斜めの直線は m − 1 個（斜めに入ったターンと出るターンが 1 ホップずつ使う）。
 *     こうすると、見つけた道のコストは、その道を path.c で動きにしたときの走る時間の見積もり（dijkstra_count_path_cost()）と同じになる。
 *
 * ■ 探し方
 *   1. スタートの状態（スタートの区画 (0,0) の北の壁を北向きに通るところ）をコスト 0 にする
 *   2. 「コストが下がった状態」に印（dirty）を付けておき、印の付いた状態から、できる移動を全部調べて、
 *      行き先の状態のコストを下げられたら書き込んで印を付ける。印がなくなるまで、全部の状態を何周も見る
 *      （KOGUMA は待ち行列を使うが、RAM を減らすため、待ち行列のかわりに印だけにした）
 *   3. 移動の途中でゴールの区画に入ったら、そこで道の終わり。一番安いものを覚えておく。
 *      コストは進むほど増えるので、その一番安いものより高い状態からは調べない
 *   4. 道をたどる：ゴールに入った移動から、「1 つ前の状態のコスト ＋ 移動のコスト ＝ 今の状態のコスト」になる
 *      1 つ前の状態を探して、スタートまで逆にたどる（KOGUMA と同じ。1 つ前の状態を覚えないので RAM が少なくて済む）
 *
 * ■ 区画の並び
 * 道は、スタートの区画 (0,0)、ホップで通った区画（順に）、ゴールの最初の区画、の並びにする。
 */
#include "maze/dijkstra.h"

#include <math.h>
#include <stdint.h>

#include "config/run_params.h"
#include "config/turn_params.h"

/* 3 つの数のコスト。KOGUMA の ST_COST, DIAG_COST, CON_COST と同じ値 */
#define COST_STRAIGHT 5
#define COST_DIAGONAL 3
#define COST_CONTINUE 2
#define COST_MAX 60000  /* まだたどり着いていない（コストは 2 バイトの整数なので、これより大きくしない） */

#define SMALL_TURN_COST_PER_SECOND 200.0f  /* (A) の走行時間の表の単位：コスト 1 が 5ms */
#define TURN_COST_PER_SECOND 500.0f        /* (B) ターンごとの時間の単位：コスト 1 が 2ms（60000 で 120 秒） */

#define STRAIGHT_HOP_MAX (2 * MAZE_SIZE)       /* 1 回の移動で、同じ向きに進むホップの数の上限（斜めのジグザグでも足りる） */
#define ORTH_HALF_MAX (2 * MAZE_SIZE + 4)      /* (B) の縦横の直線の表の長さ [半区画] */
#define DIAG_STRAIGHT_MAX (2 * MAZE_SIZE + 1)  /* (B) の斜めの直線の表の長さ [斜めの 1 つぶん] */

/* コストの数え方（先頭のコメント） */
typedef enum {
  COST_MODEL_STEP,        /* (A) 3 つの数 */
  COST_MODEL_SMALL_TURN,  /* (A) 走行時間の表（小回りだけで走る） */
  COST_MODEL_TURN,        /* (B) ターンごとの時間（大回り・斜めで走る） */
} cost_model_t;

static cost_model_t s_cost_model = COST_MODEL_STEP;

/* 8 方向。北から右回りの順に並べてある（1 つ進むと右に 45°） */
typedef enum {
  DIR8_NORTH,
  DIR8_NORTH_EAST,
  DIR8_EAST,
  DIR8_SOUTH_EAST,
  DIR8_SOUTH,
  DIR8_SOUTH_WEST,
  DIR8_WEST,
  DIR8_NORTH_WEST,
  DIR8_NONE,  /* 向きがない（ターンのホップがないときなど） */
} dir8_t;

/* (B) のターンの種類 */
typedef enum {
  TURN_BIG90,
  TURN_BIG180,
  TURN_IN45,
  TURN_IN135,
  TURN_OUT45,
  TURN_OUT135,
  TURN_V90,
  TURN_NONE,  /* ターンしない（縦横の直線のままゴールに入る） */
} turn_kind_t;

#define TURN_KIND_COUNT 7  /* TURN_NONE を除いたターンの種類の数 */

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

/* 状態：点と、その点を通るときの向き */
typedef struct {
  edge_t edge;
  dir8_t dir;
} state_t;

/* 状態を作る */
static state_t make_state(edge_t edge, dir8_t dir)
{
  state_t state;
  state.edge = edge;
  state.dir = dir;
  return state;
}

/* 同じ状態か */
static bool is_same_state(state_t a, state_t b)
{
  if (a.edge.is_row != b.edge.is_row || a.edge.x != b.edge.x || a.edge.y != b.edge.y) {
    return false;
  }
  return a.dir == b.dir;
}

/* スタートの状態：スタートの区画 (0,0) の北の壁を、北向きに通るところ */
static state_t start_state(void)
{
  return make_state(make_edge(true, 0, 1), DIR8_NORTH);
}

/* 状態ごとのコストと印 [x][y][向きの番号]。向きの番号は direction_slot() で決める。
 * どちらも、探すあいだだけ使う（RAM は 2 つで約 38KB） */
#define SLOT_COUNT 6
static uint16_t s_row_cost[MAZE_SIZE][MAZE_SIZE + 1][SLOT_COUNT];   /* 横の壁の点 */
static uint16_t s_col_cost[MAZE_SIZE + 1][MAZE_SIZE][SLOT_COUNT];   /* 縦の壁の点 */
static bool s_row_dirty[MAZE_SIZE][MAZE_SIZE + 1][SLOT_COUNT];      /* コストが下がって、まだその先を調べていない */
static bool s_col_dirty[MAZE_SIZE + 1][MAZE_SIZE][SLOT_COUNT];

/* 探すときの条件（dijkstra_find_route() の引数を、何度も渡さなくて済むように覚えておく） */
static bool s_unknown_as_wall = true;
static int s_find_goal_x = 0;
static int s_find_goal_y = 0;
static int s_find_goal_width = 1;
static int s_find_goal_height = 1;

/* ゴールに入った移動のうち、一番安いもの（道をたどるときの出発点） */
static uint32_t s_best_cost = COST_MAX;  /* そのコスト（スタートからゴールまで） */
static state_t s_best_from;              /* その移動を始めた状態 */
static edge_t s_best_end;                /* ゴールに入ったときの点 */
static dir8_t s_best_end_dir;            /* ゴールに入ったホップの向き */
static dir8_t s_best_straight_dir;       /* 移動の直線の向き */
static int s_best_hops;                  /* 移動の直線のホップの数 */
static dir8_t s_best_turn_dir1;          /* 直線のあとのターンのホップの向き（なければ DIR8_NONE） */
static dir8_t s_best_turn_dir2;          /* その次のホップの向き（なければ DIR8_NONE） */

/* 見つけた道（通る区画の並び） */
static uint8_t s_route_x[ROUTE_CELL_MAX];  /* 区画の座標は 0〜31 なので 1 バイトで足りる */
static uint8_t s_route_y[ROUTE_CELL_MAX];
static int s_route_length = 0;
static int s_route_cost = 0;  /* 見つけた道のコストの合計（見つからなかったら 0） */

/* ===========================================================================
 * 向き
 * ======================================================================== */

/* 向き dir を、右に 45° × eighths だけ回した向き（eighths がマイナスなら左に回す） */
static dir8_t rotate_dir8(dir8_t dir, int eighths)
{
  int index = (int)dir + eighths;
  while (index >= 8) {
    index = index - 8;
  }
  while (index < 0) {
    index = index + 8;
  }
  return (dir8_t)index;
}

/* 反対の向き */
static dir8_t reverse_dir8(dir8_t dir)
{
  if (dir == DIR8_NONE) {
    return DIR8_NONE;
  }
  return rotate_dir8(dir, 4);
}

/* 縦横の向きか（斜めでないか） */
static bool is_orthogonal(dir8_t dir)
{
  if (dir == DIR8_NORTH || dir == DIR8_EAST || dir == DIR8_SOUTH || dir == DIR8_WEST) {
    return true;
  }
  return false;
}

/* 縦横 4 方向（direction_t）を 8 方向にする */
static dir8_t dir8_from_direction(direction_t dir)
{
  if (dir == DIR_NORTH) {
    return DIR8_NORTH;
  }
  if (dir == DIR_EAST) {
    return DIR8_EAST;
  }
  if (dir == DIR_SOUTH) {
    return DIR8_SOUTH;
  }
  return DIR8_WEST;
}

/* ===========================================================================
 * 状態のコストと印の読み書き（横の壁か縦の壁かで配列が違うので、ここでまとめて振り分ける）
 * ======================================================================== */

/* 状態の向きを、配列の向きの番号（0〜5）にする。その点で使えない向きなら -1
 *   横の壁：北 0、南 1、北東 2、北西 3、南東 4、南西 5
 *   縦の壁：東 0、西 1、北東 2、南東 3、北西 4、南西 5 */
static int direction_slot(bool is_row, dir8_t dir)
{
  if (dir == DIR8_NORTH_EAST) {
    return 2;
  }
  if (dir == DIR8_SOUTH_WEST) {
    return 5;
  }
  if (is_row) {
    if (dir == DIR8_NORTH) {
      return 0;
    }
    if (dir == DIR8_SOUTH) {
      return 1;
    }
    if (dir == DIR8_NORTH_WEST) {
      return 3;
    }
    if (dir == DIR8_SOUTH_EAST) {
      return 4;
    }
    return -1;  /* 横の壁の点を、東・西に通ることはない */
  }
  if (dir == DIR8_EAST) {
    return 0;
  }
  if (dir == DIR8_WEST) {
    return 1;
  }
  if (dir == DIR8_SOUTH_EAST) {
    return 3;
  }
  if (dir == DIR8_NORTH_WEST) {
    return 4;
  }
  return -1;  /* 縦の壁の点を、北・南に通ることはない */
}

/* direction_slot() の逆：配列の向きの番号を、向きにする */
static dir8_t slot_direction(bool is_row, int slot)
{
  static const dir8_t row_dirs[SLOT_COUNT] = {
    DIR8_NORTH, DIR8_SOUTH, DIR8_NORTH_EAST, DIR8_NORTH_WEST, DIR8_SOUTH_EAST, DIR8_SOUTH_WEST,
  };
  static const dir8_t col_dirs[SLOT_COUNT] = {
    DIR8_EAST, DIR8_WEST, DIR8_NORTH_EAST, DIR8_SOUTH_EAST, DIR8_NORTH_WEST, DIR8_SOUTH_WEST,
  };
  if (is_row) {
    return row_dirs[slot];
  }
  return col_dirs[slot];
}

/* その状態が配列にあるか（点が配列の範囲の中で、向きがその点で使えるもの） */
static bool is_valid_state(state_t state)
{
  if (direction_slot(state.edge.is_row, state.dir) < 0) {
    return false;
  }
  if (state.edge.x < 0 || state.edge.y < 0) {
    return false;
  }
  if (state.edge.is_row) {
    return state.edge.x < MAZE_SIZE && state.edge.y <= MAZE_SIZE;
  }
  return state.edge.x <= MAZE_SIZE && state.edge.y < MAZE_SIZE;
}

/* 状態のコスト（スタートからその状態までの、かかる時間の目安）を読む。配列にない状態は COST_MAX */
static uint16_t get_cost(state_t state)
{
  if (!is_valid_state(state)) {
    return COST_MAX;
  }
  int slot = direction_slot(state.edge.is_row, state.dir);
  if (state.edge.is_row) {
    return s_row_cost[state.edge.x][state.edge.y][slot];
  }
  return s_col_cost[state.edge.x][state.edge.y][slot];
}

/* 状態のコストを書いて、印を付ける（その先をあとで調べる） */
static void set_cost_and_mark(state_t state, uint16_t cost)
{
  int slot = direction_slot(state.edge.is_row, state.dir);
  if (state.edge.is_row) {
    s_row_cost[state.edge.x][state.edge.y][slot] = cost;
    s_row_dirty[state.edge.x][state.edge.y][slot] = true;
  } else {
    s_col_cost[state.edge.x][state.edge.y][slot] = cost;
    s_col_dirty[state.edge.x][state.edge.y][slot] = true;
  }
}

/* 状態のコストを、cost のほうが安ければ書き換える */
static void relax(state_t state, uint32_t cost)
{
  if (cost >= COST_MAX || cost >= s_best_cost) {
    return;  /* 長すぎる道（(A) の 3 つの数なら 60000、(B) なら 120 秒より長い）、またはゴールまでの一番速い道より遅い */
  }
  if (!is_valid_state(state)) {
    return;
  }
  if (cost < get_cost(state)) {
    set_cost_and_mark(state, (uint16_t)cost);
  }
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

/* 向き dir で点 edge を通るとき、その点を通り抜ける向き（縦横） */
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

/* 点 edge を向き dir で通ると、ゴールの区画に入るか */
static bool is_goal_entry(edge_t edge, dir8_t dir)
{
  int cell_x;
  int cell_y;
  get_cell_beyond(edge, dir, &cell_x, &cell_y);
  return maze_is_in_area(cell_x, cell_y, s_find_goal_x, s_find_goal_y, s_find_goal_width, s_find_goal_height);
}

/* 点 from から向き dir に 1 ホップ進む。通る区画が迷路の中で、行き先の点を通れれば、行き先を to に入れて true */
static bool hop_forward(edge_t from, dir8_t dir, edge_t *to)
{
  int cell_x;
  int cell_y;
  if (!move_from(from, dir, to, &cell_x, &cell_y)) {
    return false;
  }
  if (!maze_is_inside(cell_x, cell_y)) {
    return false;
  }
  return is_open(*to, s_unknown_as_wall);
}

/* 点 to に向き dir のホップで来たときの、1 つ前の点を from に、通った区画を cell_x, cell_y に入れる（壁は見ない）。
 * 点から点へのホップは、反対向きに進むと元の点に戻る（同じ区画を通る）ので、反対向きに 1 ホップ進めばよい */
static bool hop_backward(edge_t to, dir8_t dir, edge_t *from, int *cell_x, int *cell_y)
{
  if (!move_from(to, reverse_dir8(dir), from, cell_x, cell_y)) {
    return false;
  }
  return maze_is_inside(*cell_x, *cell_y);
}

/* 点 edge が、向き dir のホップの「途中の行き先」になれるか（通れて、ゴールの入口でない）。
 * 道をたどるときに使う（進むときは、ゴールの入口に着いたら、そこで移動を終えているので） */
static bool can_pass_through(edge_t edge, dir8_t dir)
{
  if (!is_open(edge, s_unknown_as_wall)) {
    return false;
  }
  return !is_goal_entry(edge, dir);
}

/* (A) で使う状態：点 edge を向き dir で通るときの、「通り抜ける向き（縦横）」の状態 */
static state_t crossing_state(edge_t edge, dir8_t dir)
{
  return make_state(edge, dir8_from_direction(crossing_after(edge, dir)));
}

/* (B) で使う：斜めの向き dir で点 edge に着いたとき、その点から 90° 向きを変えて進める、もう 1 つの斜めの向き。
 * 例：横の壁に北東で着いたら（北向きに通り抜ける）、北西 */
static dir8_t other_diagonal(edge_t edge, dir8_t dir)
{
  direction_t crossing = crossing_after(edge, dir);
  dir8_t left = rotate_dir8(dir, -2);
  if (is_forward_move(crossing, left)) {
    return left;
  }
  return rotate_dir8(dir, 2);
}

/* ===========================================================================
 * 走る時間の見積もり（(A) の表と (B) のコストに使う）
 * ======================================================================== */

/* 距離 distance_mm を、速さ start_mm_s から（最高 max_speed_mm_s まで上げて）end_mm_s まで変えて走る時間 [s]。
 * 加速と減速は同じ加速度 accel_mm_s2 の台形（ctrl/trajectory.c と同じ形） */
static float straight_time_s(float distance_mm, float start_mm_s, float end_mm_s, float max_speed_mm_s,
                             float accel_mm_s2)
{
  if (distance_mm <= 0.0f) {
    return 0.0f;
  }
  /* 一番速いところの速さ。最高速まで上げて戻す距離が distance_mm より長ければ、ちょうど使い切る速さまで */
  float peak_mm_s = max_speed_mm_s;
  float up_mm = (peak_mm_s * peak_mm_s - start_mm_s * start_mm_s) / (2.0f * accel_mm_s2);
  float down_mm = (peak_mm_s * peak_mm_s - end_mm_s * end_mm_s) / (2.0f * accel_mm_s2);
  if (up_mm + down_mm > distance_mm) {
    peak_mm_s = sqrtf(accel_mm_s2 * distance_mm + (start_mm_s * start_mm_s + end_mm_s * end_mm_s) / 2.0f);
  }
  /* 距離が短すぎて、始めか終わりの速さに届かないとき（ふつうは起きない）は、その速さを一番速いところにする */
  if (peak_mm_s < start_mm_s) {
    peak_mm_s = start_mm_s;
  }
  if (peak_mm_s < end_mm_s) {
    peak_mm_s = end_mm_s;
  }
  up_mm = (peak_mm_s * peak_mm_s - start_mm_s * start_mm_s) / (2.0f * accel_mm_s2);
  down_mm = (peak_mm_s * peak_mm_s - end_mm_s * end_mm_s) / (2.0f * accel_mm_s2);
  float time_s = (peak_mm_s - start_mm_s) / accel_mm_s2 + (peak_mm_s - end_mm_s) / accel_mm_s2;
  float cruise_mm = distance_mm - up_mm - down_mm;  /* 一番速い速さのまま走る距離 */
  if (cruise_mm > 0.0f) {
    time_s = time_s + cruise_mm / peak_mm_s;
  }
  return time_s;
}

/* 時間 [s] を、コストの単位 cost_per_second に直す（四捨五入。COST_MAX より大きくしない） */
static uint16_t time_to_cost(float time_s, float cost_per_second)
{
  float cost = time_s * cost_per_second + 0.5f;
  if (cost >= (float)COST_MAX) {
    return COST_MAX;
  }
  if (cost < 0.0f) {
    return 0;
  }
  return (uint16_t)cost;
}

/* ===========================================================================
 * (A) 区間ごとのコスト
 * ======================================================================== */

static uint16_t s_orth_step_cost[DIJKSTRA_TABLE_LENGTH];  /* 走行時間の表：縦横の区間の k 歩目のコスト（[0] は使わない） */
static uint16_t s_diag_step_cost[DIJKSTRA_TABLE_LENGTH];  /* 斜めの区間の k 歩目のコスト */

/* 向き dir の区間の step 歩目（2 歩目から）のコスト */
static uint16_t get_next_step_cost(dir8_t dir, int step)
{
  if (s_cost_model == COST_MODEL_STEP) {
    return COST_CONTINUE;
  }
  if (step >= DIJKSTRA_TABLE_LENGTH) {
    step = DIJKSTRA_TABLE_LENGTH - 1;  /* 表より長い区間は、表の最後の値 */
  }
  if (is_orthogonal(dir)) {
    return s_orth_step_cost[step];
  }
  return s_diag_step_cost[step];
}

/* 向き dir の区間の最初の 1 歩のコスト */
static uint16_t get_first_step_cost(dir8_t dir)
{
  if (s_cost_model != COST_MODEL_STEP) {
    return get_next_step_cost(dir, 1);
  }
  if (is_orthogonal(dir)) {
    return COST_STRAIGHT;
  }
  return COST_DIAGONAL;
}

/* 表 table を作る。k 歩目のコスト = （k 歩の時間 − k−1 歩までのコストの合計）。
 * 1 より小さくしない。k−1 歩目より大きくしない（先頭のコメント） */
static void make_step_table(uint16_t table[], float step_mm, float speed_mm_s, float max_speed_mm_s, float accel_mm_s2)
{
  int sum = 0;
  table[0] = 0;
  for (int k = 1; k < DIJKSTRA_TABLE_LENGTH; k++) {
    float time_s = straight_time_s(step_mm * k, speed_mm_s, speed_mm_s, max_speed_mm_s, accel_mm_s2);
    int step_cost = (int)(time_s * SMALL_TURN_COST_PER_SECOND + 0.5f) - sum;
    if (step_cost < 1) {
      step_cost = 1;
    }
    if (k > 1 && step_cost > table[k - 1]) {
      step_cost = table[k - 1];
    }
    table[k] = (uint16_t)step_cost;
    sum = sum + step_cost;
  }
}

/* ===========================================================================
 * (B) ターンごとの時間のコスト
 * ======================================================================== */

static float s_turn_speed_mm_s = FAST_TURN_SPEED_MM_S;  /* ターンの速さ */
static float s_max_speed_mm_s = FAST_TURN_SPEED_MM_S;   /* 縦横の直線の最高速 */
static float s_accel_mm_s2 = 1000.0f;                   /* 縦横の直線の加速度 */
static uint16_t s_orth_straight_cost[ORTH_HALF_MAX + 1];  /* ターンの速さから上げて戻す縦横の直線 [半区画] のコスト */
static uint16_t s_orth_goal_cost[ORTH_HALF_MAX + 1];      /* ターンの速さから、縦横の直線 [半区画] でゴールの区画のまん中に止まる */
static uint16_t s_diag_straight_cost[DIAG_STRAIGHT_MAX + 1];  /* ターンの速さから上げて戻す斜めの直線 [斜めの 1 つぶん] */
static uint16_t s_turn_cost[TURN_KIND_COUNT];             /* ターンの速さで、ターン 1 つ */
static uint16_t s_turn_stop_cost;                         /* ターンでゴールの区画のまん中に着いてから止まる */

/* ターン kind を、速さ speed_mm_s で走る時間 [s]。
 * ターンは「前距離 → スラローム（角速度は台形）→ 後距離」（config/turn_params.h）。
 * 値は、そのターンの速さ（*_SPEED_MM_S）でのもの。それより遅いときは、同じ形のまま時間だけが
 *（そのターンの速さ ÷ 速さ）倍になる（ctrl/run.c の fast_turn()） */
static float turn_time_s(turn_kind_t kind, float speed_mm_s)
{
  float angle_deg = 90.0f;
  float design_speed_mm_s = BIG90_SPEED_MM_S;
  float omega_deg_s = BIG90_OMEGA_DEG_S;
  float alpha_deg_s2 = BIG90_ALPHA_DEG_S2;
  float straight_mm = BIG90_PRE_MM + BIG90_POST_MM + BIG90_PRE_ADJUST_MM + BIG90_POST_ADJUST_MM;
  if (kind == TURN_BIG180) {
    angle_deg = 180.0f;
    design_speed_mm_s = BIG180_SPEED_MM_S;
    omega_deg_s = BIG180_OMEGA_DEG_S;
    alpha_deg_s2 = BIG180_ALPHA_DEG_S2;
    straight_mm = BIG180_PRE_MM + BIG180_POST_MM + BIG180_PRE_ADJUST_MM + BIG180_POST_ADJUST_MM;
  } else if (kind == TURN_IN45) {
    angle_deg = 45.0f;
    design_speed_mm_s = IN45_SPEED_MM_S;
    omega_deg_s = IN45_OMEGA_DEG_S;
    alpha_deg_s2 = IN45_ALPHA_DEG_S2;
    straight_mm = IN45_PRE_MM + IN45_POST_MM + IN45_PRE_ADJUST_MM + IN45_POST_ADJUST_MM;
  } else if (kind == TURN_IN135) {
    angle_deg = 135.0f;
    design_speed_mm_s = IN135_SPEED_MM_S;
    omega_deg_s = IN135_OMEGA_DEG_S;
    alpha_deg_s2 = IN135_ALPHA_DEG_S2;
    straight_mm = IN135_PRE_MM + IN135_POST_MM + IN135_PRE_ADJUST_MM + IN135_POST_ADJUST_MM;
  } else if (kind == TURN_OUT45) {
    angle_deg = 45.0f;
    design_speed_mm_s = OUT45_SPEED_MM_S;
    omega_deg_s = OUT45_OMEGA_DEG_S;
    alpha_deg_s2 = OUT45_ALPHA_DEG_S2;
    straight_mm = OUT45_PRE_MM + OUT45_POST_MM + OUT45_PRE_ADJUST_MM + OUT45_POST_ADJUST_MM;
  } else if (kind == TURN_OUT135) {
    angle_deg = 135.0f;
    design_speed_mm_s = OUT135_SPEED_MM_S;
    omega_deg_s = OUT135_OMEGA_DEG_S;
    alpha_deg_s2 = OUT135_ALPHA_DEG_S2;
    straight_mm = OUT135_PRE_MM + OUT135_POST_MM + OUT135_PRE_ADJUST_MM + OUT135_POST_ADJUST_MM;
  } else if (kind == TURN_V90) {
    angle_deg = 90.0f;
    design_speed_mm_s = V90_SPEED_MM_S;
    omega_deg_s = V90_OMEGA_DEG_S;
    alpha_deg_s2 = V90_ALPHA_DEG_S2;
    straight_mm = V90_PRE_MM + V90_POST_MM + V90_PRE_ADJUST_MM + V90_POST_ADJUST_MM;
  }

  /* そのターンの速さで回る時間。最高角速度まで上がれば「角度 ÷ 最高角速度 ＋ 最高角速度 ÷ 角加速度」、
   * 上がりきらなければ（三角形）「2 × √(角度 ÷ 角加速度)」 */
  float slalom_s;
  if (angle_deg >= omega_deg_s * omega_deg_s / alpha_deg_s2) {
    slalom_s = angle_deg / omega_deg_s + omega_deg_s / alpha_deg_s2;
  } else {
    slalom_s = 2.0f * sqrtf(angle_deg / alpha_deg_s2);
  }
  float time_at_design_speed_s = straight_mm / design_speed_mm_s + slalom_s;
  return time_at_design_speed_s * design_speed_mm_s / speed_mm_s;
}

/* 縦横の直線 half_count [半区画] のコスト（ターンの速さから上げて、またターンの速さに戻す） */
static uint32_t orth_straight_cost(int half_count)
{
  if (half_count > ORTH_HALF_MAX) {
    half_count = ORTH_HALF_MAX;  /* ふつうは起きない */
  }
  return s_orth_straight_cost[half_count];
}

/* 縦横の直線 half_count [半区画] で、ゴールの区画のまん中に止まるコスト（ターンの速さから） */
static uint32_t orth_goal_cost(int half_count)
{
  if (half_count > ORTH_HALF_MAX) {
    half_count = ORTH_HALF_MAX;
  }
  return s_orth_goal_cost[half_count];
}

/* 斜めの直線 count [斜めの 1 つぶん] のコスト */
static uint32_t diag_straight_cost(int count)
{
  if (count > DIAG_STRAIGHT_MAX) {
    count = DIAG_STRAIGHT_MAX;
  }
  return s_diag_straight_cost[count];
}

/* スタートからの最初の移動のコスト（その場で計算する）。
 * 止まっているところから、縦横の直線 half_count [半区画] を走ってから、ターン kind をする。
 * kind が TURN_NONE なら、その直線でゴールの区画のまん中に止まる。stops_after_turn が true なら、ターンのあとゴールで止まる。
 * 走り方は sys/shortest.c と同じ：最初の直線はスタートの置き方の分だけ短く、終わりの速さは上げられる速さまで
 * （ターンの速さまで上げきれなければ、その速さでターンする） */
static uint32_t start_move_cost(int half_count, turn_kind_t kind, bool stops_after_turn)
{
  float distance_mm = half_count * HALF_CELL_MM - (HALF_CELL_MM - START_OFFSET_MM);
  if (distance_mm < 0.0f) {
    distance_mm = 0.0f;
  }
  if (kind == TURN_NONE) {
    return time_to_cost(straight_time_s(distance_mm, 0.0f, 0.0f, s_max_speed_mm_s, s_accel_mm_s2),
                        TURN_COST_PER_SECOND);
  }
  float end_mm_s = sqrtf(2.0f * s_accel_mm_s2 * distance_mm);
  if (end_mm_s > s_turn_speed_mm_s) {
    end_mm_s = s_turn_speed_mm_s;
  }
  if (end_mm_s < 1.0f) {
    end_mm_s = 1.0f;  /* 0 で割らないように（ふつうは起きない） */
  }
  float time_s = straight_time_s(distance_mm, 0.0f, end_mm_s, s_max_speed_mm_s, s_accel_mm_s2);
  time_s = time_s + turn_time_s(kind, end_mm_s);
  if (stops_after_turn) {
    time_s = time_s + end_mm_s / SHORTEST_STOP_ACCEL_MM_S2;  /* その速さから止まる時間 */
  }
  return time_to_cost(time_s, TURN_COST_PER_SECOND);
}

/* 縦横の状態から、縦横に k ホップ進んでから、ターン kind をする移動のコスト。
 * is_start はスタートの状態からか、stops_after_turn はターンでゴールに入って止まるか */
static uint32_t orth_move_cost(int k, bool is_start, turn_kind_t kind, bool stops_after_turn)
{
  if (is_start) {
    /* スタートの区画の 2 半区画 + k ホップ − ターンが使う半区画 */
    return start_move_cost(2 * k + 1, kind, stops_after_turn);
  }
  /* k ホップ − 前のターンと次のターンが使う半区画ずつ */
  uint32_t cost = orth_straight_cost(2 * k - 2) + s_turn_cost[kind];
  if (stops_after_turn) {
    cost = cost + s_turn_stop_cost;
  }
  return cost;
}

/* 縦横の状態から、縦横に hops ホップ進んで、そのままゴールの区画のまん中に止まる移動のコスト */
static uint32_t orth_goal_move_cost(int hops, bool is_start)
{
  if (is_start) {
    /* スタートの区画の 2 半区画 + hops ホップ + ゴールの区画の半分 */
    return start_move_cost(2 * hops + 3, TURN_NONE, false);
  }
  /* hops ホップ − 前のターンが使う半区画 + ゴールの区画の半分 */
  return orth_goal_cost(2 * hops);
}

/* 斜めの状態から、斜めに m ホップ進んでから（最後のホップはターンが使う）、ターン kind をする移動のコスト */
static uint32_t diag_move_cost(int m, turn_kind_t kind, bool stops_after_turn)
{
  uint32_t cost = diag_straight_cost(m - 1) + s_turn_cost[kind];
  if (stops_after_turn) {
    cost = cost + s_turn_stop_cost;
  }
  return cost;
}

/* ===========================================================================
 * 探す：ゴールに入った移動を覚える
 * ======================================================================== */

/* ゴールに入った移動を、今までの一番安いものより安ければ覚える。
 * from から、向き straight_dir に hops ホップ、続いてターンのホップ turn_dir1・turn_dir2（なければ DIR8_NONE）。
 * end はゴールに入った点、end_dir はそのホップの向き */
static void record_goal(uint32_t cost, state_t from, dir8_t straight_dir, int hops, dir8_t turn_dir1, dir8_t turn_dir2,
                        edge_t end, dir8_t end_dir)
{
  if (cost >= s_best_cost || cost >= COST_MAX) {
    return;
  }
  s_best_cost = cost;
  s_best_from = from;
  s_best_end = end;
  s_best_end_dir = end_dir;
  s_best_straight_dir = straight_dir;
  s_best_hops = hops;
  s_best_turn_dir1 = turn_dir1;
  s_best_turn_dir2 = turn_dir2;
}

/* (A) で、ゴールに入った区間（向き dir に hops ホップ）のコストに直す。
 * 3 つの数のコストで縦横の直線のままゴールに入るときは、全部 COST_CONTINUE にする（KOGUMA と同じ） */
static uint32_t segment_goal_cost(uint32_t base_cost, uint32_t segment_cost, dir8_t dir, int hops)
{
  if (s_cost_model == COST_MODEL_STEP && is_orthogonal(dir)) {
    return base_cost + (uint32_t)(COST_CONTINUE * hops);
  }
  return base_cost + segment_cost;
}

/* ===========================================================================
 * 探す：(A) 区間ごとに進める
 * ======================================================================== */

/* 状態 from から、向き dir に 1 歩、2 歩、… と続けて進む区間を調べ、着いた状態のコストを下げられたら書き込む。
 * 壁に当たったら、そこから先へは進めないので終わる。ゴールに入ったら、そこで道の終わり */
static void spread_segment(state_t from, dir8_t dir)
{
  edge_t edge = from.edge;
  uint32_t base_cost = get_cost(from);
  /* コストの足し算は 4 バイトの整数でする（2 バイトのままだと、60000 を超えたときに小さな数に戻ってしまう） */
  uint32_t segment_cost = get_first_step_cost(dir);

  for (int hops = 1; hops <= STRAIGHT_HOP_MAX; hops++) {
    edge_t next_edge;
    if (!hop_forward(edge, dir, &next_edge)) {
      return;  /* 壁があって通れない */
    }
    if (is_goal_entry(next_edge, dir)) {
      record_goal(segment_goal_cost(base_cost, segment_cost, dir, hops), from, dir, hops, DIR8_NONE, DIR8_NONE,
                  next_edge, dir);
      return;
    }
    relax(crossing_state(next_edge, dir), base_cost + segment_cost);

    /* 同じ向きにもう 1 歩 */
    edge = next_edge;
    segment_cost = segment_cost + get_next_step_cost(dir, hops + 1);
  }
}

/* (A) 状態 from から進める区間を全部調べる。進めるのは、通り抜けた先の区画を通る 3 本の向き */
static void expand_segments(state_t from)
{
  direction_t crossing = crossing_after(from.edge, from.dir);
  for (int i = 0; i < 8; i++) {
    dir8_t dir = (dir8_t)i;
    if (is_forward_move(crossing, dir)) {
      spread_segment(from, dir);
    }
  }
}

/* ===========================================================================
 * 探す：(B) 直線 ＋ ターンで進める
 * ======================================================================== */

/* (B) 縦横の状態 from から、縦横に k ホップ進んで点 node に着いたところで、side の側（+1 なら右、-1 なら左）に
 * 入りのターン（大回り 90°・180°、斜め入り 45°・135°）をする移動を調べる */
static void try_entry_turns(state_t from, edge_t node, int k, bool is_start, int side)
{
  uint32_t base_cost = get_cost(from);
  dir8_t first_dir = rotate_dir8(from.dir, side);  /* 45° 曲がるホップ */
  edge_t after_first;
  if (!hop_forward(node, first_dir, &after_first)) {
    return;
  }
  if (is_goal_entry(after_first, first_dir)) {
    /* 大回り 90° でゴールに入る */
    record_goal(base_cost + orth_move_cost(k, is_start, TURN_BIG90, true), from, from.dir, k, first_dir, DIR8_NONE,
                after_first, first_dir);
    return;
  }
  relax(make_state(after_first, rotate_dir8(from.dir, 2 * side)), base_cost + orth_move_cost(k, is_start, TURN_BIG90, false));
  relax(make_state(after_first, first_dir), base_cost + orth_move_cost(k, is_start, TURN_IN45, false));

  dir8_t second_dir = rotate_dir8(first_dir, 2 * side);  /* さらに 90° 曲がるホップ */
  edge_t after_second;
  if (!hop_forward(after_first, second_dir, &after_second)) {
    return;
  }
  if (is_goal_entry(after_second, second_dir)) {
    /* 大回り 180° でゴールに入る */
    record_goal(base_cost + orth_move_cost(k, is_start, TURN_BIG180, true), from, from.dir, k, first_dir, second_dir,
                after_second, second_dir);
    return;
  }
  relax(make_state(after_second, reverse_dir8(from.dir)), base_cost + orth_move_cost(k, is_start, TURN_BIG180, false));
  relax(make_state(after_second, second_dir), base_cost + orth_move_cost(k, is_start, TURN_IN135, false));
}

/* (B) 縦横の状態 from から、縦横の直線を k ホップ（スタート以外は 1 以上）進んでから入りのターンをする移動と、
 * 縦横の直線のままゴールに入る移動を、全部調べる */
static void expand_orth(state_t from)
{
  bool is_start = is_same_state(from, start_state());
  edge_t node = from.edge;  /* 直線を k ホップ進んだ点 */
  for (int k = 0; k <= STRAIGHT_HOP_MAX; k++) {
    if (k >= 1 || is_start) {
      try_entry_turns(from, node, k, is_start, 1);
      try_entry_turns(from, node, k, is_start, -1);
    }
    edge_t next_node;
    if (!hop_forward(node, from.dir, &next_node)) {
      return;
    }
    if (is_goal_entry(next_node, from.dir)) {
      /* 縦横の直線のままゴールに入る */
      record_goal(get_cost(from) + orth_goal_move_cost(k + 1, is_start), from, from.dir, k + 1, DIR8_NONE, DIR8_NONE,
                  next_node, from.dir);
      return;
    }
    node = next_node;
  }
}

/* (B) 斜めの状態 from から、斜めの直線を m ホップ（1 以上）進んでから、出るターン（斜め出 45°・135°、V90°）をする移動を全部調べる */
static void expand_diag(state_t from)
{
  uint32_t base_cost = get_cost(from);
  edge_t node = from.edge;
  for (int m = 1; m <= STRAIGHT_HOP_MAX; m++) {
    edge_t next_node;
    if (!hop_forward(node, from.dir, &next_node)) {
      return;
    }
    if (is_goal_entry(next_node, from.dir)) {
      /* 斜め出 45° でゴールに入る（最後のホップがターン） */
      record_goal(base_cost + diag_move_cost(m, TURN_OUT45, true), from, from.dir, m, DIR8_NONE, DIR8_NONE,
                  next_node, from.dir);
      return;
    }
    node = next_node;

    /* 斜め出 45°：最後のホップがターンで、この点から縦横に進む */
    relax(crossing_state(node, from.dir), base_cost + diag_move_cost(m, TURN_OUT45, false));

    /* 90° 向きを変えるホップ：斜め出 135° か V90° */
    dir8_t turn_dir = other_diagonal(node, from.dir);
    edge_t after_turn;
    if (hop_forward(node, turn_dir, &after_turn)) {
      if (is_goal_entry(after_turn, turn_dir)) {
        record_goal(base_cost + diag_move_cost(m, TURN_OUT135, true), from, from.dir, m, turn_dir, DIR8_NONE,
                    after_turn, turn_dir);
      } else {
        relax(crossing_state(after_turn, turn_dir), base_cost + diag_move_cost(m, TURN_OUT135, false));
        relax(make_state(after_turn, turn_dir), base_cost + diag_move_cost(m, TURN_V90, false));
      }
    }
  }
}

/* ===========================================================================
 * 探す：全体
 * ======================================================================== */

/* 状態 from から、できる移動を全部調べる */
static void expand(state_t from)
{
  if (get_cost(from) >= s_best_cost) {
    return;  /* ゴールまでの一番速い道より遅い（コストは進むほど増えるので、この先も速くならない） */
  }
  if (s_cost_model != COST_MODEL_TURN) {
    expand_segments(from);
  } else if (is_orthogonal(from.dir)) {
    expand_orth(from);
  } else {
    expand_diag(from);
  }
}

/* 全部の状態を「まだたどり着いていない」にして、印を消す */
static void reset_all(void)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE + 1; y++) {
      for (int slot = 0; slot < SLOT_COUNT; slot++) {
        s_row_cost[x][y][slot] = COST_MAX;
        s_row_dirty[x][y][slot] = false;
      }
    }
  }
  for (int x = 0; x < MAZE_SIZE + 1; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int slot = 0; slot < SLOT_COUNT; slot++) {
        s_col_cost[x][y][slot] = COST_MAX;
        s_col_dirty[x][y][slot] = false;
      }
    }
  }
  s_best_cost = COST_MAX;
}

/* 印の付いた状態を 1 周ぶん調べる（印は消す）。1 つでも調べたら true */
static bool expand_marked_states(void)
{
  bool expanded = false;
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE + 1; y++) {
      for (int slot = 0; slot < SLOT_COUNT; slot++) {
        if (s_row_dirty[x][y][slot]) {
          s_row_dirty[x][y][slot] = false;
          expand(make_state(make_edge(true, x, y), slot_direction(true, slot)));
          expanded = true;
        }
      }
    }
  }
  for (int x = 0; x < MAZE_SIZE + 1; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int slot = 0; slot < SLOT_COUNT; slot++) {
        if (s_col_dirty[x][y][slot]) {
          s_col_dirty[x][y][slot] = false;
          expand(make_state(make_edge(false, x, y), slot_direction(false, slot)));
          expanded = true;
        }
      }
    }
  }
  return expanded;
}

/* 印がなくなるまで、状態のコストを広げていく */
static void spread_costs(void)
{
  while (expand_marked_states()) {
    /* 印の付いた状態がある限り、もう 1 周 */
  }
}

/* ===========================================================================
 * 道をたどる
 * ======================================================================== */

/* 道に区画を 1 つ足す（たどるときはゴール側から足していき、最後に並びを逆にする） */
static bool route_add(int x, int y)
{
  if (s_route_length >= ROUTE_CELL_MAX) {
    return false;  /* 長すぎる（普通は起きない） */
  }
  s_route_x[s_route_length] = (uint8_t)x;
  s_route_y[s_route_length] = (uint8_t)y;
  s_route_length = s_route_length + 1;
  return true;
}

/* 点 *node に向き dir のホップで count 回続けて来たとして、通った区画を（ゴール側から）道に足し、*node を戻った点にする */
static bool add_hops_backward(edge_t *node, dir8_t dir, int count)
{
  for (int i = 0; i < count; i++) {
    edge_t previous;
    int cell_x;
    int cell_y;
    if (!hop_backward(*node, dir, &previous, &cell_x, &cell_y)) {
      return false;
    }
    if (!route_add(cell_x, cell_y)) {
      return false;
    }
    *node = previous;
  }
  return true;
}

/* 1 回の移動（向き straight_dir の直線 hops ホップ → ターンのホップ turn_dir1 → turn_dir2）で点 *node に着いたとして、
 * 通った区画を（ゴール側から）道に足し、*node を移動の始まりの点にする */
static bool add_move_backward(edge_t *node, dir8_t straight_dir, int hops, dir8_t turn_dir1, dir8_t turn_dir2)
{
  if (turn_dir2 != DIR8_NONE && !add_hops_backward(node, turn_dir2, 1)) {
    return false;
  }
  if (turn_dir1 != DIR8_NONE && !add_hops_backward(node, turn_dir1, 1)) {
    return false;
  }
  return add_hops_backward(node, straight_dir, hops);
}

/* (A) 状態 to に来た 1 つ前の区間を探し、その区間で通った区画を道に足す。見つかれば 1 つ前の状態を previous に入れて true */
static bool step_back_segment(state_t to, state_t *previous)
{
  direction_t crossing = crossing_after(to.edge, to.dir);
  uint16_t target_cost = get_cost(to);
  for (int i = 0; i < 8; i++) {
    dir8_t dir = (dir8_t)i;
    if (!is_forward_move(crossing, dir)) {
      continue;  /* この点にこの向きで来ることはない */
    }
    /* 向き dir に 1 ホップずつ戻りながら、「その点の状態のコスト ＋ 区間のコスト ＝ target_cost」になるものを探す */
    edge_t node = to.edge;
    uint32_t segment_cost = 0;
    for (int hops = 1; hops <= STRAIGHT_HOP_MAX; hops++) {
      if (hops >= 2 && !can_pass_through(node, dir)) {
        break;  /* 途中の点を通れない */
      }
      edge_t back_node;
      int cell_x;
      int cell_y;
      if (!hop_backward(node, dir, &back_node, &cell_x, &cell_y)) {
        break;
      }
      if (hops == 1) {
        segment_cost = get_first_step_cost(dir);
      } else {
        segment_cost = segment_cost + get_next_step_cost(dir, hops);
      }
      state_t candidate = crossing_state(back_node, dir);
      uint16_t candidate_cost = get_cost(candidate);
      if (candidate_cost < COST_MAX && candidate_cost + segment_cost == target_cost) {
        edge_t start_node = to.edge;
        if (!add_move_backward(&start_node, dir, hops, DIR8_NONE, DIR8_NONE)) {
          return false;
        }
        *previous = candidate;
        return true;
      }
      node = back_node;
    }
  }
  return false;
}

/* (B) ターンの始まりの点 turn_start から、直線の向き straight_dir に 1 ホップずつ戻りながら、
 * 「その点の状態のコスト ＋ この移動（直線 ＋ ターン kind）のコスト ＝ target_cost」になる 1 つ前の状態を探す。
 * 見つかれば、その状態を previous に、直線のホップの数を hops に入れて true */
static bool find_previous(edge_t turn_start, dir8_t straight_dir, turn_kind_t kind, uint16_t target_cost,
                          state_t *previous, int *hops)
{
  bool is_orth = is_orthogonal(straight_dir);
  edge_t node = turn_start;
  for (int count = 0; count <= STRAIGHT_HOP_MAX; count++) {
    state_t candidate = make_state(node, straight_dir);
    bool is_start = is_same_state(candidate, start_state());
    uint16_t candidate_cost = get_cost(candidate);
    /* 縦横の直線は 1 ホップ以上（スタートだけ 0 でもよい）、斜めの直線は 1 ホップ以上 */
    bool length_ok = (count >= 1 || (is_orth && is_start));
    if (length_ok && candidate_cost < COST_MAX) {
      uint32_t move_cost;
      if (is_orth) {
        move_cost = orth_move_cost(count, is_start, kind, false);
      } else {
        move_cost = diag_move_cost(count, kind, false);
      }
      if (candidate_cost + move_cost == target_cost) {
        *previous = candidate;
        *hops = count;
        return true;
      }
    }
    /* もう 1 ホップ戻る。node は直線のホップの行き先なので、通れて、ゴールの入口でないこと */
    if (!can_pass_through(node, straight_dir)) {
      return false;
    }
    edge_t back_node;
    int cell_x;
    int cell_y;
    if (!hop_backward(node, straight_dir, &back_node, &cell_x, &cell_y)) {
      return false;
    }
    node = back_node;
  }
  return false;
}

/* (B) 状態 to に、「向き straight_dir の直線 → ターン kind（ターンのホップ turn_dir1 → turn_dir2）」で来たかを調べる。
 * 来ていれば、その移動で通った区画を道に足し、1 つ前の状態を previous に入れて true */
static bool try_back_move(state_t to, dir8_t straight_dir, dir8_t turn_dir1, dir8_t turn_dir2, turn_kind_t kind,
                          state_t *previous)
{
  /* ターンのホップを戻って、ターンの始まりの点にする */
  edge_t node = to.edge;
  int cell_x;
  int cell_y;
  if (turn_dir2 != DIR8_NONE) {
    if (!hop_backward(node, turn_dir2, &node, &cell_x, &cell_y)) {
      return false;
    }
    if (!can_pass_through(node, turn_dir1)) {
      return false;  /* ターンの途中の点を通れない */
    }
  }
  if (turn_dir1 != DIR8_NONE) {
    if (!hop_backward(node, turn_dir1, &node, &cell_x, &cell_y)) {
      return false;
    }
  }

  int hops;
  if (!find_previous(node, straight_dir, kind, get_cost(to), previous, &hops)) {
    return false;
  }
  edge_t start_node = to.edge;
  return add_move_backward(&start_node, straight_dir, hops, turn_dir1, turn_dir2);
}

/* (B) 状態 to に来た 1 つ前の移動を探し、その移動で通った区画を道に足す。見つかれば 1 つ前の状態を previous に入れて true。
 * side が +1 なら右に曲がって、-1 なら左に曲がって来た場合（向きは先頭のコメントの表から計算した） */
static bool step_back_turn(state_t to, state_t *previous)
{
  dir8_t f = to.dir;
  for (int side = 1; side >= -1; side = side - 2) {
    if (is_orthogonal(f)) {
      /* 縦横の状態には、大回り 90°・180°、斜め出 45°・135° で来る */
      if (try_back_move(to, rotate_dir8(f, -2 * side), rotate_dir8(f, -side), DIR8_NONE, TURN_BIG90, previous)) {
        return true;
      }
      if (try_back_move(to, reverse_dir8(f), rotate_dir8(f, -3 * side), rotate_dir8(f, -side), TURN_BIG180, previous)) {
        return true;
      }
      if (try_back_move(to, rotate_dir8(f, -side), DIR8_NONE, DIR8_NONE, TURN_OUT45, previous)) {
        return true;
      }
      if (try_back_move(to, rotate_dir8(f, -3 * side), rotate_dir8(f, -side), DIR8_NONE, TURN_OUT135, previous)) {
        return true;
      }
    } else {
      /* 斜めの状態には、斜め入り 45°・135°、V90° で来る */
      if (try_back_move(to, rotate_dir8(f, -side), f, DIR8_NONE, TURN_IN45, previous)) {
        return true;
      }
      if (try_back_move(to, rotate_dir8(f, -3 * side), rotate_dir8(f, -2 * side), f, TURN_IN135, previous)) {
        return true;
      }
      if (try_back_move(to, rotate_dir8(f, -2 * side), f, DIR8_NONE, TURN_V90, previous)) {
        return true;
      }
    }
  }
  return false;
}

/* 道の並びを逆にする（ゴール側から足したので、スタート側からの順にする） */
static void reverse_route(void)
{
  int left = 0;
  int right = s_route_length - 1;
  while (left < right) {
    uint8_t x = s_route_x[left];
    uint8_t y = s_route_y[left];
    s_route_x[left] = s_route_x[right];
    s_route_y[left] = s_route_y[right];
    s_route_x[right] = x;
    s_route_y[right] = y;
    left = left + 1;
    right = right - 1;
  }
}

/* 途中まで作った道を捨てる（道が見つからなかったときは、長さもコストも 0 にしておく） */
static void forget_route(void)
{
  s_route_length = 0;
  s_route_cost = 0;
}

/* 一番安いゴールの移動から、スタートまで逆にたどって道を作る。たどれなければ false */
static bool trace_route(void)
{
  /* 1. ゴールの最初の区画と、ゴールに入った移動で通った区画 */
  int goal_cell_x;
  int goal_cell_y;
  get_cell_beyond(s_best_end, s_best_end_dir, &goal_cell_x, &goal_cell_y);
  if (!route_add(goal_cell_x, goal_cell_y)) {
    return false;
  }
  edge_t node = s_best_end;
  if (!add_move_backward(&node, s_best_straight_dir, s_best_hops, s_best_turn_dir1, s_best_turn_dir2)) {
    return false;
  }

  /* 2. 1 つ前の状態を順に探して、スタートの状態まで */
  state_t state = s_best_from;
  int guard = 0;
  while (!is_same_state(state, start_state())) {
    state_t previous;
    bool found;
    if (s_cost_model == COST_MODEL_TURN) {
      found = step_back_turn(state, &previous);
    } else {
      found = step_back_segment(state, &previous);
    }
    if (!found) {
      return false;  /* たどれない（普通は起きない） */
    }
    state = previous;
    guard = guard + 1;
    if (guard > ROUTE_CELL_MAX) {
      return false;
    }
  }

  /* 3. スタートの区画を足して、スタート側からの順にする */
  if (!route_add(0, 0)) {
    return false;
  }
  reverse_route();
  return true;
}

/* スタートからゴールへの道を探す。見つかったら true。
 * 結果は dijkstra_get_route_x/y() で、スタート (0,0) からゴールの最初の区画まで順に読める。
 * unknown_as_wall：まだ見ていない壁を「ある」とみなすか（最短走行では true） */
bool dijkstra_find_route(int goal_x, int goal_y, int goal_width, int goal_height, bool unknown_as_wall)
{
  forget_route();
  s_unknown_as_wall = unknown_as_wall;
  s_find_goal_x = goal_x;
  s_find_goal_y = goal_y;
  s_find_goal_width = goal_width;
  s_find_goal_height = goal_height;

  reset_all();
  state_t start = start_state();
  if (!is_open(start.edge, unknown_as_wall)) {
    return false;  /* スタートの区画から出られない */
  }
  if (is_goal_entry(start.edge, start.dir)) {
    return false;  /* ゴールがスタートのすぐ北の区画（ありえない置き方） */
  }
  set_cost_and_mark(start, 0);
  spread_costs();

  if (s_best_cost >= COST_MAX) {
    return false;  /* ゴールにたどり着けない */
  }
  if (!trace_route()) {
    forget_route();
    return false;
  }
  s_route_cost = (int)s_best_cost;
  return true;
}

/* ===========================================================================
 * コストの選び方
 * ======================================================================== */

/* (A) 走行時間の表を使う。曲がるところを全部小回り（ターンの速さ turn_speed_mm_s）で走る最短走行用。
 *   縦横の区間：1 歩 90mm を、ターンの速さから max_speed_mm_s まで上げて戻す（加速度 accel_mm_s2）
 *   斜めの区間：1 歩（ジグザグの 1 区画）を、ターンの速さのまま 90mm 走る（小回りを続けるので速度は上がらない） */
void dijkstra_set_time_cost_small_turn(float turn_speed_mm_s, float max_speed_mm_s, float accel_mm_s2)
{
  if (max_speed_mm_s < turn_speed_mm_s) {
    max_speed_mm_s = turn_speed_mm_s;
  }
  make_step_table(s_orth_step_cost, CELL_MM, turn_speed_mm_s, max_speed_mm_s, accel_mm_s2);
  make_step_table(s_diag_step_cost, CELL_MM, turn_speed_mm_s, turn_speed_mm_s, accel_mm_s2);
  s_cost_model = COST_MODEL_SMALL_TURN;
}

/* (B) ターンごとの時間を使う。大回り・斜めのターン（ターンの速さ turn_speed_mm_s）で走る最短走行用。
 *   縦横の直線：ターンの速さから max_speed_mm_s まで上げて戻す（加速度 accel_mm_s2）
 *   斜めの直線：1 つ DIAG_STEP_MM（63.6mm）を、ターンの速さから diag_max_speed_mm_s まで上げて戻す（diag_accel_mm_s2）
 *   ターン    ：config/turn_params.h のターンごとの速さ・前距離・後距離・最高角速度・角加速度から計算する（turn_time_s()）
 *   止まる    ：縦横の直線ならゴールの区画のまん中で止まるように、ターンでゴールに入ったら SHORTEST_STOP_ACCEL_MM_S2 で
 * 表を作る時間はほとんどかからない（PC で 1ms もかからない） */
void dijkstra_set_time_cost_diagonal(float turn_speed_mm_s, float max_speed_mm_s, float accel_mm_s2,
                                     float diag_max_speed_mm_s, float diag_accel_mm_s2)
{
  if (max_speed_mm_s < turn_speed_mm_s) {
    max_speed_mm_s = turn_speed_mm_s;
  }
  if (diag_max_speed_mm_s < turn_speed_mm_s) {
    diag_max_speed_mm_s = turn_speed_mm_s;
  }
  s_turn_speed_mm_s = turn_speed_mm_s;
  s_max_speed_mm_s = max_speed_mm_s;
  s_accel_mm_s2 = accel_mm_s2;

  for (int half_count = 0; half_count <= ORTH_HALF_MAX; half_count++) {
    float distance_mm = half_count * HALF_CELL_MM;
    s_orth_straight_cost[half_count] = time_to_cost(
        straight_time_s(distance_mm, turn_speed_mm_s, turn_speed_mm_s, max_speed_mm_s, accel_mm_s2), TURN_COST_PER_SECOND);
    s_orth_goal_cost[half_count] = time_to_cost(
        straight_time_s(distance_mm, turn_speed_mm_s, 0.0f, max_speed_mm_s, accel_mm_s2), TURN_COST_PER_SECOND);
  }
  for (int count = 0; count <= DIAG_STRAIGHT_MAX; count++) {
    float distance_mm = count * DIAG_STEP_MM;
    s_diag_straight_cost[count] = time_to_cost(
        straight_time_s(distance_mm, turn_speed_mm_s, turn_speed_mm_s, diag_max_speed_mm_s, diag_accel_mm_s2),
        TURN_COST_PER_SECOND);
  }
  for (int kind = 0; kind < TURN_KIND_COUNT; kind++) {
    s_turn_cost[kind] = time_to_cost(turn_time_s((turn_kind_t)kind, turn_speed_mm_s), TURN_COST_PER_SECOND);
  }
  s_turn_stop_cost = time_to_cost(turn_speed_mm_s / SHORTEST_STOP_ACCEL_MM_S2, TURN_COST_PER_SECOND);
  s_cost_model = COST_MODEL_TURN;
}

/* 3 つの数（COST_STRAIGHT・COST_DIAGONAL・COST_CONTINUE）のコストに戻す */
void dijkstra_set_step_cost(void)
{
  s_cost_model = COST_MODEL_STEP;
}

/* 走行時間のコスト（(A) の表か (B)）を使っているか */
bool dijkstra_is_time_cost(void)
{
  return s_cost_model != COST_MODEL_STEP;
}

/* (B) ターンごとの時間のコストを使っているか */
bool dijkstra_is_turn_cost(void)
{
  return s_cost_model == COST_MODEL_TURN;
}

/* (A) の表の、縦横・斜めの区間の k 歩目のコスト（PC のテストで、表を確かめるのに使う） */
int dijkstra_get_orth_step_cost(int step)
{
  return get_next_step_cost(DIR8_NORTH, step);
}

int dijkstra_get_diag_step_cost(int step)
{
  return get_next_step_cost(DIR8_NORTH_EAST, step);
}

/* ===========================================================================
 * 経路の時間の見積もり（(B)）
 * ======================================================================== */

/* 動き motion が、どのターンか。ターンでなければ TURN_NONE */
static turn_kind_t turn_kind_of(motion_t motion)
{
  if (motion == MOTION_BIG_TURN_LEFT_90 || motion == MOTION_BIG_TURN_RIGHT_90) {
    return TURN_BIG90;
  }
  if (motion == MOTION_BIG_TURN_LEFT_180 || motion == MOTION_BIG_TURN_RIGHT_180) {
    return TURN_BIG180;
  }
  if (motion == MOTION_DIAG_IN45_LEFT || motion == MOTION_DIAG_IN45_RIGHT) {
    return TURN_IN45;
  }
  if (motion == MOTION_DIAG_IN135_LEFT || motion == MOTION_DIAG_IN135_RIGHT) {
    return TURN_IN135;
  }
  if (motion == MOTION_DIAG_OUT45_LEFT || motion == MOTION_DIAG_OUT45_RIGHT) {
    return TURN_OUT45;
  }
  if (motion == MOTION_DIAG_OUT135_LEFT || motion == MOTION_DIAG_OUT135_RIGHT) {
    return TURN_OUT135;
  }
  if (motion == MOTION_DIAG_V90_LEFT || motion == MOTION_DIAG_V90_RIGHT) {
    return TURN_V90;
  }
  return TURN_NONE;
}

/* 最短経路（path.c の path_make_shortest() で作ったもの）を走る時間の見積もりを、(B) のコストで数える。
 * ダイクストラと同じ数え方なので、ダイクストラで見つけた道なら dijkstra_get_route_cost() と同じになる。
 * (B) を使っていないとき、経路の形がおかしいとき（小回りが残っているなど）は -1 */
int dijkstra_count_path_cost(const path_t *path)
{
  if (s_cost_model != COST_MODEL_TURN || path->count == 0 || path->steps[0].motion != MOTION_STRAIGHT) {
    return -1;
  }

  /* 1. 走り出し：最初の直進と、その次のターン（なければ、その直進でゴールに止まる） */
  int half_count = path->steps[0].half_count;
  if (path->count == 1) {
    return (int)start_move_cost(half_count, TURN_NONE, false);
  }
  turn_kind_t first_turn = turn_kind_of(path->steps[1].motion);
  if (first_turn == TURN_NONE) {
    return -1;
  }
  uint32_t total = start_move_cost(half_count, first_turn, path->count == 2);

  /* 2. そのあと：「直進か斜めの直進（なくてもよい）→ ターン」を順に。最後が直進なら、それでゴールに止まる */
  int i = 2;
  while (i < path->count) {
    motion_t motion = path->steps[i].motion;
    bool is_last = (i == path->count - 1);
    if (motion == MOTION_STRAIGHT && is_last) {
      total = total + orth_goal_cost(path->steps[i].half_count);
      i = i + 1;
      continue;
    }
    uint32_t straight_cost = 0;
    if (motion == MOTION_STRAIGHT) {
      straight_cost = orth_straight_cost(path->steps[i].half_count);
      i = i + 1;
    } else if (motion == MOTION_DIAG_STRAIGHT) {
      straight_cost = diag_straight_cost(path->steps[i].half_count);
      i = i + 1;
    }
    if (i >= path->count) {
      return -1;  /* 斜めの直進で終わっている（ありえない形） */
    }
    turn_kind_t turn = turn_kind_of(path->steps[i].motion);
    if (turn == TURN_NONE) {
      return -1;  /* 小回りなど、(B) で数えられない動き */
    }
    total = total + straight_cost + s_turn_cost[turn];
    if (i == path->count - 1) {
      total = total + s_turn_stop_cost;  /* ターンでゴールに入って止まる */
    }
    i = i + 1;
  }
  return (int)total;
}

/* 見つけた道の区画の数 */
int dijkstra_get_route_length(void)
{
  return s_route_length;
}

/* 見つけた道のコストの合計（スタートからゴールまで）。道の速さを比べるのに使う。道が見つからなかったときは 0。
 * (B) のときは、走る時間の見積もり（単位 2ms） */
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
