/*
 * maze_map.h
 * 迷路の壁の情報（どこに壁があるか、まだ見ていないか）を持っておくところ
 *
 * ■ 座標と向き
 *   - 区画は (x, y) で表す。スタートが (0, 0)。x は右（東）、y は上（北）に増える
 *   - スタートでは北を向いている
 *   - 向きは DIR_NORTH（北）、DIR_EAST（東）、DIR_SOUTH（南）、DIR_WEST（西）
 *
 * このファイルは HAL もモータも使わない。PC 上でもそのまま動く。
 */
#ifndef MAZE_MAZE_MAP_H
#define MAZE_MAZE_MAP_H

#include <stdbool.h>

#define MAZE_SIZE 32  /* 迷路の大きさ（ハーフサイズは 32×32 区画） */

/* 向き */
typedef enum {
  DIR_NORTH = 0,
  DIR_EAST  = 1,
  DIR_SOUTH = 2,
  DIR_WEST  = 3,
} direction_t;

/* 壁の状態 */
typedef enum {
  WALL_UNKNOWN = 0,  /* まだ見ていない */
  WALL_NONE    = 1,  /* 壁がないことを確認した */
  WALL_EXIST   = 2,  /* 壁があることを確認した */
} wall_state_t;

void maze_map_init(void);

void maze_map_set_wall(int x, int y, direction_t dir, wall_state_t state);
wall_state_t maze_map_get_wall(int x, int y, direction_t dir);
void maze_map_record_walls(int x, int y, direction_t dir, bool wall_front, bool wall_left, bool wall_right);
bool maze_map_is_visited(int x, int y);
void maze_map_set_visited(int x, int y, bool visited);

/* フェイルセーフで止まったときに戻す、壁の記録の回数（MODES.md の 5）。
 * ふつうは 5 区画ぶん。U ターンで同じ区画に 2 回入ると、その区画は 2 回と数える */
#define MAZE_UNDO_CELL_COUNT 5

void maze_map_undo_recent_cells(void);
void maze_map_clear_recent_cells(void);

bool maze_is_inside(int x, int y);
bool maze_is_in_area(int x, int y, int area_x, int area_y, int width, int height);
direction_t maze_direction_between(int from_x, int from_y, int to_x, int to_y);
int maze_next_x(int x, direction_t dir);
int maze_next_y(int y, direction_t dir);
direction_t maze_turn_left(direction_t dir);
direction_t maze_turn_right(direction_t dir);
direction_t maze_turn_back(direction_t dir);

#endif /* MAZE_MAZE_MAP_H */
