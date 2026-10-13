/*
 * dijkstra.h
 * ダイクストラ法で、スタートからゴールまでの「速く走れる道」を探す
 *
 * 結果は「通る区画の並び」（スタート (0,0) からゴールの最初の区画まで）。
 * それを走る動き（直進・大回り・斜め）にするのは path.c の path_make_shortest()。
 */
#ifndef MAZE_DIJKSTRA_H
#define MAZE_DIJKSTRA_H

#include <stdbool.h>

#include "maze/maze_map.h"

#define ROUTE_CELL_MAX (MAZE_SIZE * MAZE_SIZE)  /* 道に入る区画の数の上限 */

bool dijkstra_find_route(int goal_x, int goal_y, int goal_width, int goal_height, bool unknown_as_wall);
int dijkstra_get_route_length(void);
int dijkstra_get_route_x(int index);
int dijkstra_get_route_y(int index);

#endif /* MAZE_DIJKSTRA_H */
