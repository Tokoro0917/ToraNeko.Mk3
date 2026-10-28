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
#define DIJKSTRA_TABLE_LENGTH 40  /* 走行時間のコストの表の長さ（区間の 1〜39 歩目。もっと長い区間は 39 歩目の値） */

bool dijkstra_find_route(int goal_x, int goal_y, int goal_width, int goal_height, bool unknown_as_wall);
int dijkstra_get_route_length(void);
int dijkstra_get_route_cost(void);
int dijkstra_get_route_x(int index);
int dijkstra_get_route_y(int index);

/* コストの選び方（dijkstra.c の先頭のコメント）。何も呼ばなければ 3 つの数のコスト */
void dijkstra_set_time_cost_small_turn(float turn_speed_mm_s, float max_speed_mm_s, float accel_mm_s2);
void dijkstra_set_time_cost_diagonal(float turn_speed_mm_s, float max_speed_mm_s, float accel_mm_s2,
                                     float diag_max_speed_mm_s, float diag_accel_mm_s2);
void dijkstra_set_step_cost(void);
bool dijkstra_is_time_cost(void);
int dijkstra_get_orth_step_cost(int step);
int dijkstra_get_diag_step_cost(int step);

#endif /* MAZE_DIJKSTRA_H */
