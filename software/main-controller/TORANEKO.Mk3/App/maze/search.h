/*
 * search.h
 * 探索（足立法）：歩数マップを作って、次にどちらへ進むかを決める
 *
 * ■ 使い方（sys/explore.c から、1 区画進むたびに）
 *   1. maze_map_record_walls() で、センサで見た壁を記録する
 *   2. search_is_in_target() で、目的地に着いたかを調べる
 *   3. search_update_step_map() で、歩数マップを作り直す
 *   4. search_decide_action() で、次の動き（直進・左・右・Uターン）を決める
 *
 * ■ 未知壁探索（KOGUMA のモード0 No.7）
 * ゴールに着いたあと、search_find_unknown_wall() で「最短経路の上にある、まだ見ていない壁」を探し、
 * その壁の手前の区画を目的地にして、壁を見るまで上と同じように走る。
 * 見ていない壁がなくなったら（search_find_unknown_wall() が false）、その場で止まり、
 * 止まってから path_make_shortest() で最短経路を計算する。そのあとスタートに戻る。
 */
#ifndef MAZE_SEARCH_H
#define MAZE_SEARCH_H

#include <stdbool.h>
#include <stdint.h>

#include "maze/maze_map.h"

#define STEP_MAX 9999  /* 歩数マップで「たどり着けない」を表す値 */

/* 次の動き */
typedef enum {
  ACTION_FORWARD,     /* 直進 */
  ACTION_TURN_LEFT,   /* 左に曲がる */
  ACTION_TURN_RIGHT,  /* 右に曲がる */
  ACTION_UTURN,       /* Uターン */
  ACTION_STOP,        /* どこにも進めない（目的地にたどり着けない） */
} action_t;

void search_set_target(int x, int y, int width, int height);
bool search_is_in_target(int x, int y);
void search_update_step_map(void);
uint16_t search_get_step(int x, int y);
action_t search_decide_action(int x, int y, direction_t dir);
bool search_find_unknown_wall(int goal_x, int goal_y, int goal_width, int goal_height,
                               int *cell_x, int *cell_y, direction_t *wall_dir);

#endif /* MAZE_SEARCH_H */
