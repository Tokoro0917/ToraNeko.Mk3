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
 * ■ 未知壁探索（KOGUMA のモード0 No.6・No.7。ゴールに着いたあと、最短経路の上の見ていない壁を見に行く）
 * ダイクストラの計算は重い（数十 ms の見込み）ので、止まっているときだけ計算する（KOGUMA と同じ）。
 *   - 止まっているとき：search_plan_unknown_walls() でダイクストラを計算し、経路を覚える
 *   - 走りながら      ：search_check_unknown_walls() で、覚えた経路の壁を見直す（軽い）
 *       UNKNOWN_WALL_FOUND ：見ていない壁がある。その手前の区画を目的地にして、壁を見るまで走る（止まらない）
 *       UNKNOWN_WALL_BLOCKED：経路の上に壁があった（ふさがれた）。次の区画で止まって計算し直す
 *       UNKNOWN_WALL_NONE  ：経路の壁を全部見た。その場で止まって path_make_shortest() で最短経路を計算する
 * U ターンするときも止まるので、そのときにも計算し直す。
 */
#ifndef MAZE_SEARCH_H
#define MAZE_SEARCH_H

#include <stdbool.h>
#include <stdint.h>

#include "maze/maze_map.h"

#define STEP_MAX 9999  /* 歩数マップで「たどり着けない」を表す値 */

/* 未知壁探索で、覚えた経路を見直した結果 */
typedef enum {
  UNKNOWN_WALL_FOUND,    /* 経路の上に見ていない壁がある */
  UNKNOWN_WALL_BLOCKED,  /* 経路の上に壁があった（経路がふさがれた）。止まって計算し直す */
  UNKNOWN_WALL_NONE,     /* 経路の壁を全部見た（未知壁探索は終わり） */
} unknown_wall_result_t;

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
bool search_plan_unknown_walls(int goal_x, int goal_y, int goal_width, int goal_height);
unknown_wall_result_t search_check_unknown_walls(int *cell_x, int *cell_y, direction_t *wall_dir);

#endif /* MAZE_SEARCH_H */
