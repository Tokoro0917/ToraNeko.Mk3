/*
 * shortest.h
 * 最短走行（モード1。中身は shortest.c）
 */
#ifndef SYS_SHORTEST_H
#define SYS_SHORTEST_H

#include <stdbool.h>

#include "sys/explore.h"

/* 最短走行の結果 */
typedef enum {
  SHORTEST_RESULT_DONE,      /* ゴールの区画のまん中で止まった（入ってきた向きを向き、位置を覚えている。shortest.c） */
  SHORTEST_RESULT_FAILSAFE,  /* フェイルセーフで止まった */
  SHORTEST_RESULT_NO_ROUTE,  /* スタートからゴールへの道がない（走らない） */
} shortest_result_t;

bool shortest_has_route(int number, int goal_x, int goal_y);
shortest_result_t shortest_run(int number, int goal_x, int goal_y);
explore_start_pose_t shortest_get_end_pose(void);
void shortest_forget_end_pose(void);

#endif /* SYS_SHORTEST_H */
