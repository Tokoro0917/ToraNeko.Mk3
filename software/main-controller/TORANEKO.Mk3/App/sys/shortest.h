/*
 * shortest.h
 * 最短走行（モード1。中身は shortest.c）
 */
#ifndef SYS_SHORTEST_H
#define SYS_SHORTEST_H

#include <stdbool.h>

/* 最短走行の結果 */
typedef enum {
  SHORTEST_RESULT_DONE,      /* ゴールの区画のまん中で止まった */
  SHORTEST_RESULT_FAILSAFE,  /* フェイルセーフで止まった */
  SHORTEST_RESULT_NO_ROUTE,  /* スタートからゴールへの道がない（走らない） */
} shortest_result_t;

bool shortest_has_route(int goal_x, int goal_y);
shortest_result_t shortest_run(int number, int goal_x, int goal_y);

#endif /* SYS_SHORTEST_H */
