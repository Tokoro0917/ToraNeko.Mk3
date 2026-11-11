/*
 * shortest.h
 * 最短走行（モード1。中身は shortest.c）
 */
#ifndef SYS_SHORTEST_H
#define SYS_SHORTEST_H

#include <stdbool.h>

#include "maze/maze_map.h"

/* 最短走行の結果 */
typedef enum {
  SHORTEST_RESULT_DONE,      /* ゴールの区画のまん中で止まった（入ってきた向きを向き、位置を覚えている。shortest.c） */
  SHORTEST_RESULT_FAILSAFE,  /* フェイルセーフで止まった */
  SHORTEST_RESULT_NO_ROUTE,  /* スタートからゴールへの道がない（走らない） */
} shortest_result_t;

/* 最短走行がゴールで止まったときに覚えた位置（shortest.c の「■ ゴールで止まったあと」）。電源を切ると消える */
typedef struct {
  bool is_set;              /* 覚えているか（最後の最短走行が、ゴールで止まって向きを変えられたら true） */
  int x;                    /* 止まった区画（このまん中で止まっている） */
  int y;
  direction_t arrive_dir;   /* その区画に入ってきた向き。止まったあとは、その反対（入ってきた向き）を向いている */
  bool wall_front;          /* 止まったときにセンサで見た壁（arrive_dir を向いていたときの前・左・右） */
  bool wall_left;
  bool wall_right;
} shortest_end_pose_t;

bool shortest_has_route(int number, int goal_x, int goal_y);
shortest_result_t shortest_run(int number, int goal_x, int goal_y);
shortest_end_pose_t shortest_get_end_pose(void);

#endif /* SYS_SHORTEST_H */
