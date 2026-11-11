/*
 * explore.h
 * 探索走行（モード0）：迷路の処理（maze/）に次の行き先を聞き、機体を動かす処理（ctrl/run）で 1 区画ずつ動かす
 *
 * ■ 走り方（MODES.md のモード0。どれも吸引なしのスラローム探索）
 *   EXPLORE_ROUND_TRIP    往復      ：スタート → ゴール → スタート（No.1）
 *   EXPLORE_ONE_WAY       片道      ：スタート → ゴール。ゴールの区画の真ん中で止まる（No.2）
 *   EXPLORE_UNKNOWN_WALLS 未知壁探索：スタート → ゴール → 最短経路の上の見ていない壁を見に行く → スタート（No.3）
 *   EXPLORE_UNKNOWN_WALLS_KNOWN 未知壁探索＋既知区間加速：No.3 と同じ道を通り、行きと帰りで、見たことのある区画が
 *                                 まっすぐ 2 区画以上続くところは、まとめて速く走る（No.4）
 *   No.5・6 は、No.3・4 と同じ走り方を、最短走行のあとのゴールから始める（下の「■ ゴールから始める探索」）
 *
 * ■ 使い方
 * 機体をスタート区画に北向きに置いてから explore_run() を呼ぶ。走り終わるまで戻ってこない。
 * 迷路は、今 RAM にある迷路の続きから使う（電源を入れた直後なら空の迷路）。
 *
 * ■ ゴールから始める探索（No.5・6。2026-11-11。作者の案）
 * 最短走行でゴールに止まると、止まった区画・入ってきた向き・そこで見た壁を覚えて、入ってきた向きへ向きを変えて止まる
 * （sys/shortest.c）。そのあと置き直さずに explore_run_from_goal() を呼ぶと、そこから未知壁探索をしてスタートへ戻る。
 * 止まった位置は explore_start_pose_t で渡す。
 */
#ifndef SYS_EXPLORE_H
#define SYS_EXPLORE_H

#include <stdbool.h>

#include "maze/maze_map.h"

/* 探索の走り方 */
typedef enum {
  EXPLORE_ROUND_TRIP,     /* 往復 */
  EXPLORE_ONE_WAY,        /* 片道 */
  EXPLORE_UNKNOWN_WALLS,  /* 未知壁探索 */
  EXPLORE_UNKNOWN_WALLS_KNOWN,  /* 未知壁探索＋既知区間加速 */
} explore_type_t;

/* 探索の結果 */
typedef enum {
  EXPLORE_RESULT_DONE,      /* 最後まで走った（往復・未知壁探索はスタート、片道はゴールの区画の真ん中で止まっている） */
  EXPLORE_RESULT_FAILSAFE,  /* フェイルセーフで止まった（最後に壁を記録した 5 区画ぶんを戻してから、迷路を保存した） */
  EXPLORE_RESULT_NO_ROUTE,  /* 目的地（行きはゴール、帰りはスタート）へ行く道がなくなった（壁の読み間違いなど）、
                             * または動きすぎた。その場で止まっている。迷路は保存しない */
} explore_result_t;

/* ゴールから始める探索の、始める位置（最短走行がゴールで止まったときに覚える。sys/shortest.c） */
typedef struct {
  bool is_set;              /* 覚えているか（false なら、ゴールから始める探索はできない） */
  int x;                    /* 止まっている区画（このまん中で止まっている） */
  int y;
  direction_t arrive_dir;   /* その区画に入ってきた向き。今は、その反対（入ってきた向き）を向いている */
  bool wall_front;          /* 止まったときにセンサで見た壁（arrive_dir を向いていたときの前・左・右） */
  bool wall_left;
  bool wall_right;
} explore_start_pose_t;

explore_result_t explore_run(explore_type_t type, int goal_x, int goal_y, int goal_width, int goal_height);
explore_result_t explore_run_from_goal(explore_type_t type, explore_start_pose_t pose,
                                       int goal_x, int goal_y, int goal_width, int goal_height);

#endif /* SYS_EXPLORE_H */
