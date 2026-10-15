/*
 * explore.h
 * 探索走行（モード0）：迷路の処理（maze/）に次の行き先を聞き、機体を動かす処理（ctrl/run）で 1 区画ずつ動かす
 *
 * ■ 走り方（MODES.md のモード0。どれも吸引なしのスラローム探索）
 *   EXPLORE_ROUND_TRIP    往復      ：スタート → ゴール → スタート（No.1）
 *   EXPLORE_ONE_WAY       片道      ：スタート → ゴール。ゴールの区画の真ん中で止まる（No.2）
 *   EXPLORE_UNKNOWN_WALLS 未知壁探索：スタート → ゴール → 最短経路の上の見ていない壁を見に行く → スタート（No.3）
 *
 * ■ 使い方
 * 機体をスタート区画に北向きに置いてから explore_run() を呼ぶ。走り終わるまで戻ってこない。
 * 迷路は、今 RAM にある迷路の続きから使う（電源を入れた直後なら空の迷路）。
 */
#ifndef SYS_EXPLORE_H
#define SYS_EXPLORE_H

/* 探索の走り方 */
typedef enum {
  EXPLORE_ROUND_TRIP,     /* 往復 */
  EXPLORE_ONE_WAY,        /* 片道 */
  EXPLORE_UNKNOWN_WALLS,  /* 未知壁探索 */
} explore_type_t;

/* 探索の結果 */
typedef enum {
  EXPLORE_RESULT_DONE,      /* 最後まで走った（往復・未知壁探索はスタート、片道はゴールの区画の真ん中で止まっている） */
  EXPLORE_RESULT_FAILSAFE,  /* フェイルセーフで止まった（最後に壁を記録した 5 区画ぶんを戻してから、迷路を保存した） */
  EXPLORE_RESULT_NO_ROUTE,  /* 目的地（行きはゴール、帰りはスタート）へ行く道がなくなった（壁の読み間違いなど）、
                             * または動きすぎた。その場で止まっている。迷路は保存しない */
} explore_result_t;

explore_result_t explore_run(explore_type_t type, int goal_x, int goal_y, int goal_width, int goal_height);

#endif /* SYS_EXPLORE_H */
