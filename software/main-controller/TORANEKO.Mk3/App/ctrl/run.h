/*
 * run.h
 * 機体を動かす（直進・スラローム・止まる・発進など）
 *
 * 中身（run.c）はステップ7で作る。今は、探索（sys/explore.c）が使う動きの形だけを先に決めておく。
 * PC のテストでは、test/test_maze.c の中の「仮の機体」がこの関数を持っている。
 *
 * ■ 探索の動き（探索の速さ、吸引なし）
 * 機体は、次の 2 つの状態のどちらかにいる（sys/explore.c、maze/search.c と同じ考え方）。
 *   走っている  ：区画の入口の辺を、探索の速さで通るところ。ここでセンサがその区画の前・左・右の壁を見る
 *   止まっている：区画の真ん中で止まっている
 *
 *   run_search_start()            スタート区画から走り出し、北隣の区画 (0, 1) の入口の辺まで進む
 *   run_search_straight()         走っている → 1 区画まっすぐ進み、次の区画の入口の辺へ
 *   run_search_turn_left()        走っている → スラロームで左 90°（区画の中で曲がり、左の区画の入口の辺へ）
 *   run_search_turn_right()       走っている → スラロームで右 90°
 *   run_search_stop_at_center()   走っている → 今入った区画の真ん中で止まる
 *   run_search_go_from_center()   止まっている → 前に壁があれば前壁合わせをし、その場で向きを変えて発進し、
 *                                 次の区画の入口の辺へ（U ターンは「止まる」と「後ろを向いて発進」でできている）
 *
 * フェイルセーフで止まったときは、動きの途中でも戻ってくる（呼んだ側が failsafe_is_stopped() で調べる）。
 */
#ifndef CTRL_RUN_H
#define CTRL_RUN_H

/* 止まっているところから発進するときの向きの変え方 */
typedef enum {
  RUN_TURN_NONE,   /* 向きを変えずに発進する */
  RUN_TURN_LEFT,   /* その場で左に 90° 回ってから発進する */
  RUN_TURN_RIGHT,  /* その場で右に 90° 回ってから発進する */
  RUN_TURN_BACK,   /* その場で 180° 回ってから発進する（U ターン） */
} run_turn_t;

void run_search_start(void);
void run_search_straight(void);
void run_search_turn_left(void);
void run_search_turn_right(void);
void run_search_stop_at_center(void);
void run_search_go_from_center(run_turn_t turn);

#endif /* CTRL_RUN_H */
