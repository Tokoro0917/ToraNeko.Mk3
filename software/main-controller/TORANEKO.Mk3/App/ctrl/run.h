/*
 * run.h
 * 機体を動かす（直進・スラローム・止まる・発進など）
 *
 * 中身は run.c（2026-10-24 に作った。横の壁に沿う補正と前壁合わせは、まだない）。探索（sys/explore.c）が使う動き。
 * PC のテストでは、test/test_maze.c の中の「仮の機体」がこの関数を持っている。
 *
 * ■ 探索の動き（探索の速さ、吸引なし）
 * 機体は、次の 2 つの状態のどちらかにいる（sys/explore.c、maze/search.c と同じ考え方）。
 *   走っている  ：区画の入口の辺を、探索の速さで通るところ。ここでセンサがその区画の前・左・右の壁を見る
 *   止まっている：区画の真ん中で止まっている
 *
 *   run_search_start()            スタート区画から走り出し、北隣の区画 (0, 1) の入口の辺まで進む
 *   run_search_straight()         走っている → 1 区画まっすぐ進み、次の区画の入口の辺へ
 *   run_search_known_straight(n)  走っている → 見たことのある区画を n 区画まっすぐ速く進み、その先の区画の入口の辺へ
 *                                 （既知区間加速。探索の速さで着く）
 *   run_search_turn_left()        走っている → スラロームで左 90°（区画の中で曲がり、左の区画の入口の辺へ）
 *   run_search_turn_right()       走っている → スラロームで右 90°
 *   run_search_stop_at_center()   走っている → 今入った区画の真ん中で止まる
 *   run_search_go_from_center()   止まっている → 前に壁があれば前壁合わせをし、その場で向きを変えて発進し、
 *                                 次の区画の入口の辺へ（U ターンは「止まる」と「後ろを向いて発進」でできている）
 *
 * ■ 中身（run.c）を作るときに守ること（sys/explore.c と、test/test_maze.c の「仮の機体」はこれを前提にしている）
 *   1. 走っているあいだの動き（直進・スラローム）は、次の区画の入口の辺に着いたところで戻る。
 *      機体は探索の速さのまま進み続け、次の動きを頼まれるまで、まっすぐ進む。
 *      呼んだ側は、戻ってから次の動きを頼むまでのあいだに、壁の記録と歩数マップの計算をする（短くする）
 *   2. run_search_stop_at_center() は、真ん中で止まったあと、モーターの出力を 0 にしてから戻る
 *      （このあと Flash に保存すると CPU が 1〜2 秒止まるが、出力が 0 なので機体は動かない）
 *   3. フェイルセーフで止まったときは、動きの途中でも戻ってくる（呼んだ側が failsafe_is_stopped() で調べる）。
 *      フェイルセーフで止まっているあいだに呼ばれたら、モーターを動かさず、何も待たずにすぐ戻る
 */
#ifndef CTRL_RUN_H
#define CTRL_RUN_H

#include <stdbool.h>
#include <stdint.h>

/* 止まっているところから発進するときの向きの変え方 */
typedef enum {
  RUN_TURN_NONE,   /* 向きを変えずに発進する */
  RUN_TURN_LEFT,   /* その場で左に 90° 回ってから発進する */
  RUN_TURN_RIGHT,  /* その場で右に 90° 回ってから発進する */
  RUN_TURN_BACK,   /* その場で左に 90° を 2 回回ってから発進する（U ターン。回るたびに、前に壁があれば前壁合わせ） */
} run_turn_t;

void run_search_start(void);
void run_search_straight(void);
void run_search_known_straight(int cells);
void run_search_turn_left(void);
void run_search_turn_right(void);
void run_search_stop_at_center(void);
void run_search_go_from_center(run_turn_t turn);

/* 最短走行（モード1 は sys/shortest.c、モード3 のターンの調整は sys/turn_test.c）の動き。
 *   run_fast_start()                                  制御とログを始める（動かない）
 *   run_fast_straight(距離, 最高速, 終わりの速さ, 加速度) 直進（止まっているところからでもよい）。横の壁に沿う補正を入れる
 *   run_fast_diag_straight(…)                         斜めの直進（同じ。横の壁に沿う補正は入れない）
 *   run_fast_turn_left()・_right()                     探索の速さ（300mm/s）で、小回りのスラローム 90°
 *   run_fast_turn(ターン, 左なら true, 速さ)           大回り・斜めのターン（ふつうはターンの速さ。遅くても同じ形で曲がる）
 *   run_fast_stop(距離, 最高速, 加速度)                進みながら止まり、モーターの出力を 0 にして、ログを止める
 *   run_fast_diag_stop(…)                             斜めに進みながら止まる（同じ。横の壁に沿う補正は入れない）
 * フェイルセーフで止まったときは、探索と同じく、すぐに戻る */

/* 大回り・斜めのターン（始めと終わりの位置は config/turn_params.h の先頭のコメント） */
typedef enum {
  RUN_FAST_TURN_BIG90,   /* 大回り 90°：区画のまん中 → 区画のまん中 */
  RUN_FAST_TURN_BIG180,  /* 大回り 180°：区画のまん中 → 区画のまん中（反対向き） */
  RUN_FAST_TURN_IN45,    /* 斜め入り 45°：区画のまん中 → 壁の辺のまん中（斜め） */
  RUN_FAST_TURN_IN135,   /* 斜め入り 135° */
  RUN_FAST_TURN_OUT45,   /* 斜め出 45°：壁の辺のまん中（斜め） → 区画のまん中 */
  RUN_FAST_TURN_OUT135,  /* 斜め出 135° */
  RUN_FAST_TURN_V90,     /* V90°：壁の辺のまん中（斜め） → 壁の辺のまん中（斜め） */
} run_fast_turn_t;

void run_fast_start(void);
void run_fast_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2);
void run_fast_diag_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2);
void run_fast_turn_left(void);
void run_fast_turn_right(void);
void run_fast_turn(run_fast_turn_t turn, bool is_left, float speed_mm_s);
void run_fast_stop(float distance_mm, float max_speed_mm_s, float accel_mm_s2);
void run_fast_diag_stop(float distance_mm, float max_speed_mm_s, float accel_mm_s2);

/* 走っているあいだに、動きが戻ってから次の動きを頼まれるまでの、いちばん長かった時間 [µs]（発進してから）。
 * 探索の計算が長すぎないかを確かめる（300mm/s では 1ms で 0.3mm 進む）。モード7 No.1 で出す */
uint32_t run_get_longest_wait_us(void);

/* 探索の前壁補正（run.c の先頭のコメント）を測った回数と、量のいちばん小さい・大きい値 [mm]。モード7 No.1 で出す */
int run_get_front_correction_count(void);
float run_get_front_correction_min_mm(void);
float run_get_front_correction_max_mm(void);

/* 探索の壁切れ（run.c の先頭のコメント）を見つけた回数と、見つけた位置（区画の入口から）の平均・最小・最大 [mm]。モード7 No.1 で出す */
int run_get_wall_edge_count(void);
float run_get_wall_edge_average_mm(void);
float run_get_wall_edge_min_mm(void);
float run_get_wall_edge_max_mm(void);

#endif /* CTRL_RUN_H */
