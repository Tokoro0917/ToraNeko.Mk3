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
 *   run_search_align_to_side(左)  止まっている → 横の壁のほうへ 90° 回って前壁合わせをし、元の向きに戻って止まる
 *                                 （横壁合わせ。横の壁が近すぎたときに sys/explore.c が使う）
 *   run_search_go_from_center()   止まっている → 前に壁があれば前壁合わせをし、その場で向きを変えて発進し、
 *                                 次の区画の入口の辺へ（U ターンは「止まる」と「後ろを向いて発進」でできている）
 *   run_search_turn_back_at_center()  止まっている → 前壁合わせ・左 90°・前壁合わせ・左 90° で反対を向いて止まる
 *                                 （スタート区画に戻ったとき、次にすぐスタートできるように北を向く。2026-11-10。
 *                                 最短走行でゴールに止まったとき、入ってきた向きを向くのにも使う。2026-11-11）
 *   run_search_back_to_wall()     止まっている → まっすぐ後ろへゆっくり下がり、お尻を後ろの壁に押し当てて止まる
 *                                 （北を向いたあとに使う。走り出す前の置き方と同じになる。2026-11-10）
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
void run_search_turn_back_at_center(void);
void run_search_back_to_wall(void);
void run_search_align_to_side(bool is_left);

/* 最短走行（モード1 は sys/shortest.c、モード3 のターンの調整は sys/turn_test.c）の動き。
 *   run_fast_start()                                  制御とログを始める（動かない）
 *   run_fast_straight(距離, 最高速, 終わりの速さ, 加速度, 始める位置)
 *                                                     直進（止まっているところからでもよい）。横の壁に沿う補正を入れる。
 *                                                     始める位置は、うしろの区画の境目から何 mm 先か（壁切れに使う）
 *   run_fast_diag_straight(…)                         斜めの直進（同じ。横の壁に沿う補正のかわりに、斜めの補正を入れる）
 *   run_fast_turn_left()・_right()                     探索の速さ（300mm/s）で、小回りのスラローム 90°
 *   run_fast_turn(ターン, 左なら true, 速さ)           大回り・斜めのターン（そのターンの速さ以下。遅くても同じ形で曲がる）
 *   run_fast_turn_speed_mm_s(ターン)                  そのターンを曲がる速さ（config/turn_params.h の *_SPEED_MM_S）
 *   run_fast_set_turn_set(組)                         ターンの値の組を選ぶ（1：800・600mm/s、2：7 つとも 1000mm/s、3：ゆるい 1200・きつい 1000mm/s。はじめは 1）
 *   run_fast_stop(距離, 最高速, 加速度)                進みながら止まり、モーターの出力を 0 にして、ログを止める
 *   run_fast_diag_stop(…)                             斜めに進みながら止まる（同じ。横の壁に沿う補正のかわりに、斜めの補正を入れる）
 *   run_fast_set_diag_correction(入れるなら true)      斜めの直進で、斜めの補正を入れるか（ふつうは入れる。ターンの調整では入れない）
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
void run_fast_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2,
                       float start_in_cell_mm);
void run_fast_diag_straight(float distance_mm, float max_speed_mm_s, float end_speed_mm_s, float accel_mm_s2);
void run_fast_turn_left(void);
void run_fast_turn_right(void);
void run_fast_turn(run_fast_turn_t turn, bool is_left, float speed_mm_s);
float run_fast_turn_speed_mm_s(run_fast_turn_t turn);
void run_fast_set_turn_set(int set);
void run_fast_stop(float distance_mm, float max_speed_mm_s, float accel_mm_s2);
void run_fast_diag_stop(float distance_mm, float max_speed_mm_s, float accel_mm_s2);
void run_fast_set_diag_correction(bool on);

/* 走っているあいだに、動きが戻ってから次の動きを頼まれるまでの、いちばん長かった時間 [µs]（発進してから）。
 * 探索の計算が長すぎないかを確かめる（300mm/s では 1ms で 0.3mm 進む）。モード7 No.1 で出す */
uint32_t run_get_longest_wait_us(void);

/* 探索の前壁補正（run.c の先頭のコメント）を測った回数と、量の平均・いちばん小さい・大きい値 [mm]（上限をかける前の量）。モード7 No.1 で出す */
int run_get_front_correction_count(void);
float run_get_front_correction_average_mm(void);
float run_get_front_correction_min_mm(void);
float run_get_front_correction_max_mm(void);

/* 前壁補正の 1 回ごとの記録（最初の 40 回まで）。量と、そのときまでに走った距離 [mm]（探索を始めてから）。モード7 No.1 で出す */
int run_get_front_correction_list_count(void);
float run_get_front_correction_list_mm(int index);
float run_get_front_correction_list_traveled_mm(int index);

/* 壁切れ（探索・最短走行。run.c の先頭のコメント）の 1 回ごとの記録（最初の 40 回まで。モード7 No.1 で出す）：
 * いちばん近い区画の境目からのずれ [mm]（+ なら思っているより手前）、そのときの目標の速さ [mm/s]、走った距離 [mm]（走り始めてから）、
 * 目標の位置からの遅れ [mm]（ずれからは引いてある）、左のセンサか、柱だけか（壁の終わりなら false）、位置を直すのに使えたか */
int run_get_edge_count(void);
float run_get_edge_error_mm(int index);
float run_get_edge_speed_mm_s(int index);
float run_get_edge_traveled_mm(int index);
float run_get_edge_lag_mm(int index);
bool run_get_edge_is_left(int index);
bool run_get_edge_is_pillar(int index);
bool run_get_edge_used(int index);

/* 前壁補正を測ったときの、くわしい様子（1 回ごと。最初の 40 回まで）。2026-11-03 に足した。
 * 止めて測ると合っているのに、走っていると大きくずれる場所があったので、そのとき何が起きていたかを見る */
typedef struct {
  float front_left_mm;   /* 左前のセンサから求めた、前の壁までの距離 [mm] */
  float front_right_mm;  /* 右前のセンサから求めた、前の壁までの距離 [mm] */
  float side_left_mm;    /* 左の壁までの距離 [mm]（区画のまん中で 42） */
  float side_right_mm;   /* 右の壁までの距離 [mm] */
  float overrun_mm;      /* 入口の辺から、もう進んでいた距離 [mm]（測った量に足している分） */
  float speed_mm_s;      /* そのときの速さ [mm/s]（エンコーダ） */
  float yaw_deg;         /* 区画の向き（探索を始めた向きから 90° ごと）に対する、機体の向きのずれ [deg]（左回りが +） */
} run_front_detail_t;
run_front_detail_t run_get_front_correction_detail(int index);

#endif /* CTRL_RUN_H */
