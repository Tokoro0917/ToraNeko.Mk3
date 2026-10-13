/*
 * path.h
 * 経路（走る動きの並び）を作る
 *
 *   - 既知区間の経路：探索中、すでに壁を見た区画が続くところを、まとめて速く走るための経路
 *   - （このあと追加）最短経路：ダイクストラで作る最短走行の経路と、斜めへの変換
 *
 * ■ 経路の表し方
 * 「動き」（直進、左ターン、大回り左 90° など）を順に並べたもの。
 * 直進だけは長さを持っていて、半区画の数で表す（1 区画 = 2 半区画）。
 *
 * ■ 位置の考え方（search.c と同じ）
 * ロボットは区画 (x, y) の入口の辺にいて、向き dir でその区画に入るところ。
 * 1 つの動きは、その区画を通って次の区画の入口の辺まで進む。
 */
#ifndef MAZE_PATH_H
#define MAZE_PATH_H

#include "maze/maze_map.h"

/* 動きの種類 */
typedef enum {
  MOTION_STRAIGHT,            /* 直進。長さは half_count [半区画] */
  MOTION_TURN_LEFT,           /* 探索用の左ターン（小さいスラローム 90°）。1 区画の中で曲がる */
  MOTION_TURN_RIGHT,          /* 探索用の右ターン */
  MOTION_UTURN,               /* 区画の中で止まって 180° 回り、入ってきた辺から出る */
  MOTION_BIG_TURN_LEFT_90,    /* 大回り左 90°。前後の直進を半区画ずつ使って大きく曲がる */
  MOTION_BIG_TURN_RIGHT_90,   /* 大回り右 90° */
  MOTION_BIG_TURN_LEFT_180,   /* 大回り左 180°（左に 2 回続けて曲がるところ） */
  MOTION_BIG_TURN_RIGHT_180,  /* 大回り右 180° */
} motion_t;

/* 経路の 1 つの動き */
typedef struct {
  motion_t motion;
  int half_count;  /* 直進の長さ [半区画]。直進以外では 0 */
} path_step_t;

#define PATH_STEP_MAX (MAZE_SIZE * MAZE_SIZE)  /* 経路に入れられる動きの数 */

/* 経路 */
typedef struct {
  path_step_t steps[PATH_STEP_MAX];  /* 動きの並び */
  int count;                         /* 動きの数 */
  int end_x;                         /* 走り終わったときの区画（その区画の入口の辺にいる） */
  int end_y;
  direction_t end_dir;               /* 走り終わったときの向き */
} path_t;

void path_make_known(int x, int y, direction_t dir, path_t *path);

#endif /* MAZE_PATH_H */
