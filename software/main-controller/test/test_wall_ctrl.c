/*
 * test_wall_ctrl.c
 * 壁の終わりの見分け（ctrl/wall_ctrl.c の WALL_END_*。2026-11-04）の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * 壁センサ（drv/wall.c）は、ここで仮に作る（テストが決めた値を答える）。走った距離を 1mm ずつ進めながら
 * wall_ctrl_update_end_check() を呼び、横の壁の補正に使える壁（wall_ctrl_get_side_walls_used()）を調べる。
 *   1. 壁の値が変わらなければ、左右とも使える
 *   2. 壁の終わり（5mm で 60 より大きく下がる）では、下限（左 1000）を切る前でも、左の壁を使わなくなる
 *   3. 下がり方がゆっくり（横に少しずつ寄るくらい）なら、使い続ける
 *   4. 壁の終わりとみなしてから 15mm 進んで、値が下がっていなければ、また使う
 *   5. 止まっているあいだは、値が変わっても見分けない（距離あたりで見るので）
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "ctrl/wall_ctrl.h"
#include "drv/wall.h"

static int s_fail_count = 0;

/* ok が false なら失敗として数え、message を表示する */
static void check(bool ok, const char *message)
{
  if (!ok) {
    s_fail_count = s_fail_count + 1;
    printf("  失敗: %s\n", message);
  }
}

/* ---------------------------------------------------------------------------
 * 仮の壁センサ（ctrl/wall_ctrl.c が使う drv/wall.c の関数）
 * 左の壁 = 組B の受光 3、右の壁 = 組A の受光 2、左前 = 組A の受光 1、右前 = 組B の受光 4（ctrl/wall_ctrl.c の先頭のコメント）
 * ------------------------------------------------------------------------- */

static int32_t s_fake_left = 1460;   /* 左の壁のセンサの値（区画のまん中の値） */
static int32_t s_fake_right = 815;   /* 右の壁のセンサの値（区画のまん中の値） */

int32_t wall_get_a_minus_dark(int num)
{
  if (num == 2) {
    return s_fake_right;
  }
  return 0;  /* 左前：前に壁はない */
}

int32_t wall_get_b_minus_dark(int num)
{
  if (num == 3) {
    return s_fake_left;
  }
  return 0;  /* 右前：前に壁はない */
}

/* ---------------------------------------------------------------------------
 * テスト
 * ------------------------------------------------------------------------- */

static float s_distance_mm = 0.0f;  /* 仮の、走った距離 */

/* 左の壁を、横の壁の補正に使えるか（使える壁の 1 が左） */
static bool left_used(void)
{
  int used = wall_ctrl_get_side_walls_used();
  if (used == 1 || used == 3) {
    return true;
  }
  return false;
}

/* 走った距離を 1mm 進めて、左の値を left にし、壁の終わりの見分けを 1 回まわす */
static void step(int32_t left)
{
  s_fake_left = left;
  s_distance_mm = s_distance_mm + 1.0f;
  wall_ctrl_update_end_check(s_distance_mm);
}

/* 左右とも区画のまん中の値で、20mm 進んだところから始め直す */
static void reset(void)
{
  s_fake_left = 1460;
  s_fake_right = 815;
  s_distance_mm = 0.0f;
  wall_ctrl_reset_end_check(s_distance_mm);
  for (int i = 0; i < 20; i++) {
    step(1460);
  }
}

int main(void)
{
  printf("[1] 壁の値が変わらなければ、左右とも使える\n");
  reset();
  check(wall_ctrl_get_side_walls_used() == 3, "左右とも使える");

  printf("[2] 壁の終わり（1mm で 16 ずつ下がる）では、下限 1000 を切る前に、左の壁を使わなくなる\n");
  reset();
  int32_t left = 1460;
  int stopped_after_mm = -1;
  for (int i = 1; i <= 20; i++) {
    left = left - 16;
    step(left);
    if (!left_used() && stopped_after_mm < 0) {
      stopped_after_mm = i;
    }
  }
  printf("  下がり始めてから %d mm で使わなくなった（そのときの値 %d）\n", stopped_after_mm, 1460 - 16 * stopped_after_mm);
  check(stopped_after_mm > 0 && stopped_after_mm <= 5, "下がり始めてから 5mm 以内に、使わなくなる");
  check(1460 - 16 * stopped_after_mm > 1000, "値が下限 1000 を切るより前に、使わなくなる");
  check(wall_ctrl_get_side_walls_used() == 2, "右の壁は使い続ける");

  printf("[3] ゆっくり下がる（1mm で 5 ずつ。5mm で 25）なら、使い続ける\n");
  reset();
  left = 1460;
  bool always_used = true;
  for (int i = 0; i < 40; i++) {
    left = left - 5;
    step(left);
    if (!left_used()) {
      always_used = false;
    }
  }
  check(always_used, "横に少しずつ寄るくらいの変わり方では、壁の終わりとみなさない");

  printf("[4] 壁の終わりとみなしてから 15mm 進んで、値が下がっていなければ、また使う\n");
  reset();
  for (int i = 0; i < 6; i++) {
    step(1460 - 20 * (i + 1));  /* 6mm で 120 下がる：壁の終わりとみなす */
  }
  check(!left_used(), "下がったところで、使わなくなる");
  int32_t steady = 1460 - 20 * 6;
  for (int i = 0; i < 10; i++) {
    step(steady);
  }
  check(!left_used(), "みなしてから 15mm たつまでは、値が止まっても使わない");
  for (int i = 0; i < 10; i++) {
    step(steady);
  }
  check(left_used(), "15mm より進んで、値が下がっていなければ、また使う");

  printf("[5] 止まっているあいだは、値が変わっても見分けない\n");
  reset();
  for (int i = 0; i < 10; i++) {
    s_fake_left = 1460 - 30 * (i + 1);  /* 300 下がるが、距離は進まない */
    wall_ctrl_update_end_check(s_distance_mm);
  }
  check(left_used(), "距離が進んでいないので、壁の終わりとみなさない（値は下限 1000 より上）");

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗 %d 件\n", s_fail_count);
  return 1;
}
