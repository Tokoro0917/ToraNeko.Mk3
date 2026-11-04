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
 * 壁の終わり・柱の端を見つける処理（ctrl/side_edge.c。2026-11-04）も調べる：
 *   6. 壁の終わり（高い値から下がる）を 1 回だけ見つけ、「壁の終わり」と見分け、半分を切った位置を端とする
 *   7. 柱だけ（低い山）を 1 回だけ見つけ、「柱だけ」と見分ける。境目は、端＋柱の距離
 *   8. 何もないところで、値が少しゆれても見つけない
 *   9. 直進でないあいだ（ターン）は見つけず、直進に戻っても 6mm 進むまでは見つけない
 *
 *   ./test_wall_ctrl --replay ファイル…   実機の走行ログ（モード5 No.4〜6。スタート区画から北へ走ったもの）を読ませて、
 *                                       見つけた端と、区画の境目（90mm ごと）までの距離を出す（ctrl/side_edge.c の値を決めるのに使う）
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <stdlib.h>
#include <string.h>

#include "ctrl/side_edge.h"
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

/* ---------------------------------------------------------------------------
 * 壁の終わり・柱の端（ctrl/side_edge.c）
 * ------------------------------------------------------------------------- */

static float s_edge_distance_mm = 0.0f;  /* 仮の、走った距離（side_edge のテスト用） */

/* 1mm 進めて、左の値を left、右の値を 100（何もない）にして、side_edge_update() を呼ぶ */
static void edge_step(int32_t left, bool is_straight)
{
  s_edge_distance_mm = s_edge_distance_mm + 1.0f;
  side_edge_update(s_edge_distance_mm, left, 100, is_straight);
}

/* 6〜9 のテスト */
static void test_side_edge(void)
{
  printf("[6] 壁の終わりを 1 回だけ見つけ、壁の終わりと見分ける\n");
  side_edge_reset();
  s_edge_distance_mm = 0.0f;
  for (int i = 0; i < 40; i++) {
    edge_step(1400, true);  /* 壁（左 1400） */
  }
  for (int i = 1; i <= 10; i++) {
    edge_step(1400 - 130 * i, true);  /* 10mm で 1400 → 100 に下がる（半分 757 は 40 + 約 5mm） */
  }
  for (int i = 0; i < 30; i++) {
    edge_step(110, true);
  }
  float edge_mm = side_edge_get_distance_mm(true);
  printf("  見つけた数 %d、端 %.1f mm、柱だけ %d、境目 %.1f mm\n", side_edge_get_count(true), edge_mm,
         side_edge_is_pillar(true), side_edge_get_boundary_mm(true));
  check(side_edge_get_count(true) == 1, "壁の終わりを 1 回だけ見つける");
  check(!side_edge_is_pillar(true), "壁の終わりと見分ける");
  check(edge_mm > 44.5f && edge_mm < 45.5f, "半分（1400 と 115 のまん中）を切った位置を端とする（約 44.9mm）");
  check(side_edge_get_count(false) == 0, "右（何もない）では見つけない");

  printf("[7] 柱だけ（低い山）を 1 回だけ見つけ、柱だけと見分ける\n");
  side_edge_reset();
  s_edge_distance_mm = 0.0f;
  for (int i = 0; i < 20; i++) {
    edge_step(115, true);
  }
  for (int i = 1; i <= 15; i++) {
    edge_step(115 + 29 * i, true);  /* 15mm で 550 まで上がる */
  }
  for (int i = 1; i <= 15; i++) {
    edge_step(550 - 29 * i, true);  /* 15mm で 115 まで下がる */
  }
  for (int i = 0; i < 20; i++) {
    edge_step(115, true);
  }
  printf("  見つけた数 %d、端 %.1f mm、柱だけ %d、境目 %.1f mm\n", side_edge_get_count(true),
         side_edge_get_distance_mm(true), side_edge_is_pillar(true), side_edge_get_boundary_mm(true));
  check(side_edge_get_count(true) == 1, "柱の山を 1 回だけ見つける");
  check(side_edge_is_pillar(true), "柱だけと見分ける（山が 900 より低い）");
  check(fabsf(side_edge_get_boundary_mm(true) - side_edge_get_distance_mm(true) - 29.9f) < 0.01f,
        "境目は、端 ＋ 柱だけの距離（29.9mm）");

  printf("[8] 何もないところで、値が少しゆれても見つけない\n");
  side_edge_reset();
  s_edge_distance_mm = 0.0f;
  for (int i = 0; i < 100; i++) {
    int32_t noise = 0;
    if (i == 10 || i == 13 || i == 16 || i == 40 || i == 41 || i == 70) {
      noise = 40;  /* ところどころ 40 だけ上がる */
    }
    edge_step(115 + noise, true);
  }
  check(side_edge_get_count(true) == 0, "何もないときより 100 以上高い山がなければ、見つけない");

  printf("[9] 直進でないあいだは見つけず、直進に戻っても 6mm 進むまでは見つけない\n");
  side_edge_reset();
  s_edge_distance_mm = 0.0f;
  for (int i = 0; i < 20; i++) {
    edge_step(1400, true);
  }
  for (int i = 1; i <= 10; i++) {
    edge_step(1400 - 130 * i, false);  /* ターンのあいだに下がる */
  }
  edge_step(1400, true);
  edge_step(100, true);   /* 直進に戻ってすぐ下がる（2mm 目） */
  for (int i = 0; i < 10; i++) {
    edge_step(100, true);
  }
  check(side_edge_get_count(true) == 0, "ターンのあいだと、直進に戻ってすぐの変化では見つけない");
}

/* --replay：実機の走行ログを読ませて、見つけた端を出す（先頭のコメント）。
 * ログの x（走った距離）に、スタートのタイヤの軸の位置 35mm を足して、スタート区画の南の辺からの位置 y にする */
static int replay(int file_count, char *files[])
{
  for (int i = 0; i < file_count; i++) {
    FILE *file = fopen(files[i], "r");
    if (file == NULL) {
      printf("%s が開けない\n", files[i]);
      continue;
    }
    side_edge_reset();
    int seen_left = 0;
    int seen_right = 0;
    char line[512];
    while (fgets(line, sizeof(line), file) != NULL) {
      if (line[0] < '0' || line[0] > '9') {
        continue;  /* 数字で始まらない行（見出しなど）は読まない */
      }
      /* 列：t,x,v,v_target,w,w_target,angle,sensor_L,sensor_R,...,phase（20 列目） */
      double column[21];
      int count = 0;
      char *token = strtok(line, ",");
      while (token != NULL && count < 21) {
        column[count] = atof(token);
        count = count + 1;
        token = strtok(NULL, ",");
      }
      if (count < 20) {
        continue;
      }
      bool is_straight = column[19] > 0.5 && column[19] < 1.5;  /* phase 1 が直線 */
      side_edge_update(column[1], column[7], column[8], is_straight);
      for (int side = 0; side < 2; side++) {
        bool is_left = side == 0;
        int now = side_edge_get_count(is_left);
        int before = seen_right;
        if (is_left) {
          before = seen_left;
        }
        if (now != before) {
          float y_mm = side_edge_get_distance_mm(is_left) + 35.0f;
          float boundary_y_mm = roundf(y_mm / 90.0f + 0.3f) * 90.0f;  /* 端は境目より手前で見つかるので、先の境目を選ぶ */
          const char *side_name = "右";
          if (is_left) {
            side_name = "左";
          }
          const char *kind_name = "壁の終わり";
          if (side_edge_is_pillar(is_left)) {
            kind_name = "柱だけ";
          }
          printf("%s %s %s 端 y=%.1f 境目 %.0f まで %.1f mm\n", files[i], side_name, kind_name, y_mm, boundary_y_mm,
                 boundary_y_mm - y_mm);
          if (is_left) {
            seen_left = now;
          } else {
            seen_right = now;
          }
        }
      }
    }
    fclose(file);
  }
  return 0;
}

int main(int argc, char *argv[])
{
  if (argc >= 2 && strcmp(argv[1], "--replay") == 0) {
    return replay(argc - 2, argv + 2);
  }

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

  test_side_edge();

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗 %d 件\n", s_fail_count);
  return 1;
}
