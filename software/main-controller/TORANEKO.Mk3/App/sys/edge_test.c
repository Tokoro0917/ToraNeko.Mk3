/*
 * edge_test.c
 * 壁切れ・柱の見え方テスト（モード5 No.4〜6。MODES.md。2026-11-04 に作った。KOGUMA のモード3 No.6〜9 と同じ考え方）
 *
 * スタート区画から、まっすぐ 3 区画先の区画のまん中まで、決めた速さで走って止まる。走るあいだの横のセンサの値を、
 * 走行ログに細かく（1〜2ms ごと）残す。柱だけのところ・壁が切れるところで、横のセンサの値がどう変わるか
 * （山の高さ、下がり始める位置、速さで変わるか）を見て、柱を壁切れに使うやり方を決める。
 *   No.4：300mm/s（探索と同じ）  No.5：800mm/s  No.6：1000mm/s（最短走行 No.4 の直進の最高速と同じ）
 * 2026-11-04 に、6 区画先までにしていたのを 3 区画先までにした（作者の通り道が 4 区画しかなく、いちばん奥の壁に当たっていた）。
 *
 * ■ 流れ
 *   1. 吸引を上げて（0.5 秒）、0.5 秒待つ（最短走行と同じく、吸引して走る。吸引なしでは 4000mm/s² で滑る）
 *   2. ジャイロのゼロ点を測り直し、制御とログを始める（ログは No.4 が 2ms ごと、No.5・6 が 1ms ごと。1000 行ぶん）
 *   3. 決めた速さまで EDGE_TEST_ACCEL_MM_S2 で速くして、そのまま走る（横の壁に沿う補正あり。最短走行の壁切れの記録もとる）
 *   4. 3 区画先の区画のまん中で止まるように、同じ加速度で遅くして止まる
 *   5. 吸引を止める
 * 走ったあと、モード7 No.1 で、走行ログ（sensor_L・sensor_R の列）と、壁切れの記録（edge の行）を出す。
 *
 * ■ 置き方と壁の並べ方
 * スタート区画 (0, 0) に、北向きに、お尻を後ろの壁に付けて置く（最短走行と同じ）。北へ (0, 0)〜(0, 3) の 4 区画を走り、
 * (0, 3) のまん中で止まる（(0, 3) の北には壁があってよい）。横の壁は、区画ごとに次のように置く（○ 壁あり、× 壁なし。
 * 柱はどこにも立てておく）。4 区画では全部の組み合わせを一度に作れないので、左右を入れかえた 2 通りで走る：
 *     置き方 1       (0,0) (0,1) (0,2) (0,3)        置き方 2（左右を入れかえる）
 *     左（西）        ○     ○     ×     ○          左   ○     ×     ×     ×
 *     右（東）        ○     ×     ×     ×          右   ○     ○     ×     ○
 * 区画の境目（北の辺）ごとに、次のものが見える（KOGUMA の docs/pillar_cut_design.md の表の番号）：
 *     #2 壁 → 壁なし（壁の終わり）、#3 壁なし → 壁なし（柱だけ）、#4 壁なし → 壁（壁の始まり）
 *              y=90  y=180 y=270（区画 (0,0) の南の辺から [mm]）
 *     置き方 1  左   -    #2    #4      右  #2    #3    #3
 *     置き方 2  左  #2    #3    #3      右   -    #2    #4
 * 1000mm/s では、速くするのに 62mm（8000mm/s²）かかるので、y=180 の境目を、同じ速さで通る。
 */
#include "sys/edge_test.h"

#include <stdbool.h>

#include "config/run_params.h"
#include "ctrl/run.h"
#include "drv/fan.h"
#include "sys/failsafe.h"
#include "sys/log.h"
#include "sys/suction.h"
#include "sys/ui.h"

#define EDGE_TEST_ACCEL_MM_S2     8000.0f  /* 速くする・止まるときの加速度 [mm/s²]（吸引 5.0V では 10000mm/s² まで滑らない。
                                              4 区画の短い通り道で、決めた速さで走る区間を取るため、6000 から上げた） */
#define EDGE_TEST_SUCTION_WAIT_MS 500      /* 吸引を上げてから、走り出す前に待つ時間 [ms] */
#define EDGE_TEST_CELLS           3        /* 何区画先の区画のまん中まで走るか（通り道は 4 区画） */

/* No. で、走る速さ [mm/s] を決める（4 は 300、5 は 800、6 は 1000） */
static float speed_of_number(int number)
{
  if (number == 5) {
    return 800.0f;
  }
  if (number == 6) {
    return 1000.0f;
  }
  return 300.0f;
}

/* No. で、走行ログの間隔 [ms] を決める（ログは 1000 行。No.4 は約 1 秒走るので 2ms ごと、No.5・6 は 0.5 秒くらいなので 1ms ごと） */
static int log_interval_of_number(int number)
{
  if (number == 4) {
    return 2;
  }
  return 1;
}

/* 終わったことを LED で知らせる。フェイルセーフで止まったときは、スイッチが押されるまで待って解除する */
static void signal_end(void)
{
  if (failsafe_is_stopped()) {
    ui_wait_failsafe_reset();
    failsafe_clear();
  } else {
    ui_signal_ok();
  }
}

/* モード5 No.number の壁切れ・柱の見え方テストを走らせる（流れは先頭のコメント） */
void edge_test_run(int number)
{
  float speed_mm_s = speed_of_number(number);
  /* 走る距離：スタートのタイヤの軸（南の辺から 35mm）から、3 区画先の区画のまん中まで（280mm） */
  float total_mm = EDGE_TEST_CELLS * CELL_MM + HALF_CELL_MM - (HALF_CELL_MM - START_OFFSET_MM);
  float stop_mm = speed_mm_s * speed_mm_s / (2.0f * EDGE_TEST_ACCEL_MM_S2);  /* 止まるのに要る距離 */

  suction_ramp_up();  /* 0.5 秒かけて吸引を上げる（sys/suction.c） */
  ui_wait_ms(EDGE_TEST_SUCTION_WAIT_MS);
  run_fast_start();   /* ジャイロのゼロ点を測り直してから、制御を始める */
  log_start(log_interval_of_number(number));

  /* 速くしてそのまま走り（始める位置は、うしろの境目（南の辺）から 35mm）、残りで止まる */
  run_fast_straight(total_mm - stop_mm, speed_mm_s, speed_mm_s, EDGE_TEST_ACCEL_MM_S2, HALF_CELL_MM - START_OFFSET_MM);
  run_fast_stop(stop_mm, speed_mm_s, EDGE_TEST_ACCEL_MM_S2);

  fan_stop();  /* 吸引を止める（フェイルセーフで止まったときは、もう止まっている） */
  signal_end();
}
