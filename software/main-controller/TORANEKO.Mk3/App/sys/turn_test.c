/*
 * turn_test.c
 * 最短走行のターン（大回り・斜め）を 1 つずつ走らせて調整する（モード3 No.1〜7。MODES.md）
 *
 * どの No. も、区画のまん中に北向きに置いて始める（曲がるのは左だけ。KOGUMA と同じ）。
 *   1. ジャイロのゼロ点を測り直す（約 0.5 秒）
 *   2. 1 区画（90mm）走って、ターンの速さ（config/turn_params.h。今は 500mm/s）まで速くする。次の区画のまん中に着く
 *   3. 調べるターンを走る（斜め出・V90 は、その前に右の斜め入り 45° で斜めに入る）
 *   4. 止まる：区画のまん中で終わるターンのあとは 1 区画（90mm）、斜めで終わるターンのあとは
 *      斜めの直進 1 つぶん（63.6mm）進みながら止まる
 * 走行ログは 1ms ごと（1 秒ぶん）。モード7 No.1 で出す。
 *
 * ■ どこで止まれば合っているか（始めた区画を (0, 0)、北を前、東を x の + とする。単位は区画。PC で計算した位置）
 *   No.1 大回り 90°  ：(0, 1) のまん中から曲がり、(−1, 2) のまん中で西を向く → 1 区画進んで (−2, 2) のまん中で止まる
 *   No.2 大回り 180° ：(0, 1) のまん中から曲がり、(−1, 1) のまん中で南を向く → 1 区画進んで (−1, 0) のまん中で止まる
 *   No.3 斜め入り 45°：(0, 2) の西の辺のまん中で北西を向く → 斜めに 1 つ進んで (−1, 2) の北の辺のまん中で止まる
 *   No.4 斜め入り 135°：(−1, 2) の南の辺のまん中で南西を向く → 斜めに 1 つ進んで (−1, 1) の西の辺のまん中で止まる
 *   No.5 斜め出 45°  ：右の斜め入り 45° → 左の斜め出 45° で (1, 3) のまん中で北を向く → 1 区画進んで (1, 4) で止まる
 *   No.6 斜め出 135° ：右の斜め入り 45° → 左の斜め出 135° で (0, 3) のまん中で西を向く → 1 区画進んで (−1, 3) で止まる
 *   No.7 V90°        ：右の斜め入り 45° → 左の V90 → 右の斜め出 45° で (0, 4) のまん中で北を向く → 1 区画進んで (0, 5) で止まる
 * 通るところに壁や柱があるとぶつかるので、迷路の中ではなく、柱だけ（または何もない）の広いところで走らせる。
 * 止まった位置・向きがずれていたら、そのターンの前距離・後距離（config/turn_params.h）を直す。
 */
#include "sys/turn_test.h"

#include <stdbool.h>

#include "config/run_params.h"
#include "config/turn_params.h"
#include "ctrl/run.h"
#include "drv/timebase.h"
#include "sys/failsafe.h"
#include "sys/log.h"
#include "sys/ui.h"

#define TURN_TEST_LOG_INTERVAL_MS 1     /* 走行ログの間隔 [ms]（1000 行で 1 秒ぶん） */
#define TURN_TEST_ACCEL_MM_S2     3000.0f  /* 速くする・止まるときの加速度 [mm/s²]（500mm/s まで 42mm） */

/* 始める：制御とログを始めて、1 区画走ってターンの速さにする（次の区画のまん中に着く） */
static void approach(void)
{
  run_fast_start();
  log_start(TURN_TEST_LOG_INTERVAL_MS);  /* ターンを細かく見たいので、ログは 1ms ごとにし直す */
  run_fast_straight(CELL_MM, FAST_TURN_SPEED_MM_S, FAST_TURN_SPEED_MM_S, TURN_TEST_ACCEL_MM_S2);
}

/* 区画のまん中から、1 区画進みながら止まる */
static void stop_after_cell(void)
{
  run_fast_stop(CELL_MM, FAST_TURN_SPEED_MM_S, TURN_TEST_ACCEL_MM_S2);
}

/* 斜めの上（壁の辺のまん中）から、斜めの直進 1 つぶん進みながら止まる */
static void stop_after_diag(void)
{
  run_fast_diag_stop(DIAG_STEP_MM, FAST_TURN_SPEED_MM_S, TURN_TEST_ACCEL_MM_S2);
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

/* モード3 No.number のターンを走らせる（流れは先頭のコメント） */
void turn_test_run(int number)
{
  approach();
  if (number == 1) {
    run_fast_turn(RUN_FAST_TURN_BIG90, true, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  } else if (number == 2) {
    run_fast_turn(RUN_FAST_TURN_BIG180, true, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  } else if (number == 3) {
    run_fast_turn(RUN_FAST_TURN_IN45, true, FAST_TURN_SPEED_MM_S);
    stop_after_diag();
  } else if (number == 4) {
    run_fast_turn(RUN_FAST_TURN_IN135, true, FAST_TURN_SPEED_MM_S);
    stop_after_diag();
  } else if (number == 5) {
    run_fast_turn(RUN_FAST_TURN_IN45, false, FAST_TURN_SPEED_MM_S);
    run_fast_turn(RUN_FAST_TURN_OUT45, true, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  } else if (number == 6) {
    run_fast_turn(RUN_FAST_TURN_IN45, false, FAST_TURN_SPEED_MM_S);
    run_fast_turn(RUN_FAST_TURN_OUT135, true, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  } else {
    run_fast_turn(RUN_FAST_TURN_IN45, false, FAST_TURN_SPEED_MM_S);
    run_fast_turn(RUN_FAST_TURN_V90, true, FAST_TURN_SPEED_MM_S);
    run_fast_turn(RUN_FAST_TURN_OUT45, false, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  }
  signal_end();
}
