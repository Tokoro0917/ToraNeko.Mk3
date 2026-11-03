/*
 * turn_test.c
 * 最短走行のターン（大回り・斜め）を 1 つずつ走らせて調整する（モード3 No.1〜7。MODES.md）
 *
 * KOGUMA のターン単体テスト（Move.c の Turn1000_Test など）と同じやり方（2026-11-03 に作り直した）：
 *   - ターンは 1 つだけ走る（ほかのターンをつなげない。つなげると、2 つのターンのずれが混ざってしまう）
 *   - 区画のまん中から入るターン（No.1〜4）は、スタート区画に置いて始める（お尻を後ろの壁に付ける。置き方がそろう）
 *   - 斜めから入るターン（No.5〜7）は、機体を斜めに置いて始め、斜め 1 つぶん進んでから曲がる
 * 曲がるのは左だけ（KOGUMA と同じ）。
 *
 * ■ 流れ
 *   1. 吸引を上げて（0.5 秒）、0.5 秒待つ（最短走行は吸引して走るので、ターンも吸引して合わせる。
 *      吸引なしでは 4000mm/s² で前後に滑っていて、500mm/s のターンの横向きの加速度 3000〜5800mm/s² では滑る）
 *   2. ジャイロのゼロ点を測り直す（約 0.5 秒。吸ったまま測る）
 *   3. ターンの速さ（config/turn_params.h。今は 500mm/s）まで速くしながら、ターンの始まりまで進む
 *        No.1〜4：スタート区画から、次の区画のまん中まで（START_OFFSET_MM + 1 区画 = 100mm）
 *        No.5〜7：区画の辺のまん中から、斜めに 1 つぶん（63.6mm）進んで、次の辺のまん中まで
 *   4. 調べるターンを 1 つ走る
 *   5. 止まる：区画のまん中で終わるターンのあとは 1 区画（90mm）、斜めで終わるターンのあとは
 *      斜めの直進 1 つぶん（63.6mm）進みながら止まる
 *   6. 吸引を止める
 * 走行ログは 1ms ごと（1 秒ぶん）。モード7 No.1 で出す。
 *
 * ■ 置き方と、どこで止まれば合っているか
 * 始めた区画を (0, 0)、北を前、東を x の + とする。単位は区画（区画のまん中が整数）。PC で計算した位置。
 *   No.1〜4：(0, 0) のスタート区画に、北向きに、お尻を後ろの壁に付けて置く
 *     No.1 大回り 90°  ：(0, 1) のまん中から曲がり、(−1, 2) のまん中で西を向く → 1 区画進んで (−2, 2) のまん中で止まる
 *     No.2 大回り 180° ：(0, 1) のまん中から曲がり、(−1, 1) のまん中で南を向く → 1 区画進んで (−1, 0) のまん中で止まる
 *     No.3 斜め入り 45°：(0, 2) の西の辺のまん中で北西を向く → 斜めに 1 つ進んで (−1, 2) の北の辺のまん中で止まる
 *     No.4 斜め入り 135°：(−1, 2) の南の辺のまん中で南西を向く → 斜めに 1 つ進んで (−1, 1) の西の辺のまん中で止まる
 *   No.5〜7：(0, 0) の南の辺のまん中に、タイヤの軸を置き、北東（右斜め前 45°）を向けて置く
 *     斜めに 1 つ進んで (0, 0) の東の辺のまん中で、曲がり始める
 *     No.5 斜め出 45°  ：(1, 1) のまん中で北を向く → 1 区画進んで (1, 2) のまん中で止まる
 *     No.6 斜め出 135° ：(0, 1) のまん中で西を向く → 1 区画進んで (−1, 1) のまん中で止まる
 *     No.7 V90°        ：(0, 1) の東の辺のまん中で北西を向く → 斜めに 1 つ進んで (0, 1) の北の辺のまん中で止まる
 * 通るところに壁や柱があるとぶつかるので、迷路の中ではなく、柱だけ（または何もない）の広いところで走らせる。
 * 止まった位置・向きがずれていたら、そのターンの前距離・後距離（config/turn_params.h）を直す。
 */
#include "sys/turn_test.h"

#include <stdbool.h>

#include "config/run_params.h"
#include "config/turn_params.h"
#include "ctrl/run.h"
#include "drv/fan.h"
#include "drv/timebase.h"
#include "sys/failsafe.h"
#include "sys/log.h"
#include "sys/suction.h"
#include "sys/ui.h"

#define TURN_TEST_LOG_INTERVAL_MS 1        /* 走行ログの間隔 [ms]（1000 行で 1 秒ぶん） */
#define TURN_TEST_ACCEL_MM_S2     3000.0f  /* 速くする・止まるときの加速度 [mm/s²]（500mm/s まで 42mm） */
#define TURN_TEST_SUCTION_WAIT_MS 500      /* 吸引を上げてから、走り出す前に待つ時間 [ms]（ファンの回り方を落ち着かせる） */

/* 吸引を上げ、制御とログを始める（まだ動かない） */
static void prepare(void)
{
  suction_ramp_up();  /* 0.5 秒かけて吸引を上げる（sys/suction.c） */
  ui_wait_ms(TURN_TEST_SUCTION_WAIT_MS);
  run_fast_start();   /* ジャイロのゼロ点を測り直してから、制御を始める */
  log_start(TURN_TEST_LOG_INTERVAL_MS);  /* ターンを細かく見たいので、ログは 1ms ごとにし直す */
}

/* No.1〜4：スタート区画から、次の区画のまん中まで進みながら、ターンの速さにする */
static void approach_from_start(void)
{
  run_fast_straight(START_OFFSET_MM + CELL_MM, FAST_TURN_SPEED_MM_S, FAST_TURN_SPEED_MM_S, TURN_TEST_ACCEL_MM_S2);
}

/* No.5〜7：区画の辺のまん中から、斜めに 1 つぶん進みながら、ターンの速さにする */
static void approach_on_diag(void)
{
  run_fast_diag_straight(DIAG_STEP_MM, FAST_TURN_SPEED_MM_S, FAST_TURN_SPEED_MM_S, TURN_TEST_ACCEL_MM_S2);
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

/* No.1〜4（区画のまん中から入るターン）を走る */
static void run_turn_from_cell(int number)
{
  approach_from_start();
  if (number == 1) {
    run_fast_turn(RUN_FAST_TURN_BIG90, true, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  } else if (number == 2) {
    run_fast_turn(RUN_FAST_TURN_BIG180, true, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  } else if (number == 3) {
    run_fast_turn(RUN_FAST_TURN_IN45, true, FAST_TURN_SPEED_MM_S);
    stop_after_diag();
  } else {
    run_fast_turn(RUN_FAST_TURN_IN135, true, FAST_TURN_SPEED_MM_S);
    stop_after_diag();
  }
}

/* No.5〜7（斜めから入るターン）を走る */
static void run_turn_from_diag(int number)
{
  approach_on_diag();
  if (number == 5) {
    run_fast_turn(RUN_FAST_TURN_OUT45, true, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  } else if (number == 6) {
    run_fast_turn(RUN_FAST_TURN_OUT135, true, FAST_TURN_SPEED_MM_S);
    stop_after_cell();
  } else {
    run_fast_turn(RUN_FAST_TURN_V90, true, FAST_TURN_SPEED_MM_S);
    stop_after_diag();
  }
}

/* モード3 No.number のターンを走らせる（流れは先頭のコメント） */
void turn_test_run(int number)
{
  prepare();
  if (number <= 4) {
    run_turn_from_cell(number);
  } else {
    run_turn_from_diag(number);
  }
  fan_stop();  /* 吸引を止める（フェイルセーフで止まったときは、もう止まっている） */
  signal_end();
}
