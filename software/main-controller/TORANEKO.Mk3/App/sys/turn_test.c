/*
 * turn_test.c
 * 最短走行のターン（大回り・斜め）を 1 つずつ走らせて調整する（モード3 No.1〜7。MODES.md）。
 * 斜めの直進だけを走らせて、斜めの補正を調整する（モード3 No.8・9。下の「■ 斜めの直進のテスト」）
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
 *   3. そのターンの速さ（config/turn_params.h の *_SPEED_MM_S。組 1 は、ゆるいターン 800mm/s、きついターン 600mm/s。
 *      組 2 は 7 つとも 1000mm/s。どちらの組かは config/turn_params.h の TURN_TEST_TURN_SET で選ぶ）まで
 *      速くしながら、ターンの始まりまで進む
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
 * ターンの調整（No.1〜7）では、斜めの補正（ctrl/control.c）を入れない（補正で曲がると、ターンのずれと混ざるため）。
 *
 * ■ 斜めの直進のテスト（No.8・9。2026-11-04 に足した）
 * 斜めの通り道（階段のような形に壁を並べたもの）で、斜めに DIAG_TEST_STEPS つぶん（6 つ、382mm）走って止まる。
 * 速さは最短走行 No.4 の斜めの直進と同じ 800mm/s。吸引する（1 と同じ）。
 *   No.8：斜めの補正を入れない。走行ログに前のセンサの値が残るので、まん中を走ったときの値から、しきい値を決める
 *   No.9：斜めの補正を入れる。柱に近づいたら離れるかを見る
 * 置き方は No.5〜7 と同じ（(0, 0) の南の辺のまん中にタイヤの軸を置き、北東を向ける）。通り道は、
 * (0, 0)・(1, 0)・(1, 1)・(2, 1)・(2, 2)・(3, 2)・(3, 3) の 7 区画。通り道が通る辺（(0, 0) の南（置くところ）・
 * (0, 0) の東・(1, 0) の北・(1, 1) の東・(2, 1) の北・(2, 2) の東・(3, 2) の北）には壁を置かず、7 区画のほかの辺には
 * 全部壁を置く。止まるのは (3, 2) の北の辺のまん中。
 * 迷路の外の壁（南の端）にお尻が当たるので、迷路の角ではなく、まん中のあいているところに作る。
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
#define TURN_TEST_ACCEL_MM_S2     8000.0f  /* 速くする・止まるときの加速度 [mm/s²]（1000mm/s まで 62.5mm。斜め 1 つぶん 63.6mm で
                                              1000mm/s〈組 2〉に上げて止めるため。2026-11-10 に 6000 から上げた。
                                              吸引 5.0V では 10000mm/s² まで滑らない） */
#define TURN_TEST_SUCTION_WAIT_MS 500      /* 吸引を上げてから、走り出す前に待つ時間 [ms]（ファンの回り方を落ち着かせる） */
#define DIAG_TEST_STEPS           6        /* 斜めの直進のテスト（No.8・9）で進む、斜めの直進の数（6 つで 382mm） */
#define DIAG_TEST_SPEED_MM_S      SHORTEST4_DIAG_MAX_SPEED_MM_S  /* 同じく、速さ [mm/s]（最短走行 No.4 の斜めと同じ 800） */

/* 吸引を上げ、制御とログを始める（まだ動かない） */
static void prepare(void)
{
  run_fast_set_turn_set(TURN_TEST_TURN_SET);  /* 走らせる組（config/turn_params.h） */
  run_fast_set_diag_correction(false);  /* ターンの調整では、斜めの補正を入れない（先頭のコメント） */
  suction_ramp_up_to(TURN_TEST_SUCTION_V);  /* 0.5 秒かけて吸引を上げる（sys/suction.c。電圧は config/turn_params.h） */
  ui_wait_ms(TURN_TEST_SUCTION_WAIT_MS);
  run_fast_start();   /* ジャイロのゼロ点を測り直してから、制御を始める */
  log_start(TURN_TEST_LOG_INTERVAL_MS);  /* ターンを細かく見たいので、ログは 1ms ごとにし直す */
}

/* No.1〜4：スタート区画から、次の区画のまん中まで進みながら、速さ speed_mm_s にする。
 * 迷路の外の広いところで走るので、横の壁に沿う補正は入れない（run_fast_diag_straight は、横の壁に沿う補正を入れない直進。
 * 斜めの補正も、prepare() で切ってある。走行ログでは、この直進も「斜めの直線」（phase 2）になり、前のセンサの値が入る）。
 * 2026-11-04 に、補正を入れる run_fast_straight から変えた（止まるときに、横のセンサが何かを見て補正が入り、0.5° 回った） */
static void approach_from_start(float speed_mm_s)
{
  run_fast_diag_straight(START_OFFSET_MM + CELL_MM, speed_mm_s, speed_mm_s, TURN_TEST_ACCEL_MM_S2);
}

/* No.5〜7：区画の辺のまん中から、斜めに 1 つぶん進みながら、速さ speed_mm_s にする */
static void approach_on_diag(float speed_mm_s)
{
  run_fast_diag_straight(DIAG_STEP_MM, speed_mm_s, speed_mm_s, TURN_TEST_ACCEL_MM_S2);
}

/* 区画のまん中から、1 区画進みながら止まる（速さ speed_mm_s から）。横の壁に沿う補正は入れない（approach_from_start と同じ） */
static void stop_after_cell(float speed_mm_s)
{
  run_fast_diag_stop(CELL_MM, speed_mm_s, TURN_TEST_ACCEL_MM_S2);
}

/* 斜めの上（壁の辺のまん中）から、斜めの直進 1 つぶん進みながら止まる（速さ speed_mm_s から） */
static void stop_after_diag(float speed_mm_s)
{
  run_fast_diag_stop(DIAG_STEP_MM, speed_mm_s, TURN_TEST_ACCEL_MM_S2);
}

/* ターン turn を、そのターンの速さで、左に 1 つ走る */
static void run_one_turn(run_fast_turn_t turn)
{
  run_fast_turn(turn, true, run_fast_turn_speed_mm_s(turn));
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

/* No. で、調べるターンを決める（1 大回り 90°、2 大回り 180°、3 斜め入り 45°、4 斜め入り 135°、
 * 5 斜め出 45°、6 斜め出 135°、7 V90°） */
static run_fast_turn_t turn_of_number(int number)
{
  if (number == 1) {
    return RUN_FAST_TURN_BIG90;
  }
  if (number == 2) {
    return RUN_FAST_TURN_BIG180;
  }
  if (number == 3) {
    return RUN_FAST_TURN_IN45;
  }
  if (number == 4) {
    return RUN_FAST_TURN_IN135;
  }
  if (number == 5) {
    return RUN_FAST_TURN_OUT45;
  }
  if (number == 6) {
    return RUN_FAST_TURN_OUT135;
  }
  return RUN_FAST_TURN_V90;
}

/* ターン turn が、斜めの上（壁の辺のまん中）で終わるか（斜め入り 45°・135°、V90°）。
 * そうでなければ、区画のまん中で終わる（大回り、斜め出） */
static bool ends_on_diag(run_fast_turn_t turn)
{
  if (turn == RUN_FAST_TURN_IN45 || turn == RUN_FAST_TURN_IN135 || turn == RUN_FAST_TURN_V90) {
    return true;
  }
  return false;
}

/* No.8・9：斜めの直進のテストを走らせる（先頭のコメントの「■ 斜めの直進のテスト」）。
 * use_correction が true なら斜めの補正を入れる（No.9）、false なら入れない（No.8） */
static void run_diag_test(bool use_correction)
{
  prepare();
  run_fast_set_diag_correction(use_correction);
  run_fast_diag_stop(DIAG_TEST_STEPS * DIAG_STEP_MM, DIAG_TEST_SPEED_MM_S, TURN_TEST_ACCEL_MM_S2);
  run_fast_set_diag_correction(true);  /* ふつうの「入れる」に戻す（最短走行のため） */
  run_fast_set_turn_set(1);            /* 組 1 に戻す */
  fan_stop();
  signal_end();
}

/* モード3 No.number のターンを走らせる（流れは先頭のコメント）。No.8・9 は斜めの直進のテスト */
void turn_test_run(int number)
{
  if (number == 8) {
    run_diag_test(false);
    return;
  }
  if (number == 9) {
    run_diag_test(true);
    return;
  }

  run_fast_turn_t turn = turn_of_number(number);
  float speed_mm_s = run_fast_turn_speed_mm_s(turn);

  prepare();

  /* ターンの始まりまで進む（No.1〜4 は区画のまん中から、No.5〜7 は斜めから入る） */
  if (number <= 4) {
    approach_from_start(speed_mm_s);
  } else {
    approach_on_diag(speed_mm_s);
  }

  run_one_turn(turn);

  /* 止まる */
  if (ends_on_diag(turn)) {
    stop_after_diag(speed_mm_s);
  } else {
    stop_after_cell(speed_mm_s);
  }

  fan_stop();  /* 吸引を止める（フェイルセーフで止まったときは、もう止まっている） */
  run_fast_set_diag_correction(true);  /* ふつうの「入れる」に戻す（最短走行のため） */
  run_fast_set_turn_set(1);            /* 組 1 に戻す */
  signal_end();
}
