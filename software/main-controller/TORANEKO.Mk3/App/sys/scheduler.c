/*
 * scheduler.c
 * 制御周期（2kHz）の割り込みで、制御の処理を呼ぶ
 *
 * ■ 仕組み
 * TIM11 は 1MHz（1 カウント = 1µs）で数え、500 まで数えるたびに 0 に戻って割り込みを出す（0.5ms ごと = 2kHz）。
 * 割り込みが来ると、次の順に呼ばれる。
 *   1. TIM1_TRG_COM_TIM11_IRQHandler()（stm32f4xx_it.c。CubeMX が作ったもの）
 *   2. HAL_TIM_IRQHandler(&htim11)（HAL が「何の割り込みか」を調べる）
 *   3. HAL_TIM_PeriodElapsedCallback()（このファイル。「数え終わった」ときに HAL が呼ぶ関数）
 *   4. on_control_tick()（このファイル）→ run_control()（制御の処理）
 * HAL_TIM_PeriodElapsedCallback は、HAL の中に「何もしない仮のもの」があり、同じ名前の関数を書くと
 * こちらが使われる。
 *
 * ■ 処理時間の記録
 * 毎回、TIM5（µs）で「始まった時刻」と「終わった時刻」を読み、かかった時間と、前の回からの間隔を記録する。
 * app.c が 1 秒ごとに scheduler_take_stats() で取り出して、シリアルに出す。
 *
 * ■ 割り込みの中で気をつけること
 * - 割り込みの中では printf や待ち（timebase_wait_ms）を使わない。SysTick（ms を数える割り込み）は
 *   制御周期より優先度が低いので、この中では ms が進まない
 * - 割り込みの中で書き換える変数には volatile を付ける。volatile がないと、コンパイラが
 *   「この変数は変わらない」と考えて、メインの処理で古い値を使い続けることがある
 */
#include "sys/scheduler.h"

#include <stdbool.h>

#include "config/robot_params.h"
#include "drv/encoder.h"
#include "drv/timebase.h"
#include "main.h"
#include "tim.h"

#define TIMER_CLOCK_HZ 1000000  /* TIM11 が数える速さ [Hz]（CubeMX で 100MHz を 100 で割っている） */

/* 割り込みの中で書き換える記録（volatile の理由は先頭のコメント） */
static volatile uint32_t s_tick_count = 0;
static volatile uint32_t s_busy_max_us = 0;
static volatile uint32_t s_period_min_us = 0xFFFFFFFF;  /* まだ測っていないときは、いちばん大きい数にしておく */
static volatile uint32_t s_period_max_us = 0;
static volatile uint32_t s_last_start_us = 0;          /* 前の回が始まった時刻 [µs] */
static volatile bool s_has_last_start = false;         /* 前の回の時刻があるか（最初の 1 回は間隔を測れない） */

/* 制御の処理。制御周期ごとに 1 回呼ばれる（ステップ4 以降で、ジャイロ・壁センサ・モーターの処理を足していく） */
static void run_control(void)
{
  encoder_update();
}

/* 割り込みの間隔を記録する */
static void record_period(uint32_t start_us)
{
  if (s_has_last_start) {
    uint32_t period_us = start_us - s_last_start_us;
    if (period_us < s_period_min_us) {
      s_period_min_us = period_us;
    }
    if (period_us > s_period_max_us) {
      s_period_max_us = period_us;
    }
  }
  s_last_start_us = start_us;
  s_has_last_start = true;
}

/* 制御周期の割り込みが来るたびに呼ばれる。制御の処理を呼び、かかった時間を記録する */
static void on_control_tick(void)
{
  uint32_t start_us = timebase_get_us();
  record_period(start_us);

  run_control();

  uint32_t busy_us = timebase_get_us() - start_us;
  if (busy_us > s_busy_max_us) {
    s_busy_max_us = busy_us;
  }
  s_tick_count = s_tick_count + 1;
}

/* HAL がタイマーの「数え終わった」割り込みで呼ぶ関数。TIM11（制御周期）のときだけ処理する */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11) {
    on_control_tick();
  }
}

/* 制御周期の割り込みを始める。timebase_init() のあとに 1 回だけ呼ぶ */
void scheduler_start(void)
{
  /* 何回数えたら割り込むか（周期）を CTRL_FREQ_HZ から決める。2kHz なら 1000000 ÷ 2000 = 500 回。
   * タイマーは 0 から数えるので、設定する値は 1 を引いた 499 */
  __HAL_TIM_SET_AUTORELOAD(&htim11, TIMER_CLOCK_HZ / CTRL_FREQ_HZ - 1);
  HAL_TIM_Base_Start_IT(&htim11);  /* _IT は「割り込みありで」動かす */
}

/* 前に取り出してからの記録を stats に入れて、記録を 0 から取り直す。メインの処理（app.c）から呼ぶ */
void scheduler_take_stats(scheduler_stats_t *stats)
{
  /* 写しているあいだに割り込みが入ると、半分だけ新しい記録になるので、割り込みを少しのあいだ止める。
   * 止めるのは数行（1µs 未満）なので、止めているあいだに来た割り込みは、再開したときにすぐ処理される */
  __disable_irq();
  stats->tick_count = s_tick_count;
  stats->busy_max_us = s_busy_max_us;
  stats->period_min_us = s_period_min_us;
  stats->period_max_us = s_period_max_us;
  s_tick_count = 0;
  s_busy_max_us = 0;
  s_period_min_us = 0xFFFFFFFF;
  s_period_max_us = 0;
  __enable_irq();
}
