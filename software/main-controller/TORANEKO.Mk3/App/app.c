/*
 * app.c
 * App 全体の入口。main.c から app_init() と app_loop() が呼ばれる
 *
 * 今の中身はステップ1〜3の動作確認：
 *   - LED1 → LED2 → … → LED7 → LED1 … の順に、0.1 秒ごとに 1 個ずつ点灯する
 *   - 1 秒ごとに、経過時間と、制御周期（2kHz）の割り込みの記録をシリアルに出す
 *     tick=1 秒間に割り込みが来た回数（2000 のはず）、busy=1 回の処理にかかった最大の時間、
 *     period=割り込みの間隔の最小〜最大（500µs のはず）
 *   - 0.2 秒ごとに、左右のエンコーダの値をシリアルに出す
 *     angle=今の角度（0〜16383）、total=電源を入れてから回った量（前に進む向きが +）、err=エラーの回数
 */
#include "app.h"

#include <stdio.h>

#include "drv/encoder.h"
#include "drv/led.h"
#include "drv/timebase.h"
#include "drv/uart.h"
#include "sys/scheduler.h"

#define LED_INTERVAL_MS   100   /* LED を次に進める間隔 [ms] */
#define PRINT_INTERVAL_MS 1000  /* 制御周期の記録をシリアルに出す間隔 [ms] */
#define ENCODER_PRINT_INTERVAL_MS 200  /* エンコーダの値をシリアルに出す間隔 [ms] */

static uint32_t s_last_led_ms = 0;    /* 最後に LED を進めた時刻 [ms] */
static uint32_t s_last_print_ms = 0;  /* 最後に制御周期の記録を出した時刻 [ms] */
static uint32_t s_last_encoder_print_ms = 0;  /* 最後にエンコーダの値を出した時刻 [ms] */
static uint8_t s_led_num = 1;         /* 今点けている LED の番号（1〜7） */

/* 起動時に 1 回だけ呼ばれる */
void app_init(void)
{
  led_init();
  uart_init();
  timebase_init();
  printf("\r\nToraNeko.Mk3 boot (SYSCLK %lu Hz)\r\n", timebase_get_sysclk_hz());
  encoder_init();
  scheduler_start();
}

/* main() の while(1) から繰り返し呼ばれる */
void app_loop(void)
{
  uint32_t now_ms = timebase_get_ms();

  /* LED を 1 つ進める */
  if (now_ms - s_last_led_ms >= LED_INTERVAL_MS) {
    s_last_led_ms = now_ms;

    led_all_off();
    led_set(s_led_num, true);

    s_led_num = s_led_num + 1;
    if (s_led_num > LED_COUNT) {
      s_led_num = 1;
    }
  }

  /* エンコーダの値をシリアルに出す */
  if (now_ms - s_last_encoder_print_ms >= ENCODER_PRINT_INTERVAL_MS) {
    s_last_encoder_print_ms = now_ms;
    printf("L angle=%5u total=%7ld  R angle=%5u total=%7ld  err=%lu\r\n",
           encoder_get_left_angle(), encoder_get_left_total(),
           encoder_get_right_angle(), encoder_get_right_total(),
           encoder_get_error_count());
  }

  /* 経過時間と、制御周期の割り込みの記録をシリアルに出す */
  if (now_ms - s_last_print_ms >= PRINT_INTERVAL_MS) {
    s_last_print_ms = now_ms;
    scheduler_stats_t stats;
    scheduler_take_stats(&stats);
    printf("t=%lu ms tick=%lu busy=%lu us period=%lu-%lu us dropped=%lu\r\n",
           now_ms, stats.tick_count, stats.busy_max_us,
           stats.period_min_us, stats.period_max_us, uart_get_dropped_count());
  }
}
