/*
 * app.c
 * App 全体の入口。main.c から app_init() と app_loop() が呼ばれる
 *
 * 今の中身はステップ1の動作確認：
 *   - LED1 → LED2 → … → LED7 → LED1 … の順に、0.1 秒ごとに 1 個ずつ点灯する
 *   - 1 秒ごとに経過時間をシリアルに出す
 */
#include "app.h"

#include <stdio.h>

#include "drv/led.h"
#include "drv/timebase.h"
#include "drv/uart.h"

#define LED_INTERVAL_MS   100   /* LED を次に進める間隔 [ms] */
#define PRINT_INTERVAL_MS 1000  /* シリアルに出す間隔 [ms] */

static uint32_t s_last_led_ms = 0;    /* 最後に LED を進めた時刻 [ms] */
static uint32_t s_last_print_ms = 0;  /* 最後にシリアルに出した時刻 [ms] */
static uint8_t s_led_num = 1;         /* 今点けている LED の番号（1〜7） */

/* 起動時に 1 回だけ呼ばれる */
void app_init(void)
{
  led_init();
  uart_init();
  printf("\r\nToraNeko.Mk3 boot (SYSCLK %lu Hz)\r\n", timebase_get_sysclk_hz());
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

  /* 経過時間をシリアルに出す */
  if (now_ms - s_last_print_ms >= PRINT_INTERVAL_MS) {
    s_last_print_ms = now_ms;
    printf("t=%lu ms dropped=%lu\r\n", now_ms, uart_get_dropped_count());
  }
}
