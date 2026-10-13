#include "app.h"

#include <stdio.h>

#include "drv/led.h"
#include "drv/timebase.h"
#include "drv/uart.h"

/* ステップ1の動作確認：LED1→LED7 を順に点灯（100ms ごと）し、1 秒ごとに経過時間をシリアルに出す */

#define LED_STEP_MS   100U
#define PRINT_STEP_MS 1000U

static uint32_t s_led_last_ms;
static uint32_t s_print_last_ms;
static uint8_t s_led_num = 1;

void app_init(void)
{
  led_init();
  uart_init();
  printf("\r\nToraNeko.Mk3 boot (SYSCLK %lu Hz)\r\n", (unsigned long)timebase_get_sysclk_hz());
}

void app_loop(void)
{
  uint32_t now = timebase_get_ms();

  if (now - s_led_last_ms >= LED_STEP_MS) {
    s_led_last_ms = now;
    led_set_mask((uint8_t)(1U << (s_led_num - 1)));
    s_led_num = (s_led_num % LED_COUNT) + 1;
  }

  if (now - s_print_last_ms >= PRINT_STEP_MS) {
    s_print_last_ms = now;
    printf("t=%lu ms dropped=%lu\r\n", (unsigned long)now, (unsigned long)uart_get_dropped());
  }
}
