/*
 * led.c
 * 表示 LED（LED1〜LED7）の点灯・消灯
 *
 * ピン名（LED1_GPIO_Port、LED1_Pin など）は CubeMX で付けたラベルで、main.h に定義されている。
 */
#include "drv/led.h"

#include "main.h"

/* 起動時に呼ぶ。全部消しておく */
void led_init(void)
{
  led_all_off();
}

/* LED を 1 個点ける／消す。num は基板の LED 番号（1〜7）。on が true なら点灯 */
void led_set(uint8_t num, bool on)
{
  GPIO_PinState state;
  if (on) {
    state = GPIO_PIN_SET;
  } else {
    state = GPIO_PIN_RESET;
  }

  if (num == 1) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state);
  } else if (num == 2) {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state);
  } else if (num == 3) {
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, state);
  } else if (num == 4) {
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, state);
  } else if (num == 5) {
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, state);
  } else if (num == 6) {
    HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, state);
  } else if (num == 7) {
    HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, state);
  }
  /* 1〜7 以外は何もしない */
}

/* 全部の LED を消す */
void led_all_off(void)
{
  for (uint8_t num = 1; num <= LED_COUNT; num++) {
    led_set(num, false);
  }
}
