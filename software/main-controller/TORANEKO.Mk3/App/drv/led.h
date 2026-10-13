#ifndef DRV_LED_H
#define DRV_LED_H

#include <stdbool.h>
#include <stdint.h>

#define LED_COUNT 7  /* 表示 LED（LED1〜LED7） */

void led_init(void);                     /* 全消灯 */
void led_set(uint8_t num, bool on);      /* num = 1〜LED_COUNT（基板の LED 番号）。範囲外は無視 */
void led_set_mask(uint8_t mask);         /* bit0 = LED1 … bit6 = LED7 をまとめて設定 */

#endif /* DRV_LED_H */
