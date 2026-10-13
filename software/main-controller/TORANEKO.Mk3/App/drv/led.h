/*
 * led.h
 * 表示 LED（LED1〜LED7）の点灯・消灯
 */
#ifndef DRV_LED_H
#define DRV_LED_H

#include <stdbool.h>
#include <stdint.h>

#define LED_COUNT 7  /* 表示 LED の個数 */

void led_init(void);
void led_set(uint8_t num, bool on);
void led_all_off(void);

#endif /* DRV_LED_H */
