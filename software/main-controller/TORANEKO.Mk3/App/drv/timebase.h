/*
 * timebase.h
 * 時間の取得
 */
#ifndef DRV_TIMEBASE_H
#define DRV_TIMEBASE_H

#include <stdint.h>

uint32_t timebase_get_ms(void);
void timebase_wait_ms(uint32_t wait_ms);
uint32_t timebase_get_sysclk_hz(void);

#endif /* DRV_TIMEBASE_H */
