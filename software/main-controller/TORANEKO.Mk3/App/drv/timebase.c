/*
 * timebase.c
 * 時間の取得
 */
#include "drv/timebase.h"

#include "main.h"

/* 起動してからの経過時間 [ms]。SysTick（1ms ごとの割り込み）で数えている */
uint32_t timebase_get_ms(void)
{
  return HAL_GetTick();
}

/* マイコンのクロック周波数 [Hz]。クロック設定が正しければ 100000000（100MHz） */
uint32_t timebase_get_sysclk_hz(void)
{
  return HAL_RCC_GetSysClockFreq();
}
