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

/* wait_ms [ms] のあいだ待つ（そのあいだ、ほかのことはしない）。
 * HAL_Delay() は SysTick の数を見て待つ。割り込み（制御周期など）は、待っているあいだも動く */
void timebase_wait_ms(uint32_t wait_ms)
{
  HAL_Delay(wait_ms);
}

/* マイコンのクロック周波数 [Hz]。クロック設定が正しければ 100000000（100MHz） */
uint32_t timebase_get_sysclk_hz(void)
{
  return HAL_RCC_GetSysClockFreq();
}
