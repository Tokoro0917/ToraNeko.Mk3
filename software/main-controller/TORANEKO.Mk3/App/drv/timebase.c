/*
 * timebase.c
 * 時間の取得
 *
 * ms 単位は SysTick（HAL_GetTick）、µs 単位は TIM5 で数える。
 * TIM5 は 32 ビットのタイマーで、CubeMX で 1MHz（1 カウント = 1µs）に設定してある（100MHz を 100 で割る）。
 * 0 から 4294967295 まで数えると 0 に戻る（約 71 分）。2 つの時刻の差は「後 − 前」の引き算で求める。
 * uint32_t の引き算は、途中で 0 に戻っていても正しい差になる（入りきらない分を捨てる決まりのため）。
 */
#include "drv/timebase.h"

#include "main.h"
#include "tim.h"

/* µs を数える TIM5 を動かし始める。起動時に 1 回だけ呼ぶ */
void timebase_init(void)
{
  HAL_TIM_Base_Start(&htim5);
}

/* 起動してからの経過時間 [ms]。SysTick（1ms ごとの割り込み）で数えている */
uint32_t timebase_get_ms(void)
{
  return HAL_GetTick();
}

/* timebase_init() を呼んでからの経過時間 [µs]。約 71 分で 0 に戻る */
uint32_t timebase_get_us(void)
{
  return __HAL_TIM_GET_COUNTER(&htim5);  /* TIM5 の今の数（カウンタ）を読む */
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
