/*
 * failsafe.c
 * フェイルセーフで止まっているかを覚えておく（異常を見つける部分は、ステップ9で足す）
 */
#include "sys/failsafe.h"

#include "drv/fan.h"
#include "drv/motor.h"

/* フェイルセーフで止まっているか。割り込み（制御周期）の中からも止められるようにするので volatile */
static volatile bool s_stopped = false;

/* フェイルセーフで止まっているとき true */
bool failsafe_is_stopped(void)
{
  return s_stopped;
}

/* フェイルセーフを解除する */
void failsafe_clear(void)
{
  s_stopped = false;
}

/* モーターと吸引ファンを止めて、フェイルセーフで止まった状態にする */
void failsafe_stop(void)
{
  motor_stop();
  fan_stop();
  s_stopped = true;
}
