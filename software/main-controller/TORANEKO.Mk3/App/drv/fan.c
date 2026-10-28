/*
 * fan.c
 * 吸引ファン（モーター CL-0614-10250-7。TIM2 CH2 の PWM、PA1。N チャネル MOSFET Q5 で、電池からローサイドで回す）
 *
 * ■ PWM
 * TIM2 は 50kHz で 0〜99 を数える（CubeMX で 100MHz を 20 で割り、100 数えて 1 周）。
 * 比べる数（CCR）より小さいあいだ High を出し、そのあいだ MOSFET が ON になってファンに電池の電圧がかかる。
 * CCR を 0 にすると止まり、100 にすると、ずっと ON になる。
 *
 * ■ 電圧で決める
 * ファンには電池の電圧がそのままかかるので、「かけたい電圧 ÷ 電池の電圧」の割合だけ ON にする（モーターと同じ考え方）。
 * 電池の電圧は走っているあいだに下がるので、制御周期ごとに fan_update() で割合を計算し直す（sys/scheduler.c）。
 *
 * ■ 上限
 * ファンのモーター CL-0614-10250-7 の定格は DC 3.7V（無負荷 46000rpm・0.04A 以下、止めたときの電流 1A 以下。
 * 2026-10-28 に作者が教えてくれた仕様）。回しすぎて壊さないように、FAN_LIMIT_V（3.7V）より高い電圧はかけない。
 */
#include "drv/fan.h"

#include "drv/wall.h"
#include "main.h"
#include "tim.h"

#define FAN_PWM_PERIOD   100   /* TIM2 が 1 周で数える数（CubeMX の Period 99 + 1） */
#define FAN_LIMIT_V      3.7f  /* ファンにかける電圧の上限 [V]（CL-0614-10250-7 の定格） */
#define FAN_BATTERY_MIN_V 5.0f /* 電池の電圧がこれより低く読めたら回さない（まだ測れていない、など） */

/* かけたい電圧 [V]。メインの処理で書き、割り込み（fan_update()）で読むので volatile */
static volatile float s_target_v = 0.0f;

/* 起動時に 1 回だけ呼ぶ。止めた状態（CCR 0）にしてから、PWM を出し始める */
void fan_init(void)
{
  s_target_v = 0.0f;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

/* ファンにかける電圧を決める [V]。0 で止まる。上限（FAN_LIMIT_V）より高くはしない */
void fan_set_voltage(float voltage_v)
{
  if (voltage_v < 0.0f) {
    voltage_v = 0.0f;
  }
  if (voltage_v > FAN_LIMIT_V) {
    voltage_v = FAN_LIMIT_V;
  }
  s_target_v = voltage_v;
  fan_update();
}

/* ファンを止める */
void fan_stop(void)
{
  s_target_v = 0.0f;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

/* 今かけたい電圧 [V]（0 なら止まっている） */
float fan_get_voltage(void)
{
  return s_target_v;
}

/* 今の電池の電圧で、ON にする割合を計算し直す。制御周期ごとに呼ぶ（sys/scheduler.c） */
void fan_update(void)
{
  float battery_v = wall_get_battery_v();
  if (s_target_v <= 0.0f || battery_v < FAN_BATTERY_MIN_V) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    return;
  }
  int on_count = s_target_v / battery_v * FAN_PWM_PERIOD + 0.5f;  /* 0.5 を足して、四捨五入する */
  if (on_count > FAN_PWM_PERIOD) {
    on_count = FAN_PWM_PERIOD;
  }
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, on_count);
}
