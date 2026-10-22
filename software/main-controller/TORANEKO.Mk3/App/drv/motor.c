/*
 * motor.c
 * 走行モーターの出力（MP6551 ×2、TIM4 の PWM）
 *
 * ■ PWM
 * TIM4 は 100kHz で 0〜99 を数える（CubeMX で 100MHz を 10 で割り、100 数えて 1 周）。
 * 各チャンネルの比べる数（CCR）より小さいあいだ High、それ以外は Low を出す（PWM1 モード）。
 * CCR を 100 にすると、ずっと High になる。
 *
 * ■ MP6551 への出し方（KOGUMA の motor.c と同じ）
 * 1 つのモーターに入力が 2 本（IN1・IN2）あり、次のように動く。
 *   IN1 = High、IN2 = High：ブレーキ（モーターの両端をつなぐ）
 *   IN1 = Low、 IN2 = High：一方向に回す
 *   IN1 = High、IN2 = Low ：反対向きに回す
 * 片方を High のままにして、もう片方を「Low の時間の割合 = かけたい電圧 ÷ 電池の電圧」で PWM する。
 * Low でないあいだはブレーキになる（減速がなめらかになる）。
 *
 * ■ ピン
 *   左：IN1 = PB6（CH1）、IN2 = PB7（CH2）
 *   右：IN1 = PB8（CH3）、IN2 = PB9（CH4）
 *
 * ■ 向き
 * どちら向きが「機体が前に進む向き」かは、モーターの配線で決まる。前に進むときに + になるように、
 * MOTOR_*_SIGN で合わせる（実機で確かめる）。
 */
#include "drv/motor.h"

#include "drv/wall.h"
#include "main.h"
#include "tim.h"

#define MOTOR_LEFT_SIGN  1     /* 左：+ の電圧で前に回るなら 1、後ろに回るなら -1（実機で確かめる） */
#define MOTOR_RIGHT_SIGN 1     /* 右：同じく */
#define PWM_PERIOD       100   /* TIM4 が 1 周で数える数（CubeMX の Period 99 + 1） */
#define MOTOR_LIMIT_V    3.7f  /* モーターにかける電圧の上限 [V]（NFP-D0812-1-3.7 の定格） */
#define BATTERY_MIN_V    5.0f  /* 電池の電圧がこれより低く読めたら回さない（まだ測れていない、など） */

/* 起動時に 1 回だけ呼ぶ。4 本とも High（ブレーキ）にしてから、PWM を出し始める */
void motor_init(void)
{
  motor_stop();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/* 電圧 voltage_v を、PWM で Low にする数（0〜PWM_PERIOD）に直す。上限を超えないようにする */
static int voltage_to_low_count(float voltage_v, float battery_v)
{
  if (voltage_v > MOTOR_LIMIT_V) {
    voltage_v = MOTOR_LIMIT_V;
  }
  int count = voltage_v / battery_v * PWM_PERIOD + 0.5f;  /* 0.5 を足して、四捨五入する */
  if (count > PWM_PERIOD) {
    count = PWM_PERIOD;
  }
  return count;
}

/* 1 つのモーターの 2 本の入力に出す。voltage_v が + なら IN1 側、- なら IN2 側を PWM する */
static void set_one_motor(uint32_t in1_channel, uint32_t in2_channel, float voltage_v, float battery_v)
{
  if (voltage_v >= 0.0f) {
    int low_count = voltage_to_low_count(voltage_v, battery_v);
    __HAL_TIM_SET_COMPARE(&htim4, in1_channel, PWM_PERIOD - low_count);
    __HAL_TIM_SET_COMPARE(&htim4, in2_channel, PWM_PERIOD);
  } else {
    int low_count = voltage_to_low_count(-voltage_v, battery_v);
    __HAL_TIM_SET_COMPARE(&htim4, in1_channel, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim4, in2_channel, PWM_PERIOD - low_count);
  }
}

/* 左右のモーターに電圧をかける [V]。+ が前に進む向き。電池の電圧が変わっても同じ電圧になるように、
 * 今の電池の電圧で割って PWM の割合を決める */
void motor_set_voltage(float left_v, float right_v)
{
  float battery_v = wall_get_battery_v();
  if (battery_v < BATTERY_MIN_V) {
    motor_stop();
    return;
  }
  set_one_motor(TIM_CHANNEL_1, TIM_CHANNEL_2, MOTOR_LEFT_SIGN * left_v, battery_v);
  set_one_motor(TIM_CHANNEL_3, TIM_CHANNEL_4, MOTOR_RIGHT_SIGN * right_v, battery_v);
}

/* モーターを止める（4 本とも High にして、ブレーキをかける） */
void motor_stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_PERIOD);
}
