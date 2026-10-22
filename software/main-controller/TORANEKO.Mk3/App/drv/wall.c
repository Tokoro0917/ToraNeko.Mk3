/*
 * wall.c
 * 壁センサ（受光 LTR-209 ×4、発光 2 組）の AD 値の読み取りと、電池の電圧
 *
 * ■ 測定の流れ（README の決定事項「TIM3 で ADC を起動し、DMA 完了割り込みで次の LED に進める連鎖」）
 * LED を点けてから受光の値が落ち着くまで、約 25µs かかる（LTR-209 の立ち上がり 10µs・立ち下がり 15µs）。
 * CPU で待つと、その間ほかの処理ができないので、待つのはタイマーに任せる。
 *   1. wall_start_measure()：AD 変換の準備（HAL_ADC_Start_DMA）をして、TIM3 を動かす（最初は LED を全部消したまま）
 *   2. TIM3 は 35µs 数えると止まり、そのとき AD 変換を始める合図（TRGO）を出す（CubeMX で設定済み）
 *   3. AD 変換は 5 つの入力（電池、受光 4〜1）を順に変換し、DMA が結果を s_adc_buffer に書く（約 5µs）
 *   4. 書き終わると DMA の割り込みが来て、HAL が HAL_ADC_ConvCpltCallback()（このファイル）を呼ぶ
 *   5. 結果を写し、次の LED に切り替えて、また 1 の「準備して TIM3 を動かす」をする
 * これを次の 4 回くり返して終わる（1 回 約 40µs、全部で約 0.16ms）。
 *   1. 全部消したまま 35µs 待って測る（消したときの値）
 *   2. 組A を点けて 35µs 待って測る
 *   3. 組A を消して 35µs 待つ（変換はするが、結果は使わない）
 *   4. 組B を点けて 35µs 待って測り、組B を消す
 *
 * - 全部消して測るのを最初にするのは、LED を消したあとも受光の値がすぐには下がりきらず（35µs では足りない）、
 *   LED のあとに測ると「消したときの値」が大きく出てしまうため（2026-10-22 に、ON−OFF がマイナスになって気づいた）。
 *   最初なら、前の周期の組B を消してから 0.3ms 以上たっているので、下がりきっている
 * - 組A と組B のあいだに「全部消して待つだけ」を入れるのも同じ理由。組A の光が受光 3・4 に残ったまま
 *   組B を測らないように、組A を消してから組B を点けるまで約 40µs あける（2026-10-22 に追加。結果は使わない）
 *
 * ■ 割り込みの優先度
 * DMA の割り込み（優先度 1）は制御周期の割り込み（優先度 2）より強いので、制御の処理の途中でも測定は進む。
 *
 * ■ DMA
 * DMA は、CPU を使わずに、周辺機能（ここでは AD 変換）の結果をメモリに写す仕組み。
 *
 * ■ 受光の番号と AD 変換の順番（CUBEMX_SETUP.md の 5）
 *   s_adc_buffer[0]：電池（PA2）
 *   s_adc_buffer[1]：受光 4（PA3）
 *   s_adc_buffer[2]：受光 3（PA4）
 *   s_adc_buffer[3]：受光 2（PA5）
 *   s_adc_buffer[4]：受光 1（PA6）
 */
#include "drv/wall.h"

#include <stdbool.h>

#include "adc.h"
#include "main.h"
#include "tim.h"

#define ADC_CHANNEL_COUNT 5  /* AD 変換する入力の数（電池 + 受光 4） */
#define ADC_FULL_SCALE    4095.0f
#define ADC_VREF_V        3.3f
#define BATTERY_DIVIDER   (147.0f / 47.0f)  /* 電池の電圧を 100kΩ と 47kΩ で分けて測っているので、(100 + 47) / 47 倍する */

/* 今どの測定をしているか */
typedef enum {
  STAGE_IDLE = 0,  /* 測っていない */
  STAGE_DARK,      /* 全部消して測っている */
  STAGE_LED_A,     /* 組A を点けて測っている */
  STAGE_GAP,       /* 組A を消して、組B を点けるまで待っている（測った結果は使わない） */
  STAGE_LED_B,     /* 組B を点けて測っている */
} wall_stage_t;

/* DMA が AD 変換の結果を書く場所 */
static uint16_t s_adc_buffer[ADC_CHANNEL_COUNT];

/* 割り込みの中で書き換える値（volatile の理由は sys/scheduler.c の先頭のコメント）。
 * 添字は受光の番号（1〜4）。0 は使わない */
static volatile uint16_t s_lit_a[WALL_SENSOR_COUNT + 1];
static volatile uint16_t s_lit_b[WALL_SENSOR_COUNT + 1];
static volatile uint16_t s_dark[WALL_SENSOR_COUNT + 1];
static volatile uint16_t s_battery_raw = 0;
static volatile wall_stage_t s_stage = STAGE_IDLE;
static volatile uint32_t s_overrun_count = 0;

/* 組A・組B の LED を点ける（true）・消す（false） */
static void set_led_a(bool on)
{
  if (on) {
    HAL_GPIO_WritePin(WALL_LED_A_GPIO_Port, WALL_LED_A_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(WALL_LED_A_GPIO_Port, WALL_LED_A_Pin, GPIO_PIN_RESET);
  }
}

static void set_led_b(bool on)
{
  if (on) {
    HAL_GPIO_WritePin(WALL_LED_B_GPIO_Port, WALL_LED_B_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(WALL_LED_B_GPIO_Port, WALL_LED_B_Pin, GPIO_PIN_RESET);
  }
}

/* AD 変換の準備をして、TIM3 を動かす。35µs 後に TIM3 の合図で AD 変換が始まる */
static void start_conversion_after_wait(void)
{
  /* HAL の関数は、結果を書く場所を uint32_t の番地として受け取る決まりなので、その形に直して渡す */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_buffer, ADC_CHANNEL_COUNT);
  __HAL_TIM_ENABLE(&htim3);  /* TIM3 を数え始めさせる（1 回数えると自分で止まる設定） */
}

/* 受光 4 つの結果を、決めた配列に写す */
static void copy_sensors(volatile uint16_t *values)
{
  values[1] = s_adc_buffer[4];
  values[2] = s_adc_buffer[3];
  values[3] = s_adc_buffer[2];
  values[4] = s_adc_buffer[1];
}

/* 起動時に 1 回だけ呼ぶ。LED を消しておく */
void wall_init(void)
{
  set_led_a(false);
  set_led_b(false);
  s_stage = STAGE_IDLE;
}

/* 制御周期ごとに呼ぶ。LED を全部消したまま、測定の 1 回目を始める */
void wall_start_measure(void)
{
  if (s_stage != STAGE_IDLE) {
    s_overrun_count = s_overrun_count + 1;  /* 前の測定がまだ終わっていない */
    return;
  }
  s_stage = STAGE_DARK;
  start_conversion_after_wait();
}

/* AD 変換の結果を DMA が書き終えたときに、HAL が呼ぶ関数（流れは先頭のコメント） */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) {
    return;
  }
  HAL_ADC_Stop_DMA(&hadc1);  /* 次の準備のために、いったん止める */
  s_battery_raw = s_adc_buffer[0];

  if (s_stage == STAGE_DARK) {
    copy_sensors(s_dark);
    set_led_a(true);
    s_stage = STAGE_LED_A;
    start_conversion_after_wait();
  } else if (s_stage == STAGE_LED_A) {
    copy_sensors(s_lit_a);
    set_led_a(false);
    s_stage = STAGE_GAP;
    start_conversion_after_wait();  /* 全部消したまま待つ（TIM3 で待つので、もう 1 回変換するが、結果は使わない） */
  } else if (s_stage == STAGE_GAP) {
    set_led_b(true);
    s_stage = STAGE_LED_B;
    start_conversion_after_wait();
  } else if (s_stage == STAGE_LED_B) {
    copy_sensors(s_lit_b);
    set_led_b(false);
    s_stage = STAGE_IDLE;  /* 4 回とも終わった */
  }
}

/* 受光 num（1〜4）の、組A を点けたときの AD 値 */
uint16_t wall_get_lit_a(int num)
{
  return s_lit_a[num];
}

/* 受光 num（1〜4）の、組B を点けたときの AD 値 */
uint16_t wall_get_lit_b(int num)
{
  return s_lit_b[num];
}

/* 受光 num（1〜4）の、全部消したときの AD 値 */
uint16_t wall_get_dark(int num)
{
  return s_dark[num];
}

/* 受光 num（1〜4）の、組A を点けたときの値から、全部消したときの値を引いたもの（LED の光の分だけ） */
int32_t wall_get_a_minus_dark(int num)
{
  int32_t value = s_lit_a[num];
  value = value - s_dark[num];
  return value;
}

/* 受光 num（1〜4）の、組B を点けたときの値から、全部消したときの値を引いたもの */
int32_t wall_get_b_minus_dark(int num)
{
  int32_t value = s_lit_b[num];
  value = value - s_dark[num];
  return value;
}

/* 電池の電圧 [V] */
float wall_get_battery_v(void)
{
  return s_battery_raw * ADC_VREF_V / ADC_FULL_SCALE * BATTERY_DIVIDER;
}

/* 前の測定が終わらないうちに次を始めようとした回数 */
uint32_t wall_get_overrun_count(void)
{
  return s_overrun_count;
}
