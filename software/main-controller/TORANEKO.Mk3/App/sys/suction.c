/*
 * suction.c
 * 吸引ファンを、少しずつ上げて回し始める（調整の動き・ターンの調整で使う）
 *
 * いきなり SUCTION_VOLTAGE_V をかけると、ファンに大きな電流が流れて電池の電圧が落ちるので、
 * SUCTION_RAMP_MS（0.5 秒）かけて、SUCTION_RAMP_STEPS 段に分けて上げる。
 * 止めるのは drv/fan.c の fan_stop()（フェイルセーフで止まったときも、sys/failsafe.c が止める）。
 * 2026-11-03 に sys/adjust.c から移した（ターンの調整でも使うため）。
 */
#include "sys/suction.h"

#include "config/run_params.h"
#include "drv/fan.h"
#include "sys/ui.h"

#define SUCTION_RAMP_STEPS 10  /* 吸引を上げるときの段の数 */

/* 吸引ファンを、SUCTION_RAMP_MS かけて voltage_v [V] まで少しずつ上げる（最短走行の No. ごとの電圧など。2026-11-10） */
void suction_ramp_up_to(float voltage_v)
{
  for (int i = 1; i <= SUCTION_RAMP_STEPS; i++) {
    fan_set_voltage(voltage_v * i / SUCTION_RAMP_STEPS);
    ui_wait_ms(SUCTION_RAMP_MS / SUCTION_RAMP_STEPS);
  }
}

/* 吸引ファンを、SUCTION_RAMP_MS かけて SUCTION_VOLTAGE_V（ふつうの電圧）まで少しずつ上げる */
void suction_ramp_up(void)
{
  suction_ramp_up_to(SUCTION_VOLTAGE_V);
}
