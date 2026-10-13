/*
 * SensorNoise.h
 *
 * 吸引のときに壁センサにのるノイズを調べるテスト(モード3 No.10、2026-10-12)。
 * 止まったまま吸引の強さと、センサの読み方(LEDを点けてから読むまでの待ち、ADCの取り込み時間)を変え、
 * 点灯時・消灯時の生の値と、その差のばらつきを測る。
 */

#ifndef INC_SENSORNOISE_H_
#define INC_SENSORNOISE_H_

#include <stdint.h>

/* 1msごとに1回、センサを4つとも読み終えたところで呼ぶ(PL_sensor.c)。
 * on/off は点灯時・消灯時の生のAD値(並びは g_sensor と同じ: 0 FL、1 R、2 L、3 FR)、
 * batt は電池のAD値、ph は読み終えたときの吸引のPWMの位置(TIM2のカウンタ。ph[0] FL・R の点灯、ph[1] L・FR の点灯、ph[2] 消灯) */
void SensorNoise_Sample(const uint16_t on[4], const uint16_t off[4], uint16_t batt, const uint8_t ph[3]);

void SensorNoise_Run(void);	//テスト全体(止まったまま。30秒ほど)
int SensorNoise_Has_Data(void);
void SensorNoise_Print(void);	//結果(ばらつきの表だけ)をもう一度出す。出力(モード4 No.7)から呼ぶ

#endif /* INC_SENSORNOISE_H_ */
