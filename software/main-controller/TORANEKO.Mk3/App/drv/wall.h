/*
 * wall.h
 * 壁センサ（受光 LTR-209 ×4、発光 2 組）の AD 値の読み取りと、電池の電圧
 *
 * 制御周期ごとに wall_start_measure() で測り始めると、裏で次の 3 回の測定が順に進む（約 0.15ms）。
 *   1. 組A の LED（左前向き＋右向き）を点けて、4 つの受光の値を測る
 *   2. 組B の LED（右前向き＋左向き）を点けて、4 つの受光の値を測る
 *   3. LED を全部消して、4 つの受光の値を測る（まわりの光の分。点けたときの値から引くと、LED の光の分だけが残る）
 * 結果は、次に測り終えるまで get の関数で読める。
 */
#ifndef DRV_WALL_H
#define DRV_WALL_H

#include <stdint.h>

#define WALL_SENSOR_COUNT 4  /* 受光の数（回路図の Wallsen-1〜4） */

void wall_init(void);
void wall_start_measure(void);

/* 受光 num（1〜4、回路図の Wallsen-1〜4）の AD 値（0〜4095）。
 * 組A を点けたとき・組B を点けたとき・全部消したとき */
uint16_t wall_get_lit_a(int num);
uint16_t wall_get_lit_b(int num);
uint16_t wall_get_dark(int num);

/* 受光 num（1〜4）の、点けたときの値 − 全部消したときの値（ON−OFF。LED の光の分だけ。まわりの光は引かれる） */
int32_t wall_get_a_minus_dark(int num);
int32_t wall_get_b_minus_dark(int num);

/* 電池の電圧 [V]（壁センサと同じ AD 変換で一緒に測っている） */
float wall_get_battery_v(void);

/* 前の測定が終わらないうちに次を始めようとした回数（0 のはず。動作確認用） */
uint32_t wall_get_overrun_count(void);

#endif /* DRV_WALL_H */
