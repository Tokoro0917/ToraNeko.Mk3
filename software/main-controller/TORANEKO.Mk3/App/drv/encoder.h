/*
 * encoder.h
 * モーターの回転の読み取り（AS5047P ×2、SPI2）
 *
 * AS5047P は、モーターの軸に付けた磁石の向き（角度）を 0〜16383 の数で返す（1 回転で 16384）。
 * エンコーダはタイヤではなくモーターの軸に付いているので、タイヤ 1 回転はモーター約 3.7 回転
 * （ギア 41T ÷ 11T）= 約 61000 カウントになる。
 *
 * 制御周期（0.5ms）ごとに encoder_update() で角度を読み、前の回からの差を足していく。
 * 0.5ms のあいだにモーターが半回転（8192 カウント）以上回ることはない（約 16000rpm 未満）ので、
 * 差が半回転より大きく見えたら、0 と 16383 の境目をまたいだと考えて直す。
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

#define ENCODER_COUNT_PER_REV 16384  /* モーター 1 回転のカウント数 */

void encoder_init(void);
void encoder_update(void);

/* 電源を入れてから、左・右のモーターが回った量 [カウント]。
 * 機体が前に進む向きに回すと増え、後ろに進む向きに回すと減る。
 * モード選択（sys/ui.c）でも使う。PC のテストでは、test/test_mode.c の中の仮の関数がこの関数を持っている */
int32_t encoder_get_left_total(void);
int32_t encoder_get_right_total(void);

/* 今の角度 [カウント]（0〜16383）。歪みを直す前の、読んだままの角度。動作確認と、歪みを測るとき（sys/encoder_cal.c）に使う */
uint16_t encoder_get_left_angle(void);
uint16_t encoder_get_right_angle(void);

/* エンコーダがエラーを返した回数（電源を入れてから）と、最後のエラーの中身。動作確認用 */
uint32_t encoder_get_left_error_count(void);
uint32_t encoder_get_right_error_count(void);
uint16_t encoder_get_last_error_flags(void);

#endif /* DRV_ENCODER_H */
