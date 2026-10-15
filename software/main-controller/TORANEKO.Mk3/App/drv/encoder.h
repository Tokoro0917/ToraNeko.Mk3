/*
 * encoder.h
 * タイヤの回転の読み取り（AS5047P ×2）
 *
 * 中身（encoder.c）はステップ3で作る。今は、モード選択（sys/mode.c、sys/ui.c）が使う「タイヤが回った量」の
 * 形だけを先に決めておく。PC のテストでは、test/test_mode.c の中の仮の関数がこの関数を持っている。
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

/* 電源を入れてから、左・右のタイヤが回った量 [カウント]（AS5047P は 1 回転で 16384 カウント）。
 * 機体が前に進む向きに回すと増え、後ろに進む向きに回すと減る */
int32_t encoder_get_left_total(void);
int32_t encoder_get_right_total(void);

#endif /* DRV_ENCODER_H */
