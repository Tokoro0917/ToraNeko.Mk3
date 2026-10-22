/*
 * encoder_correction.h
 * エンコーダの角度の歪みを直す表（drv/encoder.c が使う）
 *
 * AS5047P が返す角度は、磁石の付け方のずれなどで、本当の角度から少しずれている（1 回転の中で、
 * 同じ形のずれが毎回くり返す）。モーターの 1 回転（0〜16383）を 64 に分けて、それぞれの場所で
 * 「読んだ角度が本当の角度より何カウント大きいか」を表にしておき、読んだ角度からこの数を引いて直す。
 * 表のあいだの角度は、となりの 2 つの数をつないで（直線で）求める。
 *
 * ■ 表の作り方（MODES.md のモード6 No.6）
 *   1. タイヤを浮かせて、モード6 No.6 を選び、前センサに手をかざして始める
 *   2. シリアルに表が出るので、下の 2 つの表の中身（{ } の中）を、出たものに入れ替える
 *   3. ビルドして書き込む
 * 全部 0 なら、直さない（読んだ角度をそのまま使う）。
 */
#ifndef CONFIG_ENCODER_CORRECTION_H
#define CONFIG_ENCODER_CORRECTION_H

#include <stdint.h>

#define ENCODER_CORRECTION_BIN_COUNT 64   /* 1 回転をいくつに分けるか */
#define ENCODER_CORRECTION_BIN_WIDTH 256  /* 1 つ分の幅 [カウント]（16384 ÷ 64） */

/* 左のエンコーダ。i 番目は、角度 i × 256 + 128（i 番目の幅のまん中）でのずれ [カウント] */
static const int16_t ENCODER_LEFT_CORRECTION[ENCODER_CORRECTION_BIN_COUNT] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
};

/* 右のエンコーダ。並び方は左と同じ */
static const int16_t ENCODER_RIGHT_CORRECTION[ENCODER_CORRECTION_BIN_COUNT] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
};

#endif /* CONFIG_ENCODER_CORRECTION_H */
