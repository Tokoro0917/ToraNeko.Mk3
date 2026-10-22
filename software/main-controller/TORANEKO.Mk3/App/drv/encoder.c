/*
 * encoder.c
 * モーターの回転の読み取り（AS5047P ×2、SPI2）
 *
 * ■ AS5047P との通信（SPI）
 * 1 回のやり取りは 16 ビット（2 バイト）。CS を Low にしているあいだに 2 バイトを送り、同時に 2 バイトを受け取る。
 * AS5047P は「1 つ前のやり取りで頼まれたもの」を返すので、角度を読むときは 2 回やり取りする。
 *   1 回目：角度を頼む（受け取ったものは前の頼みの答えなので捨てる）
 *   2 回目：もう一度角度を頼み、1 回目の答え（今の角度）を受け取る
 * 角度を頼む命令は 0xFFFF（「読む」の印 + 角度の場所 0x3FFF + 確かめのビット。計算すると全部 1 になる）。
 *
 * 受け取った 16 ビットの中身：
 *   いちばん上のビット：確かめのビット（使わない）
 *   上から 2 番目のビット：エラーの印（1 ならエラー）
 *   下の 14 ビット：角度（0〜16383）
 *
 * ■ 左右と向き
 * 左右のモーターは向かい合わせに付いているので、機体が前に進むとき、片方の角度は増え、もう片方は減る。
 * 前に進むと total が増えるように、ENCODER_*_SIGN で向きを合わせる（実機で確かめた値）。
 *
 * ■ 割り込み
 * encoder_update() は制御周期の割り込み（sys/scheduler.c）から呼ぶ。読んだ値は volatile の変数に入れ、
 * メインの処理は get の関数で読む。
 */
#include "drv/encoder.h"

#include <stdbool.h>
#include <stdio.h>

#include "drv/timebase.h"
#include "main.h"
#include "spi.h"

/* 前に進むと角度が増えるなら 1、減るなら -1。2026-10-22 に実機で確かめた
 * （前に転がすと、左の角度は減り、右の角度は増える。total は両方とも増える） */
#define ENCODER_LEFT_SIGN  -1  /* 左 */
#define ENCODER_RIGHT_SIGN 1   /* 右 */

#define READ_ANGLE_COMMAND_HIGH 0xFF  /* 角度を頼む命令 0xFFFF の上の 1 バイト */
#define READ_ANGLE_COMMAND_LOW  0xFF  /* 下の 1 バイト */
#define READ_ERRFL_COMMAND_HIGH 0x40  /* エラーの中身（ERRFL、場所 0x0001）を頼む命令 0x4001 の上の 1 バイト */
#define READ_ERRFL_COMMAND_LOW  0x01  /* 下の 1 バイト。ERRFL は読むと 0 に戻る */
#define READ_DIAG_COMMAND_HIGH 0xFF  /* 磁石の状態（DIAAGC、場所 0x3FFC）を頼む命令 0xFFFC の上の 1 バイト */
#define READ_DIAG_COMMAND_LOW  0xFC  /* 下の 1 バイト */
#define DIAG_MAGL      0x0800        /* DIAAGC の、磁石が弱すぎる（遠い・ない）の印 */
#define DIAG_MAGH      0x0400        /* 磁石が強すぎる（近すぎる）の印 */
#define DIAG_COF       0x0200        /* 角度の計算があふれた印 */
#define DIAG_LF        0x0100        /* 1 になっていれば、測定の準備ができている */
#define DIAG_AGC_MASK  0x00FF        /* 下の 8 ビット：磁石の強さに合わせた増幅の量（磁石が弱いほど大きい） */
#define ERROR_FLAG     0x4000         /* 受け取った 16 ビットの、エラーの印のビット */
#define ANGLE_MASK     0x3FFF         /* 受け取った 16 ビットの、角度の部分（下の 14 ビット） */
#define HALF_REV_COUNT 8192           /* 半回転のカウント数 */
#define SPI_TIMEOUT_MS 1              /* SPI のやり取りを待つ上限 [ms] */

typedef enum {
  ENCODER_LEFT = 0,
  ENCODER_RIGHT = 1,
} encoder_side_t;

/* 割り込みの中で書き換える値（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile uint16_t s_left_angle = 0;
static volatile uint16_t s_right_angle = 0;
static volatile int32_t s_left_total = 0;
static volatile int32_t s_right_total = 0;
static volatile uint32_t s_left_error_count = 0;
static volatile uint32_t s_right_error_count = 0;
static volatile uint16_t s_last_error_flags = 0;  /* 最後に読んだエラーの中身（ERRFL）。1=フレーム、2=命令、4=確かめのビット */

/* side のエンコーダの CS（このエンコーダと話す、の合図）を Low（話す）・High（話さない）にする */
static void set_cs(encoder_side_t side, bool talk)
{
  GPIO_PinState state = GPIO_PIN_SET;
  if (talk) {
    state = GPIO_PIN_RESET;
  }
  if (side == ENCODER_LEFT) {
    HAL_GPIO_WritePin(ENC_L_CS_GPIO_Port, ENC_L_CS_Pin, state);
  } else {
    HAL_GPIO_WritePin(ENC_R_CS_GPIO_Port, ENC_R_CS_Pin, state);
  }
}

/* 少しだけ（1〜2µs）待つ。AS5047P は、やり取りとやり取りのあいだに CS を High にして 350ns 以上待つ決まり */
static void wait_between_frames(void)
{
  uint32_t start_us = timebase_get_us();
  while (timebase_get_us() - start_us < 2) {
  }
}

/* side のエンコーダと 1 回やり取りする（命令 command_high・command_low を送り、前の命令の答えを受け取る）。
 * 受け取った 16 ビットを返す */
static uint16_t exchange_frame(encoder_side_t side, uint8_t command_high, uint8_t command_low)
{
  uint8_t send[2] = {command_high, command_low};
  uint8_t receive[2] = {0, 0};

  set_cs(side, true);
  HAL_SPI_TransmitReceive(&hspi2, send, receive, 2, SPI_TIMEOUT_MS);
  set_cs(side, false);
  wait_between_frames();

  /* 上の 1 バイトを 256 倍して、下の 1 バイトを足すと、16 ビットの数になる */
  return receive[0] * 256 + receive[1];
}

/* side のエンコーダのエラーの数を 1 つ増やす */
static void count_error(encoder_side_t side)
{
  if (side == ENCODER_LEFT) {
    s_left_error_count = s_left_error_count + 1;
  } else {
    s_right_error_count = s_right_error_count + 1;
  }
}

/* エラーの中身（ERRFL）を読んで覚えておく。ERRFL は読むと 0 に戻るので、エラーの印も消える
 * （読まないと、エラーの印が立ったままになる） */
static void read_and_clear_error(encoder_side_t side)
{
  exchange_frame(side, READ_ERRFL_COMMAND_HIGH, READ_ERRFL_COMMAND_LOW);  /* ERRFL を頼む */
  uint16_t frame = exchange_frame(side, READ_ANGLE_COMMAND_HIGH, READ_ANGLE_COMMAND_LOW);  /* 答えを受け取る */
  s_last_error_flags = frame & ANGLE_MASK;  /* 下の 14 ビットが ERRFL の中身 */
}

/* side のエンコーダの今の角度（0〜16383）を読む。エラーなら false を返し、angle は変えない */
static bool read_angle(encoder_side_t side, uint16_t *angle)
{
  exchange_frame(side, READ_ANGLE_COMMAND_HIGH, READ_ANGLE_COMMAND_LOW);                   /* 1 回目：角度を頼む（答えは捨てる） */
  uint16_t frame = exchange_frame(side, READ_ANGLE_COMMAND_HIGH, READ_ANGLE_COMMAND_LOW);  /* 2 回目：1 回目の答えを受け取る */

  /* & は、決めたビットだけを取り出す計算。ERROR_FLAG のビットが 1 ならエラー */
  if ((frame & ERROR_FLAG) != 0) {
    count_error(side);
    read_and_clear_error(side);
    return false;
  }
  *angle = frame & ANGLE_MASK;  /* 下の 14 ビットだけを取り出す（上の 2 ビットは印なので捨てる） */
  return true;
}

/* 前の角度から今の角度への変化 [カウント]。0 と 16383 の境目をまたいだときは、近い向きに直す */
static int32_t angle_change(uint16_t last_angle, uint16_t now_angle)
{
  int32_t change = now_angle;
  change = change - last_angle;  /* int32_t で引き算するので、マイナスにもなれる */
  if (change > HALF_REV_COUNT) {
    change = change - ENCODER_COUNT_PER_REV;  /* 例：10 → 16380 は、+16370 ではなく -14 */
  }
  if (change < -HALF_REV_COUNT) {
    change = change + ENCODER_COUNT_PER_REV;  /* 例：16380 → 10 は、-16370 ではなく +14 */
  }
  return change;
}

/* side のエンコーダの磁石の状態（DIAAGC）を読んで、シリアルに出す。起動時の確認用
 * （制御周期の割り込みが動き出す前に呼ぶ。動き出したあとだと、割り込みのやり取りと混ざるため） */
static void print_diagnostics(encoder_side_t side, const char *name)
{
  exchange_frame(side, READ_DIAG_COMMAND_HIGH, READ_DIAG_COMMAND_LOW);  /* DIAAGC を頼む */
  uint16_t frame = exchange_frame(side, READ_ANGLE_COMMAND_HIGH, READ_ANGLE_COMMAND_LOW);  /* 答えを受け取る */

  /* & で、それぞれの印のビットだけを取り出す。0 でなければ印が立っている */
  printf("Encoder %s: AGC=%u MAGL=%d MAGH=%d COF=%d LF=%d (raw 0x%04X)\r\n", name,
         frame & DIAG_AGC_MASK,
         (frame & DIAG_MAGL) != 0, (frame & DIAG_MAGH) != 0,
         (frame & DIAG_COF) != 0, (frame & DIAG_LF) != 0, frame);
}

/* 起動時に 1 回だけ呼ぶ。今の角度を読んで、ここを 0 として数え始める */
void encoder_init(void)
{
  /* SPI2 を先に動かしておく。HAL は最初のやり取りのときに SPI を動かし始めるので、それまでクロックの線の
   * 高さが決まっておらず、最初のやり取りがずれることがある（IMU で 2026-10-22 に起きたのと同じ。drv/imu.c）。
   * __HAL_SPI_ENABLE は SPI を動かし始めるだけの HAL の命令 */
  __HAL_SPI_ENABLE(&hspi2);

  print_diagnostics(ENCODER_LEFT, "L");
  print_diagnostics(ENCODER_RIGHT, "R");

  uint16_t angle = 0;
  if (read_angle(ENCODER_LEFT, &angle)) {
    s_left_angle = angle;
  }
  if (read_angle(ENCODER_RIGHT, &angle)) {
    s_right_angle = angle;
  }
  s_left_total = 0;
  s_right_total = 0;
}

/* 制御周期ごとに呼ぶ。左右の角度を読み、前の回からの変化を回った量に足す。エラーのときは、その回は足さない */
void encoder_update(void)
{
  uint16_t angle = 0;

  if (read_angle(ENCODER_LEFT, &angle)) {
    s_left_total = s_left_total + ENCODER_LEFT_SIGN * angle_change(s_left_angle, angle);
    s_left_angle = angle;
  }
  if (read_angle(ENCODER_RIGHT, &angle)) {
    s_right_total = s_right_total + ENCODER_RIGHT_SIGN * angle_change(s_right_angle, angle);
    s_right_angle = angle;
  }
}

/* 電源を入れてから、左のモーターが回った量 [カウント]（前に進む向きが +） */
int32_t encoder_get_left_total(void)
{
  return s_left_total;
}

/* 電源を入れてから、右のモーターが回った量 [カウント]（前に進む向きが +） */
int32_t encoder_get_right_total(void)
{
  return s_right_total;
}

/* 左の今の角度 [カウント]（0〜16383） */
uint16_t encoder_get_left_angle(void)
{
  return s_left_angle;
}

/* 右の今の角度 [カウント]（0〜16383） */
uint16_t encoder_get_right_angle(void)
{
  return s_right_angle;
}

/* 左のエンコーダがエラーを返した回数（電源を入れてから） */
uint32_t encoder_get_left_error_count(void)
{
  return s_left_error_count;
}

/* 右のエンコーダがエラーを返した回数（電源を入れてから） */
uint32_t encoder_get_right_error_count(void)
{
  return s_right_error_count;
}

/* 最後に読んだエラーの中身（ERRFL）。1=フレームのエラー、2=知らない命令、4=確かめのビットが合わない（足し合わせで出る） */
uint16_t encoder_get_last_error_flags(void)
{
  return s_last_error_flags;
}
