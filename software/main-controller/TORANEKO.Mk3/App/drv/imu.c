/*
 * imu.c
 * ジャイロと加速度の読み取り（LSM6DSR、SPI1）
 *
 * ■ LSM6DSR との通信（SPI）
 * CS を Low にして、最初の 1 バイトで「どのレジスタ（中の記憶の場所）を」「読むか書くか」を送る。
 *   読む：0x80 + レジスタの番号。続けて送ったバイトの数だけ、そのレジスタから順に中身が返ってくる
 *   書く：レジスタの番号。続けて送った 1 バイトが、そのレジスタに書かれる
 *
 * ■ 値が飛ばないようにする（KOGUMA で 2026-10-20 に見つかった問題）
 * 角速度は上位と下位の 2 バイトに分かれている。KOGUMA は 1 バイトずつ別々に読んでいたので、そのあいだに
 * 次の値に替わると、別々の値の上位と下位が組み合わさり、±35.8 dps 飛ぶことがあった。ToraNeko では、
 *   - BDU（Block Data Update）を有効にする：上位と下位の両方が読まれるまで、LSM6DSR が値を書き換えない
 *   - 必要なバイトを 1 回の SPI のやり取りで続けて読む
 * の 2 つで、最初から起きないようにする。
 *
 * ■ 1 回で読むもの
 * レジスタ 0x26 から 6 バイト続けて読む：角速度 Z（0x26 下位, 0x27 上位）、加速度 X（0x28, 0x29）、
 * 加速度 Y（0x2A, 0x2B）。ちょうど並んでいるので 1 回で済む。
 *
 * ■ ゼロ点
 * 止まっていても、角速度は少しずれた値（ゼロ点）を出す。起動してから最初の 0.5 秒（制御周期 1000 回）の
 * 平均をゼロ点とし、そのあとは読んだ値からゼロ点を引く。0.5 秒のあいだは機体を動かさない。
 *
 * ■ 割り込み
 * imu_update() は制御周期の割り込み（sys/scheduler.c）から呼ぶ。読んだ値は volatile の変数に入れる。
 */
#include "drv/imu.h"

#include "config/robot_params.h"
#include "drv/timebase.h"
#include "main.h"
#include "spi.h"

/* レジスタの番号（LSM6DSR のデータシート） */
#define REG_WHO_AM_I  0x0F  /* 名前の確かめ。0x6B が返れば LSM6DSR */
#define REG_CTRL1_XL  0x10  /* 加速度の設定 */
#define REG_CTRL2_G   0x11  /* ジャイロの設定 */
#define REG_CTRL3_C   0x12  /* 読み方の設定 */
#define REG_OUTZ_L_G  0x26  /* 角速度 Z の下位。ここから 6 バイト続けて読む */

#define WHO_AM_I_VALUE 0x6B
#define READ_FLAG      0x80  /* 読むときにレジスタの番号に足す数 */

/* 設定する値（データシートの表から選んだもの。どれも KOGUMA と同じ範囲、更新を速くした） */
#define CTRL1_XL_VALUE 0x9E  /* 加速度：1 秒に 3330 回更新、±8g、内部のフィルタあり */
#define CTRL2_G_VALUE  0x91  /* ジャイロ：1 秒に 3330 回更新、±4000 dps */
#define CTRL3_C_VALUE  0x44  /* BDU を有効（0x40）+ 続けて読むとレジスタの番号を自動で進める（0x04） */

#define GYRO_DPS_PER_LSB      0.14f       /* ±4000 dps のとき、1 あたり 0.14 deg/s（データシート） */
#define ACCEL_MPS2_PER_LSB    0.0023928f  /* ±8g のとき、1 あたり 0.244mg = 0.244 × 9.80665 / 1000 m/s² */
#define GYRO_Z_SIGN           1           /* 左回りで + になるなら 1、- になるなら -1（実機で確かめる） */
#define OFFSET_SAMPLE_COUNT   1000        /* ゼロ点を測る回数（制御周期 1000 回 = 0.5 秒） */
#define GYRO_SETTLE_MS        150         /* ジャイロを動かし始めてから、ゼロ点を測り始めるまで待つ時間 [ms] */
#define SPI_TIMEOUT_MS        1           /* SPI のやり取りを待つ上限 [ms] */

/* 割り込みの中で書き換える値（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile float s_gyro_z_dps = 0.0f;
static volatile float s_angle_deg = 0.0f;
static volatile float s_accel_x_mps2 = 0.0f;
static volatile float s_accel_y_mps2 = 0.0f;
static volatile float s_gyro_offset_dps = 0.0f;
static volatile float s_offset_sum_dps = 0.0f;  /* ゼロ点を測っているあいだの合計 */
static volatile int s_offset_count = 0;         /* ゼロ点を測った回数 */
static volatile bool s_offset_ready = false;
static uint8_t s_who_am_i = 0;                   /* 起動時に読んだ WHO_AM_I の値（動作確認用） */

/* CS を Low（話す）・High（話さない）にする */
static void set_cs(bool talk)
{
  if (talk) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  }
}

/* レジスタ reg に value を書く */
static void write_register(uint8_t reg, uint8_t value)
{
  uint8_t send[2] = {reg, value};
  set_cs(true);
  HAL_SPI_Transmit(&hspi1, send, 2, SPI_TIMEOUT_MS);
  set_cs(false);
}

/* レジスタ reg の中身を 1 バイト読む */
static uint8_t read_register(uint8_t reg)
{
  uint8_t send[2] = {READ_FLAG + reg, 0};
  uint8_t receive[2] = {0, 0};
  set_cs(true);
  HAL_SPI_TransmitReceive(&hspi1, send, receive, 2, SPI_TIMEOUT_MS);
  set_cs(false);
  return receive[1];  /* receive[0] は番号を送っているあいだのもので、意味がない */
}

/* 下位・上位の 2 バイトを、-32768〜32767 の数にする。
 * LSM6DSR はマイナスの数を「2 の補数」という形で送ってくる。上位 × 256 + 下位 で 0〜65535 にしたとき、
 * 32768 以上ならマイナスの数なので、65536 を引くと本当の値になる（例：65535 → -1） */
static int32_t to_signed(uint8_t low, uint8_t high)
{
  int32_t value = high * 256 + low;
  if (value >= 32768) {
    value = value - 65536;
  }
  return value;
}

/* 起動時に 1 回だけ呼ぶ。LSM6DSR かを確かめて、設定を書く。LSM6DSR でなければ false */
bool imu_init(void)
{
  timebase_wait_ms(20);  /* LSM6DSR は電源が入ってから動けるまで約 10ms かかるので、念のため待つ */
  /* SPI1 を先に動かしておく。HAL は最初のやり取りのときに SPI を動かし始めるが、そのときクロックの線が
   * Low から High（モード3 の待ちの高さ）に変わる。CS を Low にしたあとでこれが起きると、LSM6DSR は
   * 余計なクロックが 1 回来たと思い、最初のやり取りがずれる（2026-10-22 に WHO_AM_I が 0xFF になった原因）。
   * __HAL_SPI_ENABLE は SPI を動かし始めるだけの HAL の命令で、こうしておくとクロックの線は High で待つ */
  __HAL_SPI_ENABLE(&hspi1);
  s_who_am_i = read_register(REG_WHO_AM_I);
  if (s_who_am_i != WHO_AM_I_VALUE) {
    return false;
  }
  write_register(REG_CTRL3_C, CTRL3_C_VALUE);
  write_register(REG_CTRL1_XL, CTRL1_XL_VALUE);
  write_register(REG_CTRL2_G, CTRL2_G_VALUE);
  /* ジャイロは、動かし始めてから値が落ち着くまで約 70ms かかる（データシートの turn-on time）。
   * 落ち着く前の値でゼロ点を測るとずれて、角度が止まっていても増え続けるので、待ってから測り始める */
  timebase_wait_ms(GYRO_SETTLE_MS);
  return true;
}

/* ゼロ点を測っているあいだに呼ぶ。読んだ角速度を足していき、決めた回数になったら平均をゼロ点にする */
static void measure_offset(float raw_dps)
{
  s_offset_sum_dps = s_offset_sum_dps + raw_dps;
  s_offset_count = s_offset_count + 1;
  if (s_offset_count >= OFFSET_SAMPLE_COUNT) {
    s_gyro_offset_dps = s_offset_sum_dps / OFFSET_SAMPLE_COUNT;
    s_angle_deg = 0.0f;
    s_offset_ready = true;
  }
}

/* ゼロ点を測り直す（測り終わるまで約 0.5 秒。終わったかは imu_is_offset_ready() で調べる）。
 * 走り出す前の、機体が止まっているときに呼ぶ。電源を入れた直後のゼロ点は、機体を手で持って
 * 動かしているあいだに測ってしまうことがあるため（2026-10-24 に、止まっているのに 110deg/s と読んで気づいた）
 * 割り込みとの順番：合計と回数を先に 0 にしてから、最後に「測っていない」にする。
 * 「測っていない」になるまでは、割り込みは合計と回数を使わないので、途中の値を使うことはない */
void imu_start_offset_measure(void)
{
  s_offset_sum_dps = 0.0f;
  s_offset_count = 0;
  s_gyro_z_dps = 0.0f;
  s_offset_ready = false;
}

/* ゼロ点を測り直して、終わるまで待つ（約 0.5 秒）。機体が止まっているときに、メインの処理から呼ぶ */
void imu_remeasure_offset(void)
{
  imu_start_offset_measure();
  while (!imu_is_offset_ready()) {
    timebase_wait_ms(10);
  }
}

/* 制御周期ごとに呼ぶ。角速度と加速度を読み、角度を進める */
void imu_update(void)
{
  uint8_t send[7] = {READ_FLAG + REG_OUTZ_L_G, 0, 0, 0, 0, 0, 0};
  uint8_t receive[7] = {0, 0, 0, 0, 0, 0, 0};
  set_cs(true);
  HAL_SPI_TransmitReceive(&hspi1, send, receive, 7, SPI_TIMEOUT_MS);
  set_cs(false);

  /* receive[0] は番号を送っているあいだのもの。[1][2] 角速度 Z、[3][4] 加速度 X、[5][6] 加速度 Y */
  float raw_dps = GYRO_Z_SIGN * to_signed(receive[1], receive[2]) * GYRO_DPS_PER_LSB;
  s_accel_x_mps2 = to_signed(receive[3], receive[4]) * ACCEL_MPS2_PER_LSB;
  s_accel_y_mps2 = to_signed(receive[5], receive[6]) * ACCEL_MPS2_PER_LSB;

  if (!s_offset_ready) {
    measure_offset(raw_dps);
    return;
  }

  s_gyro_z_dps = raw_dps - s_gyro_offset_dps;
  /* 角度 = 角速度 × 時間 を足していく（1 回は 1 / CTRL_FREQ_HZ 秒 = 0.5ms） */
  s_angle_deg = s_angle_deg + s_gyro_z_dps / CTRL_FREQ_HZ;
}

/* 起動時に読んだ WHO_AM_I の値（LSM6DSR なら 0x6B） */
uint8_t imu_get_who_am_i(void)
{
  return s_who_am_i;
}

/* ゼロ点を測り終えたら true */
bool imu_is_offset_ready(void)
{
  return s_offset_ready;
}

/* 測ったゼロ点 [deg/s] */
float imu_get_gyro_offset_dps(void)
{
  return s_gyro_offset_dps;
}

/* 角速度 [deg/s]（ゼロ点を引いたもの。左回りが +） */
float imu_get_gyro_z_dps(void)
{
  return s_gyro_z_dps;
}

/* ゼロ点を測り終えてからの角度 [deg]（左回りが +） */
float imu_get_angle_deg(void)
{
  return s_angle_deg;
}

/* 加速度 X [m/s²] */
float imu_get_accel_x_mps2(void)
{
  return s_accel_x_mps2;
}

/* 加速度 Y [m/s²] */
float imu_get_accel_y_mps2(void)
{
  return s_accel_y_mps2;
}

