/*
 * app.c
 * App 全体の入口。main.c から app_init() と app_loop() が呼ばれる
 *
 * 今の中身はステップ1〜4の動作確認：
 *   - LED1 → LED2 → … → LED7 → LED1 … の順に、0.1 秒ごとに 1 個ずつ点灯する
 *   - 1 秒ごとに、経過時間と、制御周期（2kHz）の割り込みの記録をシリアルに出す
 *     tick=1 秒間に割り込みが来た回数（2000 のはず）、busy=1 回の処理にかかった最大の時間、
 *     period=割り込みの間隔の最小〜最大（500µs のはず）
 *   - 0.2 秒ごとに、エンコーダとジャイロ・加速度の値をシリアルに出す
 *     L・R=電源を入れてから左右のモーターが回った量（前に進む向きが +）、err=左/右のエンコーダのエラーの回数
 *     （かっこの中は最後のエラーの中身：1=フレーム、2=知らない命令、4=確かめのビット）、
 *     gyro=角速度 [deg/s]（左回りが +）、angle=角度 [deg]、ax・ay=加速度 [m/s²]
 *   - ジャイロのゼロ点を測り終えたら（起動して 0.5 秒）、ゼロ点を 1 回だけ出す
 */
#include "app.h"

#include <stdbool.h>
#include <stdio.h>

#include "drv/encoder.h"
#include "drv/imu.h"
#include "drv/led.h"
#include "drv/timebase.h"
#include "drv/uart.h"
#include "sys/scheduler.h"

#define LED_INTERVAL_MS   100   /* LED を次に進める間隔 [ms] */
#define PRINT_INTERVAL_MS 1000  /* 制御周期の記録をシリアルに出す間隔 [ms] */
#define SENSOR_PRINT_INTERVAL_MS 200  /* エンコーダ・ジャイロの値をシリアルに出す間隔 [ms] */

static uint32_t s_last_led_ms = 0;    /* 最後に LED を進めた時刻 [ms] */
static uint32_t s_last_print_ms = 0;  /* 最後に制御周期の記録を出した時刻 [ms] */
static uint32_t s_last_sensor_print_ms = 0;  /* 最後にエンコーダ・ジャイロの値を出した時刻 [ms] */
static bool s_offset_printed = false;        /* ジャイロのゼロ点を出したか */
static uint8_t s_led_num = 1;         /* 今点けている LED の番号（1〜7） */

/* シリアルに出す文字は英数字だけにする（日本語は、表示するソフトによって文字化けするため） */

/* 起動時に 1 回だけ呼ばれる */
void app_init(void)
{
  led_init();
  uart_init();
  timebase_init();
  printf("\r\nToraNeko.Mk3 boot (SYSCLK %lu Hz)\r\n", timebase_get_sysclk_hz());
  encoder_init();
  if (imu_init()) {
    printf("IMU OK (LSM6DSR)\r\n");
  } else {
    printf("IMU NG: WHO_AM_I=0x%02X (expected 0x6B)\r\n", imu_get_who_am_i());
    imu_print_probe();  /* 原因を調べるため（一時的） */
  }
  printf("Measuring gyro offset. Keep still for 0.5 s\r\n");
  scheduler_start();
}

/* main() の while(1) から繰り返し呼ばれる */
void app_loop(void)
{
  uint32_t now_ms = timebase_get_ms();

  /* LED を 1 つ進める */
  if (now_ms - s_last_led_ms >= LED_INTERVAL_MS) {
    s_last_led_ms = now_ms;

    led_all_off();
    led_set(s_led_num, true);

    s_led_num = s_led_num + 1;
    if (s_led_num > LED_COUNT) {
      s_led_num = 1;
    }
  }

  /* ジャイロのゼロ点を測り終えたら、1 回だけ出す */
  if (imu_is_offset_ready() && !s_offset_printed) {
    s_offset_printed = true;
    printf("Gyro offset %.3f deg/s\r\n", imu_get_gyro_offset_dps());
  }

  /* エンコーダとジャイロ・加速度の値をシリアルに出す */
  if (now_ms - s_last_sensor_print_ms >= SENSOR_PRINT_INTERVAL_MS) {
    s_last_sensor_print_ms = now_ms;
    printf("L=%7ld R=%7ld err=%lu/%lu(%X)  gyro=%8.2f dps angle=%8.2f deg  ax=%6.2f ay=%6.2f m/s2\r\n",
           encoder_get_left_total(), encoder_get_right_total(),
           encoder_get_left_error_count(), encoder_get_right_error_count(), encoder_get_last_error_flags(),
           imu_get_gyro_z_dps(), imu_get_angle_deg(),
           imu_get_accel_x_mps2(), imu_get_accel_y_mps2());
  }

  /* 経過時間と、制御周期の割り込みの記録をシリアルに出す */
  if (now_ms - s_last_print_ms >= PRINT_INTERVAL_MS) {
    s_last_print_ms = now_ms;
    scheduler_stats_t stats;
    scheduler_take_stats(&stats);
    printf("t=%lu ms tick=%lu busy=%lu us period=%lu-%lu us dropped=%lu\r\n",
           now_ms, stats.tick_count, stats.busy_max_us,
           stats.period_min_us, stats.period_max_us, uart_get_dropped_count());
  }
}
