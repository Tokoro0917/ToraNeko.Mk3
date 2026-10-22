/*
 * app.c
 * App 全体の入口。main.c から app_init() と app_loop() が呼ばれる
 *
 * 今の中身はステップ1〜6の動作確認：
 *   - LED1 → LED2 → … → LED7 → LED1 … の順に、0.1 秒ごとに 1 個ずつ点灯する
 *   - 1 秒ごとに、経過時間と、制御周期（2kHz）の割り込みの記録をシリアルに出す
 *     tick=1 秒間に割り込みが来た回数（2000 のはず）、busy=1 回の処理にかかった最大の時間、
 *     period=割り込みの間隔の最小〜最大（500µs のはず）
 *   - 0.2 秒ごとに、壁センサの AD 値をシリアルに出す
 *     A-off=組A（左前向き＋右向き）を点けたときの受光 1〜4 の値 − 全部消したときの値（ON−OFF）、
 *     B-off=組B（右前向き＋左向き）で同じもの、off=全部消したときの値
 *   - 1 秒ごとに、エンコーダとジャイロの値と、電池の電圧も出す
 *     L・R=左右のモーターが回った量、err=エンコーダのエラーの回数、angle=角度 [deg]
 *   - スイッチ SW1 を押して離すと、モーターの試しをする（前に 1 秒 → 止める → 後ろに 1 秒 → 止める）。
 *     タイヤを浮かせてから押す
 *   - ジャイロのゼロ点を測り終えたら（起動して 0.5 秒）、ゼロ点を 1 回だけ出す
 */
#include "app.h"

#include <stdbool.h>
#include <stdio.h>

#include "drv/encoder.h"
#include "drv/imu.h"
#include "drv/led.h"
#include "drv/motor.h"
#include "drv/switch.h"
#include "drv/timebase.h"
#include "drv/uart.h"
#include "drv/wall.h"
#include "sys/scheduler.h"

#define LED_INTERVAL_MS   100   /* LED を次に進める間隔 [ms] */
#define PRINT_INTERVAL_MS 1000  /* 制御周期の記録をシリアルに出す間隔 [ms] */
#define SENSOR_PRINT_INTERVAL_MS 200  /* 壁センサの値をシリアルに出す間隔 [ms] */

static uint32_t s_last_led_ms = 0;    /* 最後に LED を進めた時刻 [ms] */
static uint32_t s_last_print_ms = 0;  /* 最後に制御周期の記録を出した時刻 [ms] */
static uint32_t s_last_sensor_print_ms = 0;  /* 最後に壁センサの値を出した時刻 [ms] */
static bool s_offset_printed = false;        /* ジャイロのゼロ点を出したか */
static uint8_t s_led_num = 1;         /* 今点けている LED の番号（1〜7） */

#define MOTOR_TEST_V       1.0f  /* モーターの試しでかける電圧 [V] */
#define MOTOR_TEST_MS      1000  /* モーターの試しで 1 つの向きに回す時間 [ms] */
#define MOTOR_PAUSE_MS     500   /* 向きを変えるあいだに止めておく時間 [ms] */
#define HAND_AWAY_MS       500   /* スイッチを離してから回し始めるまで待つ時間 [ms]（手を離すため） */

/* モーターを電圧 left_v・right_v で time_ms 回して止め、そのあいだに左右のエンコーダが回った量を出す */
static void run_motor_phase(const char *name, float left_v, float right_v, uint32_t time_ms)
{
  int32_t left_start = encoder_get_left_total();
  int32_t right_start = encoder_get_right_total();
  motor_set_voltage(left_v, right_v);
  timebase_wait_ms(time_ms);
  motor_stop();
  printf("  %s: L moved %ld, R moved %ld (batt %.2f V)\r\n", name,
         encoder_get_left_total() - left_start, encoder_get_right_total() - right_start,
         wall_get_battery_v());
}

/* モーターの試し：前に回す → 止める → 後ろに回す → 止める */
static void run_motor_test(void)
{
  printf("Motor test (%.1f V, %d ms each)\r\n", MOTOR_TEST_V, MOTOR_TEST_MS);
  run_motor_phase("forward ", MOTOR_TEST_V, MOTOR_TEST_V, MOTOR_TEST_MS);
  timebase_wait_ms(MOTOR_PAUSE_MS);
  run_motor_phase("backward", -MOTOR_TEST_V, -MOTOR_TEST_V, MOTOR_TEST_MS);
  printf("Motor test done\r\n");
}

/* スイッチが押されたら、離されるまで待ってから、モーターの試しをする */
static void check_switch_and_run_motor_test(void)
{
  if (!switch_is_pressed()) {
    return;
  }
  while (switch_is_pressed()) {
    timebase_wait_ms(10);
  }
  timebase_wait_ms(HAND_AWAY_MS);
  run_motor_test();
}

/* シリアルに出す文字は英数字だけにする（日本語は、表示するソフトによって文字化けするため） */

/* 起動時に 1 回だけ呼ばれる */
void app_init(void)
{
  led_init();
  uart_init();
  timebase_init();
  printf("\r\nToraNeko.Mk3 boot (SYSCLK %lu Hz)\r\n", timebase_get_sysclk_hz());
  wall_init();
  encoder_init();
  motor_init();
  if (imu_init()) {
    printf("IMU OK (LSM6DSR)\r\n");
  } else {
    printf("IMU NG: WHO_AM_I=0x%02X (expected 0x6B)\r\n", imu_get_who_am_i());
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

  check_switch_and_run_motor_test();

  /* ジャイロのゼロ点を測り終えたら、1 回だけ出す */
  if (imu_is_offset_ready() && !s_offset_printed) {
    s_offset_printed = true;
    printf("Gyro offset %.3f deg/s\r\n", imu_get_gyro_offset_dps());
  }

  /* 壁センサの AD 値をシリアルに出す */
  if (now_ms - s_last_sensor_print_ms >= SENSOR_PRINT_INTERVAL_MS) {
    s_last_sensor_print_ms = now_ms;
    printf("A-off=%5ld %5ld %5ld %5ld  B-off=%5ld %5ld %5ld %5ld  off=%4u %4u %4u %4u\r\n",
           wall_get_a_minus_dark(1), wall_get_a_minus_dark(2), wall_get_a_minus_dark(3), wall_get_a_minus_dark(4),
           wall_get_b_minus_dark(1), wall_get_b_minus_dark(2), wall_get_b_minus_dark(3), wall_get_b_minus_dark(4),
           wall_get_dark(1), wall_get_dark(2), wall_get_dark(3), wall_get_dark(4));
  }

  /* 経過時間と、制御周期の割り込みの記録をシリアルに出す */
  if (now_ms - s_last_print_ms >= PRINT_INTERVAL_MS) {
    s_last_print_ms = now_ms;
    scheduler_stats_t stats;
    scheduler_take_stats(&stats);
    printf("t=%lu ms tick=%lu busy=%lu us period=%lu-%lu us dropped=%lu\r\n",
           now_ms, stats.tick_count, stats.busy_max_us,
           stats.period_min_us, stats.period_max_us, uart_get_dropped_count());
    printf("  L=%ld (angle %u) R=%ld (angle %u) err=%lu/%lu angle=%.1f deg batt=%.2f V wall_overrun=%lu\r\n",
           encoder_get_left_total(), encoder_get_left_angle(),
           encoder_get_right_total(), encoder_get_right_angle(),
           encoder_get_left_error_count(), encoder_get_right_error_count(),
           imu_get_angle_deg(), wall_get_battery_v(), wall_get_overrun_count());
  }
}
