/*
 * app.c
 * App 全体の入口。main.c から app_init() と app_loop() が呼ばれる
 *
 * 起動したら、センサやモーターを使えるようにしてから、モードを選んで走らせる（sys/mode.c）。
 * 操作の仕方は MODES.md。
 */
#include "app.h"

#include <stdio.h>

#include "config/robot_params.h"

#include "drv/encoder.h"
#include "drv/imu.h"
#include "drv/led.h"
#include "drv/motor.h"
#include "drv/timebase.h"
#include "drv/uart.h"
#include "drv/wall.h"
#include "sys/mode.h"
#include "sys/scheduler.h"
#include "sys/ui.h"

#define BATTERY_MEASURE_WAIT_MS 10  /* 電池の電圧を読む前に待つ時間 [ms]（制御周期 20 回ぶん） */

/* シリアルに出す文字は英数字だけにする（日本語は、表示するソフトによって文字化けするため） */

/* 電池の電圧を調べ、下限より低ければ、知らせて止まる（戻らない）。電池を調べるのは電源を入れたときだけ（KOGUMA と同じ） */
static void check_battery(void)
{
  timebase_wait_ms(BATTERY_MEASURE_WAIT_MS);  /* 制御周期の割り込みで、電池の電圧を何回か測り終えるのを待つ */
  float battery_v = wall_get_battery_v();
  printf("Battery %.2f V\r\n", battery_v);
  if (battery_v < BATTERY_LOW_V) {
    printf("Battery low (below %.1f V). Replace the battery\r\n", BATTERY_LOW_V);
    motor_stop();
    ui_stop_for_low_battery();
  }
}

/* 起動時に 1 回だけ呼ばれる。周辺機能を使えるようにして、制御周期の割り込みを始める */
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
  scheduler_start();  /* ここから、制御周期ごとにエンコーダ・IMU・壁センサを読む。ジャイロのゼロ点は最初の 0.5 秒で測る */
  check_battery();
  mode_init();
  ui_show_loading();  /* 電源を入れたときの「ロード中」（MODES.md の 2）。このあいだに、ジャイロのゼロ点も測り終わる */
}

/* main() の while(1) から繰り返し呼ばれる。モードを選んで走らせ、終わったら戻る */
void app_loop(void)
{
  mode_run_once();
}
