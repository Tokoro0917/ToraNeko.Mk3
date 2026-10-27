/*
 * sensor_view.c
 * センサ値の表示（モード5 No.1）と、壁までの距離の表示（モード5 No.3）。MODES.md
 *
 * スイッチが押されるまで、次の 2 つを続ける。
 *   - 0.2 秒ごとに、壁センサ 4 つ（ON−OFF）、電池の電圧、角速度をシリアルに出す
 *     書式は KOGUMA のモード4 No.0 と同じ（ビューアの流れるグラフで見られる）
 *   - 壁の有無を LED に出す：左の壁 LED1・2、前の壁 LED3・4・5、右の壁 LED6・7
 */
#include "sys/sensor_view.h"

#include <stdio.h>

#include "ctrl/wall_ctrl.h"
#include "drv/imu.h"
#include "drv/led.h"
#include "drv/timebase.h"
#include "drv/wall.h"
#include "sys/ui.h"

#define PRINT_INTERVAL_MS 200  /* シリアルに出す間隔 [ms] */

/* 壁の有無を LED に出す */
static void show_walls(void)
{
  bool left = wall_ctrl_has_wall_left();
  bool front = wall_ctrl_has_wall_front();
  bool right = wall_ctrl_has_wall_right();
  led_set(1, left);
  led_set(2, left);
  led_set(3, front);
  led_set(4, front);
  led_set(5, front);
  led_set(6, right);
  led_set(7, right);
}

/* センサの値を 1 行出す（KOGUMA と同じ書式） */
static void print_sensors(void)
{
  printf("FL = %4ld // L = %4ld // R = %4ld //FR = %4ld //BATT = %.2f//GyroZ = %.3f\n\r",
         wall_ctrl_get_front_left(), wall_ctrl_get_left(),
         wall_ctrl_get_right(), wall_ctrl_get_front_right(),
         wall_get_battery_v(), imu_get_gyro_z_dps());
}

/* センサの値と、壁までの距離 [mm] を 1 行出す。
 * 書式は KOGUMA のモード3 No.4 と同じ CAL,左前,左,右,右前,左前mm,左mm,右mm,右前mm（ビューアの流れるグラフで見られる） */
static void print_distances(void)
{
  printf("CAL,%ld,%ld,%ld,%ld,%.1f,%.1f,%.1f,%.1f\r\n",
         wall_ctrl_get_front_left(), wall_ctrl_get_left(),
         wall_ctrl_get_right(), wall_ctrl_get_front_right(),
         wall_ctrl_get_front_left_mm(), wall_ctrl_get_left_mm(),
         wall_ctrl_get_right_mm(), wall_ctrl_get_front_right_mm());
}

/* スイッチが押されるまで、センサの値をシリアルと LED に出し続ける */
void sensor_view_run(void)
{
  uint32_t next_print_ms = timebase_get_ms();
  while (!ui_switch_pushed()) {
    show_walls();
    if (timebase_get_ms() >= next_print_ms) {
      print_sensors();
      next_print_ms = next_print_ms + PRINT_INTERVAL_MS;
    }
    ui_wait_ms(UI_POLL_MS);
  }
  ui_all_off();
}

/* モード5 No.3：スイッチが押されるまで、センサの値と壁までの距離 [mm] をシリアルに、壁の有無を LED に出し続ける。
 * 表（ctrl/wall_dist.c）を入れたあと、ものさしで測った距離と同じになるかを確かめるのに使う */
void sensor_view_distance_run(void)
{
  uint32_t next_print_ms = timebase_get_ms();
  while (!ui_switch_pushed()) {
    show_walls();
    if (timebase_get_ms() >= next_print_ms) {
      print_distances();
      next_print_ms = next_print_ms + PRINT_INTERVAL_MS;
    }
    ui_wait_ms(UI_POLL_MS);
  }
  ui_all_off();
}
