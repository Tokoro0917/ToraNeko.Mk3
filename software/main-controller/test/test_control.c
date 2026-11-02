/*
 * test_control.c
 * 制御（ctrl/control.c）のフェイルセーフの見つけ方の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * control.c が使うセンサ・モーター・ログなどは、ここで仮に作る（目標の軌道は本物の ctrl/trajectory.c）。
 * 仮のエンコーダは「測った速さ」をテストが決めた値にして、次のことを調べる。
 *   1. 目標どおりに進んでいれば、最後まで止まらない
 *   2. 300mm/s で走っているときに進まなくなったら（壁を押している）、10ms 続いたところで止まる。それより前には止まらない
 *   3. 1000mm/s で走っているときに急に遅くなったら（ぶつかった）、10ms 続いたところで止まる
 *   4. 少しのあいだだけ遅くなって、すぐ戻ったら（ノイズ）、止まらない
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "config/robot_params.h"
#include "ctrl/control.h"
#include "ctrl/odometry.h"
#include "ctrl/trajectory.h"
#include "ctrl/wall_ctrl.h"
#include "drv/imu.h"
#include "drv/motor.h"
#include "drv/wall.h"
#include "sys/failsafe.h"
#include "sys/log.h"

static int s_fail_count = 0;

/* ok が false なら失敗として数え、message を表示する */
static void check(bool ok, const char *message)
{
  if (!ok) {
    s_fail_count = s_fail_count + 1;
    printf("  失敗: %s\n", message);
  }
}

/* ---------------------------------------------------------------------------
 * 仮のセンサ・モーター・ログ（control.c が使う関数）
 * ------------------------------------------------------------------------- */

static float s_fake_speed_mm_s = 0.0f;     /* 仮のエンコーダが答える、測った速さ */
static float s_fake_distance_mm = 0.0f;    /* 仮のエンコーダが答える、走った距離 */
static bool s_failsafe = false;

float odometry_get_speed_mm_s(void)
{
  return s_fake_speed_mm_s;
}

float odometry_get_distance_mm(void)
{
  return s_fake_distance_mm;
}

float imu_get_angle_deg(void)
{
  return 0.0f;
}

float imu_get_gyro_z_dps(void)
{
  return 0.0f;
}

void motor_set_voltage(float left_v, float right_v)
{
  (void)left_v;  /* 使わない引数（(void) を付けると、使っていないという警告が出ない） */
  (void)right_v;
}

void motor_stop(void)
{
}
float wall_get_battery_v(void)
{
  return 8.0f;
}

int32_t wall_ctrl_get_left(void)
{
  return 0;
}

int32_t wall_ctrl_get_right(void)
{
  return 0;
}

float wall_ctrl_get_left_mm(void)
{
  return 42.0f;
}

float wall_ctrl_get_right_mm(void)
{
  return 42.0f;
}

float wall_ctrl_get_front_left_mm(void)
{
  return 42.0f;
}

float wall_ctrl_get_front_right_mm(void)
{
  return 42.0f;
}

float wall_ctrl_get_side_error(void)
{
  return 0.0f;
}

int wall_ctrl_get_side_walls_used(void)
{
  return 0;
}

float wall_ctrl_get_front_distance_error(void)
{
  return 0.0f;
}

float wall_ctrl_get_front_angle_error(void)
{
  return 0.0f;
}

void log_record(log_values_t values)
{
  (void)values;
}

bool failsafe_is_stopped(void)
{
  return s_failsafe;
}

void failsafe_stop(void)
{
  s_failsafe = true;
}

/* ---------------------------------------------------------------------------
 * テスト
 * ------------------------------------------------------------------------- */

/* 制御周期を 1 回まわす。仮のエンコーダは speed_mm_s で進んだことにする */
static void tick(float speed_mm_s)
{
  s_fake_speed_mm_s = speed_mm_s;
  s_fake_distance_mm = s_fake_distance_mm + speed_mm_s * CTRL_PERIOD_S;
  control_update();
}

/* 止まった状態から始め直す */
static void reset(void)
{
  s_failsafe = false;
  s_fake_speed_mm_s = 0.0f;
  s_fake_distance_mm = 0.0f;
  control_start();
}

/* 目標の速さ speed_mm_s に乗るまで、目標どおりに進める（1000mm 先まで走る直進） */
static void run_up_to(float speed_mm_s, float accel_mm_s2)
{
  trajectory_start_straight(1000.0f, speed_mm_s, speed_mm_s, accel_mm_s2);
  for (int i = 0; i < 2000 && trajectory_get_speed_mm_s() < speed_mm_s; i++) {
    tick(trajectory_get_speed_mm_s());
  }
}

/* 1. 目標どおりなら止まらない */
static void test_normal(void)
{
  printf("[1] 目標どおりに進めば、止まらない\n");
  reset();
  trajectory_start_straight(360.0f, 300.0f, 0.0f, 1000.0f);
  for (int i = 0; i < 4000 && !trajectory_is_done(); i++) {
    tick(trajectory_get_speed_mm_s());
  }
  check(!s_failsafe, "最後まで走っても、フェイルセーフにならない");
  check(control_get_failsafe_reason() == CONTROL_FAILSAFE_NONE, "止めたわけは「なし」");
}

/* 走っているところで、測った速さを stuck_mm_s にして、何周期目に止まるかを返す（止まらなければ -1） */
static int ticks_until_failsafe(float cruise_mm_s, float stuck_mm_s)
{
  reset();
  run_up_to(cruise_mm_s, 3000.0f);
  for (int i = 1; i <= 200; i++) {
    tick(stuck_mm_s);
    if (s_failsafe) {
      return i;
    }
  }
  return -1;
}

/* 2. 進まなくなったら（壁を押している）、10ms で止まる */
static void test_stall(void)
{
  printf("[2] 300mm/s で走っていて進まなくなったら、10ms で止まる\n");
  int ticks = ticks_until_failsafe(300.0f, 0.0f);
  printf("  %d 周期目（%.1fms）に止まった\n", ticks, ticks * CTRL_PERIOD_S * 1000.0f);
  check(ticks == 20, "ちょうど 10ms（20 周期）続いたところで止まる");
  check(control_get_failsafe_reason() == CONTROL_FAILSAFE_STALL, "止めたわけは「ほとんど進んでいない」");
}

/* 3. 1000mm/s で急に遅くなったら（ぶつかった）、10ms で止まる */
static void test_slow(void)
{
  printf("[3] 1000mm/s で走っていて急に遅くなったら、10ms で止まる\n");
  int ticks = ticks_until_failsafe(1000.0f, 100.0f);
  printf("  %d 周期目（%.1fms）に止まった\n", ticks, ticks * CTRL_PERIOD_S * 1000.0f);
  check(ticks == 20, "ちょうど 10ms（20 周期）続いたところで止まる");
  check(control_get_failsafe_reason() == CONTROL_FAILSAFE_SLOW, "止めたわけは「目標よりずっと遅い」");
}

/* 4. 少しのあいだだけ遅くなっても止まらない */
static void test_glitch(void)
{
  printf("[4] 5ms だけ遅くなって戻ったら（ノイズ）、止まらない\n");
  reset();
  run_up_to(1000.0f, 3000.0f);
  for (int i = 0; i < 10; i++) {
    tick(0.0f);
  }
  for (int i = 0; i < 100; i++) {
    tick(trajectory_get_speed_mm_s());
  }
  check(!s_failsafe, "10ms より短い遅れでは止まらない");
}

int main(void)
{
  test_normal();
  test_stall();
  test_slow();
  test_glitch();

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗 %d 件\n", s_fail_count);
  return 1;
}
