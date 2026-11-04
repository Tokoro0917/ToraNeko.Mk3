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
 *   5. 吸引しているときは、摩擦の分の電圧を足す
 *   6. 斜めの補正：左の柱に近い（ずれ +）と右へ、右に近いと左へ曲げる。速さに比例し、上限で止まる。切ってあれば曲げない
 *   7. 向きを戻す：使える横の壁がないとき、迷路の向きからずれていたら戻す。壁があるとき・切ってあるときは戻さない。
 *      速さに比例し、上限で止まる。基準の向きのリセット（手で・両側の壁が続いたとき）、始め直しても基準が残ること
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "config/ctrl_gains.h"
#include "config/robot_params.h"
#include "ctrl/control.h"
#include "ctrl/odometry.h"
#include "ctrl/side_edge.h"
#include "ctrl/trajectory.h"
#include "ctrl/wall_ctrl.h"
#include "drv/fan.h"
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

static float s_fake_angle_deg = 0.0f;  /* 仮のジャイロが答える角度（テストで決める） */
float imu_get_angle_deg(void)
{
  return s_fake_angle_deg;
}

float imu_get_gyro_z_dps(void)
{
  return 0.0f;
}

/* 前後の加速度（走行ログに残すだけなので、テストでは 0） */
float imu_get_accel_y_mps2(void)
{
  return 0.0f;
}

static float s_last_left_v = 0.0f;   /* 最後にモーターにかけた電圧（左） */
static float s_last_right_v = 0.0f;  /* 最後にモーターにかけた電圧（右） */
void motor_set_voltage(float left_v, float right_v)
{
  s_last_left_v = left_v;
  s_last_right_v = right_v;
}

/* 仮の吸引ファン：s_fake_fan_v を答える（テストで吸引あり・なしを切り替える） */
static float s_fake_fan_v = 0.0f;
float fan_get_voltage(void)
{
  return s_fake_fan_v;
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

static int s_fake_walls_used = 0;  /* 仮の、横の壁の補正に使える壁（0 なし、1 左、2 右、3 両方） */
int wall_ctrl_get_side_walls_used(void)
{
  return s_fake_walls_used;
}

void wall_ctrl_reset_end_check(float distance_mm)
{
  (void)distance_mm;
}

void wall_ctrl_update_end_check(float distance_mm)
{
  (void)distance_mm;
}

/* 壁の終わり・柱の端を見つける（ctrl/side_edge.c）。このテストでは使わないので、何もしない */
void side_edge_reset(void)
{
}

void side_edge_update(float distance_mm, int32_t left_value, int32_t right_value, bool is_straight)
{
  (void)distance_mm;
  (void)left_value;
  (void)right_value;
  (void)is_straight;
}

int32_t wall_ctrl_get_front_left(void)
{
  return 0;
}

int32_t wall_ctrl_get_front_right(void)
{
  return 0;
}

/* 仮の斜めの補正のずれ：s_fake_diag_error を答える（テストで決める） */
static float s_fake_diag_error = 0.0f;
float wall_ctrl_get_diag_error(void)
{
  return s_fake_diag_error;
}

int wall_ctrl_get_diag_used(void)
{
  if (s_fake_diag_error > 0.0f) {
    return 1;
  }
  if (s_fake_diag_error < 0.0f) {
    return 2;
  }
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

/* 5. 吸引しているときは、前に進む分に、吸引で増えた摩擦の分の電圧を足す */
static void test_suction_friction(void)
{
  printf("[5] 吸引しているときは、摩擦の分 FF_SUCTION_FRICTION_V を足す\n");
  s_fake_fan_v = 0.0f;
  reset();
  run_up_to(300.0f, 1000.0f);
  tick(trajectory_get_speed_mm_s());
  float left_without_v = s_last_left_v;
  float right_without_v = s_last_right_v;

  s_fake_fan_v = 5.0f;
  reset();
  run_up_to(300.0f, 1000.0f);
  tick(trajectory_get_speed_mm_s());
  float left_diff_v = s_last_left_v - left_without_v;
  float right_diff_v = s_last_right_v - right_without_v;
  s_fake_fan_v = 0.0f;

  printf("  吸引ありとなしの電圧の差：左 %.3fV、右 %.3fV\n", left_diff_v, right_diff_v);
  check(fabsf(left_diff_v - FF_SUCTION_FRICTION_V) < 0.001f, "左は FF_SUCTION_FRICTION_V だけ大きい");
  check(fabsf(right_diff_v - FF_SUCTION_FRICTION_V) < 0.001f, "右も FF_SUCTION_FRICTION_V だけ大きい");
}

/* 速さ speed_mm_s で斜めに走っていて、斜めの補正のずれが diag_error のときの、右 − 左 の電圧 [V]（+ なら左へ曲げる）。
 * use_correction は斜めの補正を入れるか */
static float diag_turn_voltage(float speed_mm_s, float diag_error, bool use_correction)
{
  reset();
  control_set_on_diagonal(true);
  control_set_diag_control(use_correction);
  run_up_to(speed_mm_s, 6000.0f);
  s_fake_diag_error = diag_error;
  tick(trajectory_get_speed_mm_s());
  s_fake_diag_error = 0.0f;
  control_set_on_diagonal(false);
  control_set_diag_control(false);
  return s_last_right_v - s_last_left_v;
}

/* 6. 斜めの補正の向き・大きさ */
static void test_diag_control(void)
{
  printf("[6] 斜めの補正：柱に近い側と反対へ曲げる（速さに比例、上限あり）\n");
  float none_v = diag_turn_voltage(800.0f, 0.0f, true);
  float left_close_v = diag_turn_voltage(800.0f, 200.0f, true);
  float right_close_v = diag_turn_voltage(800.0f, -200.0f, true);
  float off_v = diag_turn_voltage(800.0f, 200.0f, false);
  float slow_v = diag_turn_voltage(400.0f, 200.0f, true);
  float big_v = diag_turn_voltage(800.0f, 100000.0f, true);
  float bigger_v = diag_turn_voltage(800.0f, 200000.0f, true);
  printf("  右 − 左 の電圧：ずれなし %.4fV、左に近い %.4fV、右に近い %.4fV、補正なし %.4fV、400mm/s %.4fV\n",
         none_v, left_close_v, right_close_v, off_v, slow_v);
  check(fabsf(none_v) < 0.0001f, "ずれがなければ曲げない");
  check(left_close_v < -0.0001f, "左の柱に近ければ（ずれ +）、右へ曲げる（右 − 左 が −）");
  check(right_close_v > 0.0001f, "右の柱に近ければ（ずれ −）、左へ曲げる");
  check(fabsf(off_v) < 0.0001f, "斜めの補正を切ってあれば曲げない");
  check(fabsf(slow_v - left_close_v / 2.0f) < 0.0001f, "半分の速さなら、曲げる量も半分");
  check(fabsf(big_v - bigger_v) < 0.0001f, "大きなずれでは、上限 DIAG_OMEGA_LIMIT_DEG_S で止まる");
}

/* 速さ speed_mm_s で縦横に走っていて、ジャイロの角度が angle_deg のときの、右 − 左 の電圧 [V]（+ なら左へ曲げる）。
 * hold は向きを戻すのを入れるか、walls_used は使える横の壁。制御は 0° で始める */
static float hold_turn_voltage(float speed_mm_s, float angle_deg, bool hold, int walls_used)
{
  s_fake_angle_deg = 0.0f;
  s_fake_walls_used = walls_used;
  reset();
  control_set_wall_control(true);
  control_set_heading_hold(hold);
  run_up_to(speed_mm_s, 6000.0f);
  s_fake_angle_deg = angle_deg;
  tick(trajectory_get_speed_mm_s());
  control_set_wall_control(false);
  s_fake_angle_deg = 0.0f;
  s_fake_walls_used = 0;
  return s_last_right_v - s_last_left_v;
}

/* 7. 向きを戻す */
static void test_heading_hold(void)
{
  printf("[7] 向きを戻す：使える横の壁がないとき、迷路の向きへ戻す\n");
  float none_v = hold_turn_voltage(800.0f, 0.0f, true, 0);
  /* 1° のずれなら、800mm/s で 16deg/s（上限 45deg/s より小さいので、速さに比例するかを比べられる） */
  float left_v = hold_turn_voltage(800.0f, 1.0f, true, 0);
  float right_v = hold_turn_voltage(800.0f, -1.0f, true, 0);
  float near_90_v = hold_turn_voltage(800.0f, 91.0f, true, 0);
  float wall_v = hold_turn_voltage(800.0f, 1.0f, true, 3);
  float off_v = hold_turn_voltage(800.0f, 1.0f, false, 0);
  float slow_v = hold_turn_voltage(400.0f, 1.0f, true, 0);
  float big_v = hold_turn_voltage(800.0f, 30.0f, true, 0);
  float bigger_v = hold_turn_voltage(800.0f, 40.0f, true, 0);
  printf("  右 − 左 の電圧：ずれなし %.4fV、左へ 1° %.4fV、右へ 1° %.4fV、91° %.4fV、壁あり %.4fV、切ってある %.4fV、400mm/s %.4fV\n",
         none_v, left_v, right_v, near_90_v, wall_v, off_v, slow_v);
  check(fabsf(none_v) < 0.0001f, "ずれがなければ曲げない");
  check(left_v < -0.0001f, "左を向いていたら（+1°）、右へ曲げる");
  check(right_v > 0.0001f, "右を向いていたら（−1°）、左へ曲げる");
  check(fabsf(near_90_v - left_v) < 0.0001f, "91° なら、迷路の向き 90° から +1° として、同じだけ右へ曲げる");
  check(fabsf(wall_v) < 0.0001f, "使える横の壁があるときは、向きを戻さない（横の壁の補正だけ）");
  check(fabsf(off_v) < 0.0001f, "向きを戻すのを入れていなければ曲げない");
  check(fabsf(slow_v - left_v / 2.0f) < 0.0001f, "半分の速さなら、曲げる量も半分");
  check(fabsf(big_v - bigger_v) < 0.0001f, "大きなずれでは、上限 HEADING_HOLD_LIMIT_DEG_S で止まる");

  /* 基準の向きのリセット：3° 向いているところでリセットすると、そこが迷路の向きになり、曲げなくなる */
  s_fake_angle_deg = 0.0f;
  s_fake_walls_used = 0;
  reset();
  control_set_wall_control(true);
  control_set_heading_hold(true);
  run_up_to(800.0f, 6000.0f);
  s_fake_angle_deg = 3.0f;
  control_reset_heading_reference();
  tick(trajectory_get_speed_mm_s());
  float after_reset_v = s_last_right_v - s_last_left_v;
  /* 始め直し（control_restart）では、基準の向きも、向きを戻すかどうかも、そのまま（速くするあいだは 3° のまま） */
  control_restart();
  run_up_to(800.0f, 6000.0f);
  s_fake_angle_deg = 4.0f;
  tick(trajectory_get_speed_mm_s());
  float after_restart_v = s_last_right_v - s_last_left_v;
  check(fabsf(after_reset_v) < 0.0001f, "リセットした向き（3°）を、迷路の向きとみなす");
  check(fabsf(after_restart_v - left_v) < 0.0001f, "始め直しても基準（3°）が残り、4° なら +1° として右へ曲げる");

  /* 両側の壁が続いたら、基準の向きをリセットする（入れたときだけ） */
  s_fake_angle_deg = 0.0f;
  s_fake_walls_used = 3;
  reset();
  control_set_wall_control(true);
  control_set_heading_hold(true);
  control_set_heading_auto_reset(true);
  s_fake_angle_deg = 2.0f;
  run_up_to(800.0f, 6000.0f);
  for (int i = 0; i < 400; i++) {
    tick(trajectory_get_speed_mm_s());  /* 800mm/s で 0.2 秒 = 160mm。HEADING_RESET_DISTANCE_MM（90mm）より長く、両側の壁で走る */
  }
  s_fake_walls_used = 0;
  tick(trajectory_get_speed_mm_s());
  float auto_reset_v = s_last_right_v - s_last_left_v;
  check(fabsf(auto_reset_v) < 0.0001f, "両側の壁で 90mm より長く走ったら、そのときの向き（2°）を迷路の向きとみなす");
  s_fake_angle_deg = 0.0f;
  control_set_wall_control(false);
}

int main(void)
{
  test_normal();
  test_stall();
  test_slow();
  test_glitch();
  test_suction_friction();
  test_diag_control();
  test_heading_hold();

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗 %d 件\n", s_fail_count);
  return 1;
}
