/*
 * test_trajectory.c
 * 目標の速さ・角速度を作る処理（ctrl/trajectory.c）の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * ■ テストの内容
 *   1. 台形：止まったところから 360mm、最高速 200mm/s、加速度 1000mm/s² で走って止まる
 *      （最高速を超えない、加速度を超えない、ちょうど 360mm で 0 になる、かかる時間が計算と合う）
 *   2. 三角形：距離が短くて最高速まで届かないとき
 *   3. 終わりの速さが 0 でないとき（その速さで終わり、次の動きはその速さから始まる）
 *   4. その場旋回：左に 180°、右に 90°（角速度の向き、ちょうどの角度で止まる、直進の値は 0 のまま）
 *   5. 旋回のあとの直進は、止まったところから始まる
 *   7. 動きと動きのあいだ（探索の計算のあいだ）に進んだ分は、次の直進から引く
 *   6. スラローム：前に進む速さは一定のまま、角速度を台形にして 90° 曲がる。曲がったあとの直進は、その速さから始まる
 *   8. 直進の途中で距離を足し引きする（壁切れ）：足した分だけ長く・短く進み、終わりの速さで終わる
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "config/robot_params.h"
#include "ctrl/trajectory.h"

#define MAX_STEPS 100000  /* これだけ回しても終わらなければ失敗（50 秒ぶん） */

static int s_fail_count = 0;

/* ok が false なら失敗として数え、message を表示する */
static void check(bool ok, const char *message)
{
  if (!ok) {
    s_fail_count = s_fail_count + 1;
    printf("  失敗: %s\n", message);
  }
}

/* 1 つの動きを終わるまで回した結果 */
typedef struct {
  int steps;              /* 終わるまでの周期の数 */
  float top_speed;        /* いちばん速かった目標の速さ（直進は mm/s、旋回は deg/s の大きさ） */
  float max_change;       /* 1 周期で変わった速さのいちばん大きいもの */
  bool went_negative;     /* 目標の速さがマイナスになったか */
} run_result_t;

/* 今の目標の速さ（直進は速さ、旋回は角速度の大きさ。どちらか片方は 0 なので、足せばよい） */
static float current_speed(void)
{
  return trajectory_get_speed_mm_s() + fabsf(trajectory_get_omega_deg_s());
}

/* 動きが終わるまで trajectory_update() を回して、結果を調べる */
static run_result_t run_until_done(void)
{
  run_result_t result = {0, 0.0f, 0.0f, false};
  float last_speed = current_speed();
  while (!trajectory_is_done() && result.steps < MAX_STEPS) {
    trajectory_update();
    float speed = current_speed();
    if (speed > result.top_speed) {
      result.top_speed = speed;
    }
    if (fabsf(speed - last_speed) > result.max_change) {
      result.max_change = fabsf(speed - last_speed);
    }
    if (speed < 0.0f) {
      result.went_negative = true;
    }
    last_speed = speed;
    result.steps = result.steps + 1;
  }
  return result;
}

/* 1. 台形：360mm、最高速 200mm/s、加速度 1000mm/s² */
static void test_trapezoid(void)
{
  printf("[1] 台形加減速\n");
  trajectory_reset();
  trajectory_start_straight(360.0f, 200.0f, 0.0f, 1000.0f);
  run_result_t result = run_until_done();

  /* かかる時間：加速 0.2 秒（20mm）+ 最高速 320mm ÷ 200mm/s = 1.6 秒 + 減速 0.2 秒 = 2.0 秒 */
  float time_s = result.steps * CTRL_PERIOD_S;
  printf("  かかった時間 %.4f 秒（計算 2.0 秒）、最高速 %.1f mm/s\n", time_s, result.top_speed);
  check(trajectory_is_done(), "終わる");
  check(fabsf(trajectory_get_distance_mm() - 360.0f) < 0.001f, "ちょうど 360mm 進む");
  check(trajectory_get_speed_mm_s() == 0.0f, "終わったら速さ 0");
  check(fabsf(result.top_speed - 200.0f) < 0.001f, "最高速 200mm/s まで速くなり、超えない");
  check(fabsf(time_s - 2.0f) < 0.01f, "かかる時間が 2.0 秒（±0.01 秒）");
  /* 1 周期で変わる速さは 1000 × 0.0005 = 0.5mm/s。減速の最後は √ の形なので少し大きくなるが、数 mm/s まで */
  check(result.max_change < 5.0f, "1 周期で速さが急に変わらない");
  check(!result.went_negative, "速さがマイナスにならない");

  /* 終わったあとに回しても、止まったまま */
  trajectory_update();
  check(trajectory_get_speed_mm_s() == 0.0f && trajectory_get_distance_mm() == 360.0f, "終わったあとは止まったまま");
}

/* 2. 三角形：30mm、最高速 500mm/s、加速度 1000mm/s²（最高速まで届かない） */
static void test_triangle(void)
{
  printf("[2] 最高速まで届かないとき\n");
  trajectory_reset();
  trajectory_start_straight(30.0f, 500.0f, 0.0f, 1000.0f);
  run_result_t result = run_until_done();

  /* 半分の 15mm で最高になる：速さ = √(2 × 1000 × 15) ≒ 173mm/s、時間 = 2 × 173 ÷ 1000 ≒ 0.346 秒 */
  float time_s = result.steps * CTRL_PERIOD_S;
  printf("  かかった時間 %.4f 秒（計算 0.346 秒）、最高速 %.1f mm/s（計算 173.2）\n", time_s, result.top_speed);
  check(fabsf(trajectory_get_distance_mm() - 30.0f) < 0.001f, "ちょうど 30mm 進む");
  check(fabsf(result.top_speed - 173.2f) < 1.0f, "最高速は約 173mm/s");
  check(fabsf(time_s - 0.3464f) < 0.005f, "かかる時間が約 0.346 秒");
}

/* 3. 終わりの速さが 0 でないとき：90mm を 300mm/s で終わり、続けて 90mm を 300mm/s → 0 */
static void test_end_speed(void)
{
  printf("[3] 終わりの速さが 0 でないとき\n");
  trajectory_reset();
  trajectory_start_straight(90.0f, 300.0f, 300.0f, 2000.0f);
  run_until_done();
  check(fabsf(trajectory_get_speed_mm_s() - 300.0f) < 0.001f, "300mm/s で終わる");

  trajectory_start_straight(90.0f, 300.0f, 0.0f, 2000.0f);
  trajectory_update();
  check(fabsf(trajectory_get_speed_mm_s() - 300.0f) < 0.001f, "次の動きは 300mm/s から始まる");
  run_until_done();
  check(fabsf(trajectory_get_distance_mm() - 90.0f) < 0.001f && trajectory_get_speed_mm_s() == 0.0f,
        "次の動きで 90mm 進んで止まる");
}

/* 4. その場旋回：左に 180°（最高 720deg/s、7200deg/s²）、右に 90° */
static void test_pivot(void)
{
  printf("[4] その場旋回\n");
  trajectory_reset();
  trajectory_start_pivot(180.0f, 720.0f, 7200.0f);
  trajectory_update();
  check(trajectory_get_omega_deg_s() > 0.0f, "左回りは角速度が +");
  check(trajectory_get_speed_mm_s() == 0.0f && trajectory_get_accel_mm_s2() == 0.0f, "旋回中は直進の速さ・加速度が 0");
  run_result_t result = run_until_done();

  /* かかる時間：加速 0.1 秒（36°）+ 最高速 108° ÷ 720 = 0.15 秒 + 減速 0.1 秒 = 0.35 秒（最初の 1 回ぶんを足す） */
  float time_s = (result.steps + 1) * CTRL_PERIOD_S;
  printf("  かかった時間 %.4f 秒（計算 0.35 秒）、最高の角速度 %.1f deg/s\n", time_s, result.top_speed);
  check(fabsf(trajectory_get_angle_deg() - 180.0f) < 0.001f, "ちょうど 180° 回る");
  check(trajectory_get_omega_deg_s() == 0.0f, "終わったら角速度 0");
  check(fabsf(time_s - 0.35f) < 0.005f, "かかる時間が約 0.35 秒");
  check(fabsf(result.top_speed - 720.0f) < 0.001f, "最高の角速度 720deg/s まで速くなり、超えない");
  check(trajectory_get_distance_mm() == 0.0f, "旋回中は直進の距離が 0");

  trajectory_start_pivot(-90.0f, 720.0f, 7200.0f);
  trajectory_update();
  check(trajectory_get_omega_deg_s() < 0.0f, "右回りは角速度が -");
  run_until_done();
  check(fabsf(trajectory_get_angle_deg() + 90.0f) < 0.001f, "ちょうど -90° 回る");
}

/* 5. 旋回のあとの直進は、止まったところから始まる */
static void test_straight_after_pivot(void)
{
  printf("[5] 旋回のあとの直進\n");
  trajectory_reset();
  trajectory_start_pivot(90.0f, 720.0f, 7200.0f);
  run_until_done();
  trajectory_start_straight(90.0f, 300.0f, 0.0f, 2000.0f);
  trajectory_update();
  check(fabsf(trajectory_get_speed_mm_s() - 1.0f) < 0.001f, "直進は 0 から 1 周期ぶん（2000 × 0.0005 = 1mm/s）だけ速くなったところ");
  check(trajectory_get_omega_deg_s() == 0.0f, "直進中は角速度が 0");
  run_until_done();
  check(fabsf(trajectory_get_distance_mm() - 90.0f) < 0.001f, "90mm 進んで止まる");
}

/* 6. スラローム：300mm/s で走りながら左に 90°（最高 600deg/s、8000deg/s²） */
static void test_slalom(void)
{
  printf("[6] スラローム\n");
  trajectory_reset();
  trajectory_start_straight(45.0f, 300.0f, 300.0f, 1000.0f);
  run_until_done();
  check(fabsf(trajectory_get_speed_mm_s() - 300.0f) < 0.001f, "45mm で 300mm/s になる（1000mm/s² で、ちょうど）");

  trajectory_start_slalom(90.0f, 300.0f, 600.0f, 8000.0f);
  trajectory_update();
  check(trajectory_get_kind() == TRAJECTORY_SLALOM, "動きの種類がスラロームになる");
  check(fabsf(trajectory_get_speed_mm_s() - 300.0f) < 0.001f, "曲がっているあいだも、前に進む速さは 300mm/s");
  check(trajectory_get_accel_mm_s2() == 0.0f, "曲がっているあいだは、前に進む加速度は 0");
  check(trajectory_get_omega_deg_s() > 0.0f, "左に曲がるので、角速度が +");

  /* 曲がるあいだに進む位置を、目標の速さと角度から足していく（PC での計算と同じ：前 40.4mm、横 40.3mm） */
  float forward_mm = 300.0f * CTRL_PERIOD_S;  /* 最初に回した 1 周期ぶん（ほとんど前） */
  float side_mm = 0.0f;
  int steps = 1;
  while (!trajectory_is_done() && steps < MAX_STEPS) {
    trajectory_update();
    float angle_rad = trajectory_get_angle_deg() * 3.14159265f / 180.0f;
    forward_mm = forward_mm + trajectory_get_speed_mm_s() * cosf(angle_rad) * CTRL_PERIOD_S;
    side_mm = side_mm + trajectory_get_speed_mm_s() * sinf(angle_rad) * CTRL_PERIOD_S;
    steps = steps + 1;
  }
  printf("  曲がるあいだに 前 %.1f mm、横 %.1f mm 進んだ（計算 40.4、40.3）、%.3f 秒\n",
         forward_mm, side_mm, steps * CTRL_PERIOD_S);
  check(fabsf(trajectory_get_angle_deg() - 90.0f) < 0.001f, "ちょうど 90° 曲がる");
  check(trajectory_get_omega_deg_s() == 0.0f, "曲がり終わったら角速度 0");
  check(fabsf(forward_mm - 40.4f) < 0.5f && fabsf(side_mm - 40.3f) < 0.5f, "曲がるあいだの位置が計算と合う（±0.5mm）");

  trajectory_start_straight(45.0f, 300.0f, 0.0f, 1000.0f);
  trajectory_update();
  check(trajectory_get_speed_mm_s() > 299.0f, "曲がったあとの直進は、300mm/s から始まる");
  run_until_done();
  check(fabsf(trajectory_get_distance_mm() - 45.0f) < 0.001f && trajectory_get_speed_mm_s() == 0.0f,
        "45mm で止まる（1000mm/s² で、ちょうど）");
}

/* 7. 1 区画（90mm）を 300mm/s のまま進み、30ms あいてから次の 1 区画を頼む。
 *    あいだも 300mm/s で進む（9mm）ので、2 つ合わせて、ちょうど 180mm 進むはず */
static void test_gap_between_moves(void)
{
  printf("[7] 動きと動きのあいだに進んだ分\n");
  trajectory_reset();
  trajectory_start_straight(90.0f, 300.0f, 300.0f, 1000.0f);
  float total_mm = 0.0f;
  int steps = 0;
  while (!trajectory_is_done() && steps < MAX_STEPS) {
    trajectory_update();
    total_mm = total_mm + trajectory_get_speed_mm_s() * CTRL_PERIOD_S;
    steps = steps + 1;
  }
  /* 30ms（60 周期）のあいだ、次の動きを頼まない（探索が計算しているあいだ） */
  for (int i = 0; i < 60; i++) {
    trajectory_update();
    total_mm = total_mm + trajectory_get_speed_mm_s() * CTRL_PERIOD_S;
  }
  trajectory_start_straight(90.0f, 300.0f, 300.0f, 1000.0f);
  steps = 0;
  while (!trajectory_is_done() && steps < MAX_STEPS) {
    trajectory_update();
    total_mm = total_mm + trajectory_get_speed_mm_s() * CTRL_PERIOD_S;
    steps = steps + 1;
  }
  printf("  2 つの直進とあいだで進んだ距離 %.2f mm（ちょうど 180mm になるはず）\n", total_mm);
  check(fabsf(total_mm - 180.0f) < 0.5f, "あいだに進んだ分を、次の直進から引いている（合わせて 180mm ±0.5mm）");

  /* 止まったあと（終わりの速さ 0）は、あいだに進まないので、次の直進は引かない */
  trajectory_reset();
  trajectory_start_straight(45.0f, 300.0f, 0.0f, 1000.0f);
  run_until_done();
  for (int i = 0; i < 60; i++) {
    trajectory_update();
  }
  trajectory_start_straight(45.0f, 300.0f, 0.0f, 1000.0f);
  run_until_done();
  check(fabsf(trajectory_get_distance_mm() - 45.0f) < 0.001f, "止まったあとの直進は、引かずにそのまま 45mm 進む");
}

/* 8. 直進の途中で距離を直す（壁切れで使う trajectory_add_distance()） */
static void test_add_distance(void)
{
  printf("[8] 直進の途中で距離を直す（壁切れ）\n");
  trajectory_reset();
  trajectory_start_straight(90.0f, 300.0f, 300.0f, 1000.0f);
  run_until_done();  /* 止まったところから 300mm/s にしておく（90mm で届く） */

  trajectory_start_straight(90.0f, 300.0f, 300.0f, 1000.0f);
  for (int i = 0; i < 200; i++) {  /* 0.1 秒 = 30mm 進める */
    trajectory_update();
  }
  trajectory_add_distance(5.0f);
  run_until_done();
  check(fabsf(trajectory_get_distance_mm() - 95.0f) < 0.001f, "途中で 5mm 足すと、95mm 進んで終わる");
  check(fabsf(trajectory_get_speed_mm_s() - 300.0f) < 0.001f, "終わりの速さ（300mm/s）で終わる");

  trajectory_start_straight(90.0f, 300.0f, 300.0f, 1000.0f);
  for (int i = 0; i < 200; i++) {
    trajectory_update();
  }
  trajectory_add_distance(-8.0f);
  run_until_done();
  check(fabsf(trajectory_get_distance_mm() - 82.0f) < 0.001f, "途中で 8mm 引くと、82mm 進んで終わる");

  trajectory_start_straight(90.0f, 300.0f, 300.0f, 1000.0f);
  for (int i = 0; i < 200; i++) {
    trajectory_update();
  }
  float moved_mm = trajectory_get_distance_mm();
  trajectory_add_distance(-100.0f);
  trajectory_update();
  check(trajectory_is_done() && trajectory_get_distance_mm() >= moved_mm,
        "もう進んだ量より短くはならない（すぐに終わる）");
}

int main(void)
{
  test_trapezoid();
  test_triangle();
  test_end_speed();
  test_pivot();
  test_straight_after_pivot();
  test_slalom();
  test_gap_between_moves();
  test_add_distance();

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗 %d 件\n", s_fail_count);
  return 1;
}
