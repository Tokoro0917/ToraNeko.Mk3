/*
 * test_trajectory.c
 * 目標の速さを作る処理（ctrl/trajectory.c）の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * ■ テストの内容
 *   1. 台形：止まったところから 360mm、最高速 200mm/s、加速度 1000mm/s² で走って止まる
 *      （最高速を超えない、加速度を超えない、ちょうど 360mm で 0 になる、かかる時間が計算と合う）
 *   2. 三角形：距離が短くて最高速まで届かないとき
 *   3. 終わりの速さが 0 でないとき（その速さで終わり、次の動きはその速さから始まる）
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
  float top_speed_mm_s;   /* いちばん速かった目標の速さ */
  float max_change_mm_s;  /* 1 周期で変わった速さのいちばん大きいもの */
  bool went_negative;     /* 目標の速さがマイナスになったか */
} run_result_t;

/* 動きが終わるまで trajectory_update() を回して、結果を調べる */
static run_result_t run_until_done(void)
{
  run_result_t result = {0, 0.0f, 0.0f, false};
  float last_speed = trajectory_get_speed_mm_s();
  while (!trajectory_is_done() && result.steps < MAX_STEPS) {
    trajectory_update();
    float speed = trajectory_get_speed_mm_s();
    if (speed > result.top_speed_mm_s) {
      result.top_speed_mm_s = speed;
    }
    if (fabsf(speed - last_speed) > result.max_change_mm_s) {
      result.max_change_mm_s = fabsf(speed - last_speed);
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
  printf("  かかった時間 %.4f 秒（計算 2.0 秒）、最高速 %.1f mm/s\n", time_s, result.top_speed_mm_s);
  check(trajectory_is_done(), "終わる");
  check(fabsf(trajectory_get_distance_mm() - 360.0f) < 0.001f, "ちょうど 360mm 進む");
  check(trajectory_get_speed_mm_s() == 0.0f, "終わったら速さ 0");
  check(fabsf(result.top_speed_mm_s - 200.0f) < 0.001f, "最高速 200mm/s まで速くなり、超えない");
  check(fabsf(time_s - 2.0f) < 0.01f, "かかる時間が 2.0 秒（±0.01 秒）");
  /* 1 周期で変わる速さは 1000 × 0.0005 = 0.5mm/s。減速の最後は √ の形なので少し大きくなるが、数 mm/s まで */
  check(result.max_change_mm_s < 5.0f, "1 周期で速さが急に変わらない");
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
  printf("  かかった時間 %.4f 秒（計算 0.346 秒）、最高速 %.1f mm/s（計算 173.2）\n", time_s, result.top_speed_mm_s);
  check(fabsf(trajectory_get_distance_mm() - 30.0f) < 0.001f, "ちょうど 30mm 進む");
  check(fabsf(result.top_speed_mm_s - 173.2f) < 1.0f, "最高速は約 173mm/s");
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

int main(void)
{
  test_trapezoid();
  test_triangle();
  test_end_speed();

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗 %d 件\n", s_fail_count);
  return 1;
}
