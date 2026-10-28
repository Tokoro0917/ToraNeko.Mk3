/*
 * test_turn.c
 * 最短走行のターン（大回り・斜め）の前距離・後距離を、PC で計算して確かめる。実機がなくても動く
 *
 *   ./test_turn          config/turn_params.h の値で、どのターンも決めた位置・向きで終わるかを調べる（make で実行）
 *   ./test_turn --calc   ターンの速さ・角加速度（config/turn_params.h の値）から、各ターンの最高角速度・前距離・後距離を
 *                        計算して、config/turn_params.h に貼る行を出す
 *
 * ■ 計算のしかた
 * ターンは「前距離（まっすぐ）→ スラローム（前に進む速さは一定、角速度は台形）→ 後距離（まっすぐ）」。
 * スラロームのあいだに進む位置は、本物の ctrl/trajectory.c を 0.5ms ずつ回して、目標の速さと角度から足していく
 * （探索の小回りの前距離・後距離 4.6・4.7mm と同じ計算。test_trajectory.c の [6]）。
 *
 * 終わりの位置（ターンを始めた位置と向きから見て、左へ何 mm・前へ何 mm）は、区画 90mm・半区画 45mm の迷路の形で決まる
 * （下の TURN_* の表。左に曲がるとき。右はこれを左右に裏返したもの）。
 *   - 前距離 + スラローム + 後距離（後距離は曲がったあとの向き）で、ちょうどその位置になるように、前距離と後距離を決める
 *   - 最高角速度は、前距離と後距離の短いほうが TURN_MARGIN_MM になるように決める（大きいほど小さく回り、まっすぐが長くなる）
 *   - 大回り 180° だけは、横の位置がスラロームだけで決まるので、横がちょうど 90mm になる最高角速度にする
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "config/robot_params.h"
#include "config/turn_params.h"
#include "ctrl/trajectory.h"

#define TURN_MARGIN_MM 3.0f   /* 前距離・後距離の短いほうの長さ [mm]（--calc で最高角速度を決めるとき） */
#define MAX_STEPS 100000      /* これだけ回しても終わらなければ失敗 */
#define PI_F 3.14159265f
#define SQRT2_F 1.41421356f

static int s_fail_count = 0;

/* ok が false なら失敗として数え、message を表示する */
static void check(bool ok, const char *message)
{
  if (!ok) {
    s_fail_count = s_fail_count + 1;
    printf("  失敗: %s\n", message);
  }
}

/* ターンの形（左に曲がるとき） */
typedef struct {
  const char *name;      /* 表示する名前 */
  const char *prefix;    /* config/turn_params.h の名前の頭 */
  float angle_deg;       /* 曲がる角度 */
  float target_left_mm;  /* 終わりの位置：始めた位置から左へ */
  float target_forward_mm; /* 終わりの位置：始めた位置から前へ */
} turn_shape_t;

/* 始めた位置は、大回り・斜め入りは区画のまん中、斜め出・V90 は壁の辺のまん中（斜めの通り道の上）。
 * 終わりの位置は、大回り・斜め出は区画のまん中、斜め入り・V90 は壁の辺のまん中 */
#define TURN_COUNT 7
static const turn_shape_t TURNS[TURN_COUNT] = {
  {"大回り 90°", "BIG90", 90.0f, 90.0f, 90.0f},
  {"大回り 180°", "BIG180", 180.0f, 90.0f, 0.0f},
  {"斜め入り 45°", "IN45", 45.0f, 45.0f, 90.0f},
  {"斜め入り 135°", "IN135", 135.0f, 90.0f, 45.0f},
  {"斜め出 45°", "OUT45", 45.0f, 45.0f / SQRT2_F, 135.0f / SQRT2_F},
  {"斜め出 135°", "OUT135", 135.0f, 135.0f / SQRT2_F, 45.0f / SQRT2_F},
  {"V90°", "V90", 90.0f, 90.0f / SQRT2_F, 90.0f / SQRT2_F},
};

/* スラロームだけで進む位置（始めた向きから見て、左へ・前へ） */
typedef struct {
  float left_mm;
  float forward_mm;
} arc_t;

/* trajectory.c でスラロームを 1 回走らせて、進んだ位置を足していく */
static arc_t simulate_arc(float angle_deg, float speed_mm_s, float omega_deg_s, float alpha_deg_s2)
{
  arc_t arc = {0.0f, 0.0f};
  trajectory_reset();
  trajectory_start_slalom(angle_deg, speed_mm_s, omega_deg_s, alpha_deg_s2);
  int steps = 0;
  do {
    trajectory_update();
    float angle_rad = trajectory_get_angle_deg() * PI_F / 180.0f;
    arc.forward_mm = arc.forward_mm + trajectory_get_speed_mm_s() * cosf(angle_rad) * CTRL_PERIOD_S;
    arc.left_mm = arc.left_mm + trajectory_get_speed_mm_s() * sinf(angle_rad) * CTRL_PERIOD_S;
    steps = steps + 1;
  } while (!trajectory_is_done() && steps < MAX_STEPS);
  return arc;
}

/* スラロームで進む位置から、前距離と後距離を決める。
 *   左：スラロームの左 + 後距離 × sin(角度) = 目標の左
 *   前：前距離 + スラロームの前 + 後距離 × cos(角度) = 目標の前
 * 大回り 180° は sin(角度) が 0 なので、前距離と後距離を同じにする（前と後ろで打ち消して、前の位置は変わらない） */
static void solve_pre_post(const turn_shape_t *turn, arc_t arc, float *pre_mm, float *post_mm)
{
  float angle_rad = turn->angle_deg * PI_F / 180.0f;
  float sin_value = sinf(angle_rad);
  if (fabsf(sin_value) < 0.01f) {
    *pre_mm = TURN_MARGIN_MM;
    *post_mm = TURN_MARGIN_MM;
    return;
  }
  *post_mm = (turn->target_left_mm - arc.left_mm) / sin_value;
  *pre_mm = turn->target_forward_mm - arc.forward_mm - *post_mm * cosf(angle_rad);
}

/* 最高角速度 omega_deg_s で曲がったときの「決め手の値」。--calc は、これが 0 になる最高角速度を探す。
 *   大回り 180°：スラロームの左 − 90mm（角速度を上げると小さく回り、減る）
 *   ほか：前距離と後距離の短いほう − TURN_MARGIN_MM（角速度を上げると増える） */
static float omega_criterion(const turn_shape_t *turn, float omega_deg_s)
{
  arc_t arc = simulate_arc(turn->angle_deg, FAST_TURN_SPEED_MM_S, omega_deg_s, FAST_TURN_ALPHA_DEG_S2);
  if (turn->angle_deg > 179.0f) {
    return turn->target_left_mm - arc.left_mm;  /* 角速度を上げると増えるように、引く順を逆にする */
  }
  float pre_mm;
  float post_mm;
  solve_pre_post(turn, arc, &pre_mm, &post_mm);
  float shorter_mm = pre_mm;
  if (post_mm < shorter_mm) {
    shorter_mm = post_mm;
  }
  return shorter_mm - TURN_MARGIN_MM;
}

/* --calc：各ターンの最高角速度・前距離・後距離を計算して、config/turn_params.h に貼る行を出す */
static int calculate(void)
{
  printf("/* ターンの速さ %.0fmm/s、角加速度 %.0fdeg/s²、前距離・後距離の短いほう %.1fmm で計算（test/test_turn.c --calc） */\n",
         FAST_TURN_SPEED_MM_S, FAST_TURN_ALPHA_DEG_S2, TURN_MARGIN_MM);
  for (int i = 0; i < TURN_COUNT; i++) {
    const turn_shape_t *turn = &TURNS[i];
    /* 角速度を半分ずつ狭めて、決め手の値が 0 になるところを探す（二分法） */
    float low_deg_s = 100.0f;
    float high_deg_s = 5000.0f;
    if (omega_criterion(turn, high_deg_s) < 0.0f || omega_criterion(turn, low_deg_s) > 0.0f) {
      printf("/* %s：角速度 100〜5000deg/s では決められない（角加速度を変える） */\n", turn->name);
      continue;
    }
    for (int n = 0; n < 40; n++) {
      float middle_deg_s = (low_deg_s + high_deg_s) / 2.0f;
      if (omega_criterion(turn, middle_deg_s) < 0.0f) {
        low_deg_s = middle_deg_s;
      } else {
        high_deg_s = middle_deg_s;
      }
    }
    float omega_deg_s = high_deg_s;
    arc_t arc = simulate_arc(turn->angle_deg, FAST_TURN_SPEED_MM_S, omega_deg_s, FAST_TURN_ALPHA_DEG_S2);
    float pre_mm;
    float post_mm;
    solve_pre_post(turn, arc, &pre_mm, &post_mm);
    printf("/* %s */\n", turn->name);
    printf("#define %s_OMEGA_DEG_S %.1ff\n", turn->prefix, omega_deg_s);
    printf("#define %s_PRE_MM %.1ff\n", turn->prefix, pre_mm);
    printf("#define %s_POST_MM %.1ff\n", turn->prefix, post_mm);
  }
  return 0;
}

/* config/turn_params.h の値で、ターンが決めた位置・向きで終わるかを調べる（ずれ 0.5mm まで） */
static void check_turn(const turn_shape_t *turn, float omega_deg_s, float pre_mm, float post_mm)
{
  arc_t arc = simulate_arc(turn->angle_deg, FAST_TURN_SPEED_MM_S, omega_deg_s, FAST_TURN_ALPHA_DEG_S2);
  float angle_rad = turn->angle_deg * PI_F / 180.0f;
  float end_left_mm = arc.left_mm + post_mm * sinf(angle_rad);
  float end_forward_mm = pre_mm + arc.forward_mm + post_mm * cosf(angle_rad);
  float error_left_mm = end_left_mm - turn->target_left_mm;
  float error_forward_mm = end_forward_mm - turn->target_forward_mm;
  printf("  %-16s 最高 %6.1fdeg/s、前 %5.1fmm、後 %5.1fmm → 左 %6.1f・前 %6.1fmm（目標 %5.1f・%5.1f）\n",
         turn->name, omega_deg_s, pre_mm, post_mm, end_left_mm, end_forward_mm,
         turn->target_left_mm, turn->target_forward_mm);
  check(fabsf(error_left_mm) < 0.5f && fabsf(error_forward_mm) < 0.5f, "決めた位置で終わる（±0.5mm）");
  check(pre_mm >= 0.0f && post_mm >= 0.0f, "前距離・後距離が 0 以上");
  check(fabsf(trajectory_get_angle_deg() - turn->angle_deg) < 0.01f, "ちょうど決めた角度だけ曲がる");
}

/* 速さを k 倍、最高角速度を k 倍、角加速度を k² 倍にしても、同じ位置で終わるか（遅く入ったターン。ctrl/run.c の fast_turn()） */
static void check_slow_turn(const turn_shape_t *turn, float omega_deg_s, float ratio)
{
  arc_t normal = simulate_arc(turn->angle_deg, FAST_TURN_SPEED_MM_S, omega_deg_s, FAST_TURN_ALPHA_DEG_S2);
  arc_t slow = simulate_arc(turn->angle_deg, FAST_TURN_SPEED_MM_S * ratio, omega_deg_s * ratio,
                            FAST_TURN_ALPHA_DEG_S2 * ratio * ratio);
  printf("  %-16s 速さ %.1f 倍：左 %6.2f → %6.2f、前 %6.2f → %6.2f mm\n", turn->name, ratio,
         normal.left_mm, slow.left_mm, normal.forward_mm, slow.forward_mm);
  check(fabsf(normal.left_mm - slow.left_mm) < 0.3f && fabsf(normal.forward_mm - slow.forward_mm) < 0.3f,
        "遅く曲がっても、スラロームで進む位置が同じ（±0.3mm）");
}

int main(int argc, char *argv[])
{
  if (argc >= 2 && strcmp(argv[1], "--calc") == 0) {
    return calculate();
  }

  printf("[1] 最短走行のターン（config/turn_params.h の値。速さ %.0fmm/s）\n", FAST_TURN_SPEED_MM_S);
  check_turn(&TURNS[0], BIG90_OMEGA_DEG_S, BIG90_PRE_MM, BIG90_POST_MM);
  check_turn(&TURNS[1], BIG180_OMEGA_DEG_S, BIG180_PRE_MM, BIG180_POST_MM);
  check_turn(&TURNS[2], IN45_OMEGA_DEG_S, IN45_PRE_MM, IN45_POST_MM);
  check_turn(&TURNS[3], IN135_OMEGA_DEG_S, IN135_PRE_MM, IN135_POST_MM);
  check_turn(&TURNS[4], OUT45_OMEGA_DEG_S, OUT45_PRE_MM, OUT45_POST_MM);
  check_turn(&TURNS[5], OUT135_OMEGA_DEG_S, OUT135_PRE_MM, OUT135_POST_MM);
  check_turn(&TURNS[6], V90_OMEGA_DEG_S, V90_PRE_MM, V90_POST_MM);

  printf("[2] 遅く入ったターン（速さ k 倍、最高角速度 k 倍、角加速度 k² 倍）\n");
  check_slow_turn(&TURNS[0], BIG90_OMEGA_DEG_S, 0.5f);
  check_slow_turn(&TURNS[2], IN45_OMEGA_DEG_S, 0.4f);
  check_slow_turn(&TURNS[3], IN135_OMEGA_DEG_S, 0.6f);

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗 %d 件\n", s_fail_count);
  return 1;
}
