/*
 * test_turn.c
 * 最短走行のターン（大回り・斜め）の前距離・後距離を、PC で計算して確かめる。実機がなくても動く
 *
 *   ./test_turn          config/turn_params.h の値で、どのターンも決めた位置・向きで終わるかを調べる（make で実行）
 *   ./test_turn --calc   ターンごとの速さ・角加速度（config/turn_params.h の値）から、各ターンの最高角速度・前距離・後距離を
 *                        計算して、config/turn_params.h に貼る行を出す（組 1 と組 2〈T1000_〉の両方）
 *   ./test_turn --slip   実機で合わせた前距離・後距離（*_ADJUST_MM）が、「曲がっているあいだタイヤが空回りする」という
 *                        係数 1 つの式で説明できるかを調べる（下の「■ 空回りの式」）
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
 *
 * ■ 空回りの式（--slip。2026-11-04。KOGUMA の Turn-Sim_KOGUMA の「滑りの式」と同じ考え方）
 * 実機は計算より内側を走り、前距離・後距離を足す必要があった（config/turn_params.h）。曲がっているあいだ、タイヤが床の上で
 * 少しずつ空回りして、機体が床の上を進む速さが、エンコーダで測った速さより遅くなる、と考える。角速度はジャイロで制御して
 * いるので計算どおり。速さだけ遅いので、小さく回る（半径 ＝ 速さ ÷ 角速度）。空回りの割合は、横向きの加速度
 * （速さ × 角速度）に比例するとする：
 *     床の上の速さ ＝ 速さ ×（1 − 係数 × 横向きの加速度）
 * 係数を少しずつ変えて、7 つのターンで実機で足した分に、いちばん合う係数を探す。
 * （KOGUMA の式は「外へ横すべりする」角度で、ToraNeko は内側を走るので合わない。--slip で KOGUMA の式は使わない）
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
  float speed_mm_s;      /* そのターンを曲がる速さ（config/turn_params.h の *_SPEED_MM_S） */
  float alpha_deg_s2;    /* 角加速度（*_ALPHA_DEG_S2） */
  float straight_min_mm; /* 前距離・後距離の短いほう（ふつうは TURN_MARGIN_MM。斜め出 45° は OUT45_STRAIGHT_MIN_MM） */
} turn_shape_t;

/* 始めた位置は、大回り・斜め入りは区画のまん中、斜め出・V90 は壁の辺のまん中（斜めの通り道の上）。
 * 終わりの位置は、大回り・斜め出は区画のまん中、斜め入り・V90 は壁の辺のまん中 */
#define TURN_COUNT 7
static const turn_shape_t TURNS[TURN_COUNT] = {
  {"大回り 90°", "BIG90", 90.0f, 90.0f, 90.0f, BIG90_SPEED_MM_S, BIG90_ALPHA_DEG_S2, TURN_MARGIN_MM},
  /* 大回り 180° の幅は、実機で内側を走る分（BIG180_WIDTH_ADJUST_MM）だけ広く計算する（config/turn_params.h） */
  {"大回り 180°", "BIG180", 180.0f, 90.0f + BIG180_WIDTH_ADJUST_MM, 0.0f, BIG180_SPEED_MM_S, BIG180_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"斜め入り 45°", "IN45", 45.0f, 45.0f, 90.0f, IN45_SPEED_MM_S, IN45_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"斜め入り 135°", "IN135", 135.0f, 90.0f, 45.0f, IN135_SPEED_MM_S, IN135_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"斜め出 45°", "OUT45", 45.0f, 45.0f / SQRT2_F, 135.0f / SQRT2_F, OUT45_SPEED_MM_S, OUT45_ALPHA_DEG_S2, OUT45_STRAIGHT_MIN_MM},
  {"斜め出 135°", "OUT135", 135.0f, 135.0f / SQRT2_F, 45.0f / SQRT2_F, OUT135_SPEED_MM_S, OUT135_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"V90°", "V90", 90.0f, 90.0f / SQRT2_F, 90.0f / SQRT2_F, V90_SPEED_MM_S, V90_ALPHA_DEG_S2, TURN_MARGIN_MM},
};

/* 組 2（7 つとも 1000mm/s。config/turn_params.h の T1000_…）。形の決め方は組 1 と同じ */
static const turn_shape_t TURNS_1000[TURN_COUNT] = {
  {"大回り 90°", "T1000_BIG90", 90.0f, 90.0f, 90.0f, T1000_BIG90_SPEED_MM_S, T1000_BIG90_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"大回り 180°", "T1000_BIG180", 180.0f, 90.0f + T1000_BIG180_WIDTH_ADJUST_MM, 0.0f, T1000_BIG180_SPEED_MM_S, T1000_BIG180_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"斜め入り 45°", "T1000_IN45", 45.0f, 45.0f, 90.0f, T1000_IN45_SPEED_MM_S, T1000_IN45_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"斜め入り 135°", "T1000_IN135", 135.0f, 90.0f, 45.0f, T1000_IN135_SPEED_MM_S, T1000_IN135_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"斜め出 45°", "T1000_OUT45", 45.0f, 45.0f / SQRT2_F, 135.0f / SQRT2_F, T1000_OUT45_SPEED_MM_S, T1000_OUT45_ALPHA_DEG_S2, T1000_OUT45_STRAIGHT_MIN_MM},
  {"斜め出 135°", "T1000_OUT135", 135.0f, 135.0f / SQRT2_F, 45.0f / SQRT2_F, T1000_OUT135_SPEED_MM_S, T1000_OUT135_ALPHA_DEG_S2, TURN_MARGIN_MM},
  {"V90°", "T1000_V90", 90.0f, 90.0f / SQRT2_F, 90.0f / SQRT2_F, T1000_V90_SPEED_MM_S, T1000_V90_ALPHA_DEG_S2, TURN_MARGIN_MM},
};

/* スラロームだけで進む位置（始めた向きから見て、左へ・前へ） */
typedef struct {
  float left_mm;
  float forward_mm;
} arc_t;

/* trajectory.c でスラロームを 1 回走らせて、進んだ位置を足していく。
 * creep_per_mm_s2 は空回りの係数（先頭のコメントの「■ 空回りの式」。0 なら空回りしない） */
static arc_t simulate_arc_with_creep(float angle_deg, float speed_mm_s, float omega_deg_s, float alpha_deg_s2,
                                     float creep_per_mm_s2)
{
  arc_t arc = {0.0f, 0.0f};
  trajectory_reset();
  trajectory_start_slalom(angle_deg, speed_mm_s, omega_deg_s, alpha_deg_s2);
  int steps = 0;
  do {
    trajectory_update();
    float angle_rad = trajectory_get_angle_deg() * PI_F / 180.0f;
    float omega_rad_s = fabsf(trajectory_get_omega_deg_s()) * PI_F / 180.0f;
    float lateral_accel_mm_s2 = trajectory_get_speed_mm_s() * omega_rad_s;  /* 横向きの加速度 = 速さ × 角速度 */
    float ground_speed_mm_s = trajectory_get_speed_mm_s() * (1.0f - creep_per_mm_s2 * lateral_accel_mm_s2);
    arc.forward_mm = arc.forward_mm + ground_speed_mm_s * cosf(angle_rad) * CTRL_PERIOD_S;
    arc.left_mm = arc.left_mm + ground_speed_mm_s * sinf(angle_rad) * CTRL_PERIOD_S;
    steps = steps + 1;
  } while (!trajectory_is_done() && steps < MAX_STEPS);
  return arc;
}

/* 空回りしないとして、スラロームで進む位置を求める（--calc と、ふつうのテストで使う） */
static arc_t simulate_arc(float angle_deg, float speed_mm_s, float omega_deg_s, float alpha_deg_s2)
{
  return simulate_arc_with_creep(angle_deg, speed_mm_s, omega_deg_s, alpha_deg_s2, 0.0f);
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
    *pre_mm = turn->straight_min_mm;
    *post_mm = turn->straight_min_mm;
    return;
  }
  *post_mm = (turn->target_left_mm - arc.left_mm) / sin_value;
  *pre_mm = turn->target_forward_mm - arc.forward_mm - *post_mm * cosf(angle_rad);
}

/* 最高角速度 omega_deg_s で曲がったときの「決め手の値」。--calc は、これが 0 になる最高角速度を探す。
 *   大回り 180°：スラロームの左 − 目標の幅（90mm + BIG180_WIDTH_ADJUST_MM。角速度を上げると小さく回り、減る）
 *   ほか：前距離と後距離の短いほう − TURN_MARGIN_MM（角速度を上げると増える） */
static float omega_criterion(const turn_shape_t *turn, float omega_deg_s)
{
  arc_t arc = simulate_arc(turn->angle_deg, turn->speed_mm_s, omega_deg_s, turn->alpha_deg_s2);
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
  return shorter_mm - turn->straight_min_mm;
}

/* --calc の 1 組ぶん：組 turns の各ターンの最高角速度・前距離・後距離を計算して、config/turn_params.h に貼る行を出す */
static void calculate_set(const turn_shape_t turns[], const char *title)
{
  printf("/* ---- %s ---- */\n", title);
  printf("/* 前距離・後距離の短いほう %.1fmm で計算（斜め出 45° は OUT45_STRAIGHT_MIN_MM。test/test_turn.c --calc） */\n",
         TURN_MARGIN_MM);
  for (int i = 0; i < TURN_COUNT; i++) {
    const turn_shape_t *turn = &turns[i];
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
    arc_t arc = simulate_arc(turn->angle_deg, turn->speed_mm_s, omega_deg_s, turn->alpha_deg_s2);
    float pre_mm;
    float post_mm;
    solve_pre_post(turn, arc, &pre_mm, &post_mm);
    printf("/* %s（%.0fmm/s、角加速度 %.0fdeg/s²。横向きの加速度はいちばん大きいとき %.0fmm/s²） */\n", turn->name,
           turn->speed_mm_s, turn->alpha_deg_s2, turn->speed_mm_s * omega_deg_s * PI_F / 180.0f);
    printf("#define %s_OMEGA_DEG_S %.1ff\n", turn->prefix, omega_deg_s);
    printf("#define %s_PRE_MM %.1ff\n", turn->prefix, pre_mm);
    printf("#define %s_POST_MM %.1ff\n", turn->prefix, post_mm);
  }
}

/* --calc：組 1 と組 2 の両方を計算して、config/turn_params.h に貼る行を出す */
static int calculate(void)
{
  calculate_set(TURNS, "組 1（ゆるいターン 800mm/s、きついターン 600mm/s）");
  calculate_set(TURNS_1000, "組 2（7 つとも 1000mm/s）");
  return 0;
}

/* config/turn_params.h の値で、ターンが決めた位置・向きで終わるかを調べる（ずれ 0.5mm まで） */
static void check_turn(const turn_shape_t *turn, float omega_deg_s, float pre_mm, float post_mm)
{
  arc_t arc = simulate_arc(turn->angle_deg, turn->speed_mm_s, omega_deg_s, turn->alpha_deg_s2);
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
  arc_t normal = simulate_arc(turn->angle_deg, turn->speed_mm_s, omega_deg_s, turn->alpha_deg_s2);
  arc_t slow = simulate_arc(turn->angle_deg, turn->speed_mm_s * ratio, omega_deg_s * ratio,
                            turn->alpha_deg_s2 * ratio * ratio);
  printf("  %-16s 速さ %.1f 倍：左 %6.2f → %6.2f、前 %6.2f → %6.2f mm\n", turn->name, ratio,
         normal.left_mm, slow.left_mm, normal.forward_mm, slow.forward_mm);
  /* 0.5ms ごとに足していくので、速いほど 1 回で進む量が大きく、少しずれる（800mm/s では 1 回 0.4mm）。
   * 2026-11-03 にターンごとの速さ（最高 800mm/s）にしたので、±0.3mm から ±0.5mm にした */
  check(fabsf(normal.left_mm - slow.left_mm) < 0.5f && fabsf(normal.forward_mm - slow.forward_mm) < 0.5f,
        "遅く曲がっても、スラロームで進む位置が同じ（±0.5mm）");
}

/* ---------------------------------------------------------------------------
 * --slip：空回りの式（先頭のコメントの「■ 空回りの式」）
 * ------------------------------------------------------------------------- */

/* 実機で合わせたターン（大回り 180° のほか）の、config/turn_params.h の値 */
typedef struct {
  const turn_shape_t *shape;  /* ターンの形（TURNS の 1 つ） */
  float omega_deg_s;          /* 最高角速度（*_OMEGA_DEG_S） */
  float pre_mm;               /* 計算の前距離（*_PRE_MM） */
  float post_mm;              /* 計算の後距離（*_POST_MM） */
  float pre_adjust_mm;        /* 実機で足した前距離（*_PRE_ADJUST_MM） */
  float post_adjust_mm;       /* 実機で足した後距離（*_POST_ADJUST_MM） */
} tuned_turn_t;

#define TUNED_COUNT 6
static const tuned_turn_t TUNED[TUNED_COUNT] = {
  {&TURNS[0], BIG90_OMEGA_DEG_S, BIG90_PRE_MM, BIG90_POST_MM, BIG90_PRE_ADJUST_MM, BIG90_POST_ADJUST_MM},
  {&TURNS[2], IN45_OMEGA_DEG_S, IN45_PRE_MM, IN45_POST_MM, IN45_PRE_ADJUST_MM, IN45_POST_ADJUST_MM},
  {&TURNS[3], IN135_OMEGA_DEG_S, IN135_PRE_MM, IN135_POST_MM, IN135_PRE_ADJUST_MM, IN135_POST_ADJUST_MM},
  {&TURNS[4], OUT45_OMEGA_DEG_S, OUT45_PRE_MM, OUT45_POST_MM, OUT45_PRE_ADJUST_MM, OUT45_POST_ADJUST_MM},
  {&TURNS[5], OUT135_OMEGA_DEG_S, OUT135_PRE_MM, OUT135_POST_MM, OUT135_PRE_ADJUST_MM, OUT135_POST_ADJUST_MM},
  {&TURNS[6], V90_OMEGA_DEG_S, V90_PRE_MM, V90_POST_MM, V90_PRE_ADJUST_MM, V90_POST_ADJUST_MM},
};

/* 大回り 180° の実測（2026-11-04）：最高角速度 781.0deg/s・600mm/s で曲がったら、幅が 90mm ではなく 82mm だった */
#define BIG180_MEASURED_OMEGA_DEG_S 781.0f
#define BIG180_MEASURED_WIDTH_MM    82.0f

/* 空回りの係数 creep_per_mm_s2 で、速さ ratio 倍（最高角速度 ratio 倍、角加速度 ratio² 倍）で曲がるとき、
 * 決めた位置で終わるために、計算の前距離・後距離に足す分 [mm] を求める */
static void predicted_adjust(const tuned_turn_t *tuned, float creep_per_mm_s2, float ratio,
                             float *pre_adjust_mm, float *post_adjust_mm)
{
  const turn_shape_t *shape = tuned->shape;
  arc_t arc = simulate_arc_with_creep(shape->angle_deg, shape->speed_mm_s * ratio, tuned->omega_deg_s * ratio,
                                      shape->alpha_deg_s2 * ratio * ratio, creep_per_mm_s2);
  float pre_mm;
  float post_mm;
  solve_pre_post(shape, arc, &pre_mm, &post_mm);
  *pre_adjust_mm = pre_mm - tuned->pre_mm;
  *post_adjust_mm = post_mm - tuned->post_mm;
}

/* 空回りの係数 creep_per_mm_s2 で計算した値が、実機の値からどれだけずれるか（ずれ [mm] の 2 乗の合計）。
 * 6 つのターンの前距離・後距離に足した分と、大回り 180° の幅 */
static float slip_error(float creep_per_mm_s2)
{
  float total = 0.0f;
  for (int i = 0; i < TUNED_COUNT; i++) {
    float pre_adjust_mm;
    float post_adjust_mm;
    predicted_adjust(&TUNED[i], creep_per_mm_s2, 1.0f, &pre_adjust_mm, &post_adjust_mm);
    float pre_error_mm = pre_adjust_mm - TUNED[i].pre_adjust_mm;
    float post_error_mm = post_adjust_mm - TUNED[i].post_adjust_mm;
    total = total + pre_error_mm * pre_error_mm + post_error_mm * post_error_mm;
  }
  arc_t arc = simulate_arc_with_creep(180.0f, BIG180_SPEED_MM_S, BIG180_MEASURED_OMEGA_DEG_S, BIG180_ALPHA_DEG_S2,
                                      creep_per_mm_s2);
  float width_error_mm = arc.left_mm - BIG180_MEASURED_WIDTH_MM;
  total = total + width_error_mm * width_error_mm;
  return total;
}

/* 13 個の実測値（6 つのターンの前・後と、大回り 180° の幅）に対する、ずれの大きさの目安 [mm]（2 乗平均の平方根） */
static float slip_rms_mm(float creep_per_mm_s2)
{
  return sqrtf(slip_error(creep_per_mm_s2) / 13.0f);
}

/* 係数を 0 から少しずつ増やして、実機の値にいちばん合う係数を探す */
static float find_best_creep(void)
{
  float best_creep = 0.0f;
  float best_error = slip_error(0.0f);
  for (int i = 1; i <= 250; i++) {
    float creep = i * 0.0000002f;  /* 1000mm/s² あたり 0.02% ずつ */
    float error = slip_error(creep);
    if (error < best_error) {
      best_error = error;
      best_creep = creep;
    }
  }
  return best_creep;
}

/* 速さごとに、決めた位置で終わるために足す分を表にして出す（最短走行で、決めた速さより遅く曲がるとき用） */
static void print_adjust_by_speed(float creep_per_mm_s2)
{
  printf("[3] 速さごとの、足す分の計算 [mm]（前／後。決めた速さの 1.0・0.87・0.75・0.5 倍）\n");
  for (int i = 0; i < TUNED_COUNT; i++) {
    printf("  %-16s", TUNED[i].shape->name);
    float ratios[4] = {1.0f, 0.866f, 0.75f, 0.5f};
    for (int j = 0; j < 4; j++) {
      float pre_adjust_mm;
      float post_adjust_mm;
      predicted_adjust(&TUNED[i], creep_per_mm_s2, ratios[j], &pre_adjust_mm, &post_adjust_mm);
      printf("  %4.0fmm/s %4.1f／%4.1f", TUNED[i].shape->speed_mm_s * ratios[j], pre_adjust_mm, post_adjust_mm);
    }
    printf("\n");
  }
}

/* --slip：空回りの式で、実機で合わせた値を説明できるかを調べて、表を出す */
static int slip(void)
{
  float creep = find_best_creep();
  printf("[1] いちばん合う空回りの係数：%.2e（横向きの加速度 1000mm/s² あたり、速さが %.2f%% 遅くなる）\n",
         creep, creep * 1000.0f * 100.0f);
  printf("    ずれの目安（2 乗平均）：空回りなし %.1fmm → この係数 %.1fmm\n", slip_rms_mm(0.0f), slip_rms_mm(creep));

  printf("[2] 実機で足した分と、式で計算した足す分 [mm]\n");
  for (int i = 0; i < TUNED_COUNT; i++) {
    float pre_adjust_mm;
    float post_adjust_mm;
    predicted_adjust(&TUNED[i], creep, 1.0f, &pre_adjust_mm, &post_adjust_mm);
    printf("  %-16s %4.0fmm/s  前：実機 %4.1f・式 %4.1f   後：実機 %4.1f・式 %4.1f\n", TUNED[i].shape->name,
           TUNED[i].shape->speed_mm_s, TUNED[i].pre_adjust_mm, pre_adjust_mm, TUNED[i].post_adjust_mm, post_adjust_mm);
  }
  arc_t measured = simulate_arc_with_creep(180.0f, BIG180_SPEED_MM_S, BIG180_MEASURED_OMEGA_DEG_S, BIG180_ALPHA_DEG_S2,
                                           creep);
  arc_t now = simulate_arc_with_creep(180.0f, BIG180_SPEED_MM_S, BIG180_OMEGA_DEG_S, BIG180_ALPHA_DEG_S2, creep);
  printf("  大回り 180°      幅：最高 %.1fdeg/s のとき 実機 %.0f・式 %.1f、今の最高 %.1fdeg/s では式 %.1f（目標 90）\n",
         BIG180_MEASURED_OMEGA_DEG_S, BIG180_MEASURED_WIDTH_MM, measured.left_mm, BIG180_OMEGA_DEG_S, now.left_mm);

  print_adjust_by_speed(creep);
  return 0;
}

int main(int argc, char *argv[])
{
  if (argc >= 2 && strcmp(argv[1], "--calc") == 0) {
    return calculate();
  }
  if (argc >= 2 && strcmp(argv[1], "--slip") == 0) {
    return slip();
  }

  printf("[1] 最短走行のターン（config/turn_params.h の値。ターンごとの速さで）\n");
  check_turn(&TURNS[0], BIG90_OMEGA_DEG_S, BIG90_PRE_MM, BIG90_POST_MM);
  check_turn(&TURNS[1], BIG180_OMEGA_DEG_S, BIG180_PRE_MM, BIG180_POST_MM);
  check_turn(&TURNS[2], IN45_OMEGA_DEG_S, IN45_PRE_MM, IN45_POST_MM);
  check_turn(&TURNS[3], IN135_OMEGA_DEG_S, IN135_PRE_MM, IN135_POST_MM);
  check_turn(&TURNS[4], OUT45_OMEGA_DEG_S, OUT45_PRE_MM, OUT45_POST_MM);
  check_turn(&TURNS[5], OUT135_OMEGA_DEG_S, OUT135_PRE_MM, OUT135_POST_MM);
  check_turn(&TURNS[6], V90_OMEGA_DEG_S, V90_PRE_MM, V90_POST_MM);

  printf("[1b] 組 2（7 つとも 1000mm/s。config/turn_params.h の T1000_…）\n");
  check_turn(&TURNS_1000[0], T1000_BIG90_OMEGA_DEG_S, T1000_BIG90_PRE_MM, T1000_BIG90_POST_MM);
  check_turn(&TURNS_1000[1], T1000_BIG180_OMEGA_DEG_S, T1000_BIG180_PRE_MM, T1000_BIG180_POST_MM);
  check_turn(&TURNS_1000[2], T1000_IN45_OMEGA_DEG_S, T1000_IN45_PRE_MM, T1000_IN45_POST_MM);
  check_turn(&TURNS_1000[3], T1000_IN135_OMEGA_DEG_S, T1000_IN135_PRE_MM, T1000_IN135_POST_MM);
  check_turn(&TURNS_1000[4], T1000_OUT45_OMEGA_DEG_S, T1000_OUT45_PRE_MM, T1000_OUT45_POST_MM);
  check_turn(&TURNS_1000[5], T1000_OUT135_OMEGA_DEG_S, T1000_OUT135_PRE_MM, T1000_OUT135_POST_MM);
  check_turn(&TURNS_1000[6], T1000_V90_OMEGA_DEG_S, T1000_V90_PRE_MM, T1000_V90_POST_MM);

  printf("[2] 遅く入ったターン（速さ k 倍、最高角速度 k 倍、角加速度 k² 倍）\n");
  check_slow_turn(&TURNS[0], BIG90_OMEGA_DEG_S, 0.5f);
  check_slow_turn(&TURNS[2], IN45_OMEGA_DEG_S, 0.4f);
  check_slow_turn(&TURNS[3], IN135_OMEGA_DEG_S, 0.6f);
  check_slow_turn(&TURNS_1000[0], T1000_BIG90_OMEGA_DEG_S, 0.5f);

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗 %d 件\n", s_fail_count);
  return 1;
}
