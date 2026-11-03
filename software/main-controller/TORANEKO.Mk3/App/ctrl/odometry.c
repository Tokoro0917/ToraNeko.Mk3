/*
 * odometry.c
 * エンコーダから、タイヤの速さと走った距離を求める
 *
 * ■ カウントから mm へ
 * エンコーダはタイヤの軸に付いていて、タイヤ 1 回転で 16384 カウント。そのあいだにタイヤは 1 周ぶん
 * （直径 × 円周率 ≒ 44mm）進む。なので、1 カウントは 44 ÷ 16384 ≒ 0.0027mm。
 * （最初はモーターの軸に付いていると思って、さらにギア比 3.73 で割っていたので、3.7 倍走りすぎていた。2026-10-22）
 *
 * ■ 速さ（4 回前との差で求める。2026-11-03）
 * 制御周期（0.5ms）ごとに、4 回前（2ms 前）のカウントから増えた分を mm に直し、2ms で割る。
 *
 * 最初は「1 回前（0.5ms 前）との差 ÷ 0.5ms」で求めていたが、2026-11-03 の探索のログで、
 * 止まっているのに速さが ±50mm/s（ばらつき 24mm/s）ゆれていた。エンコーダの読んだ角度が、
 * 毎回 4〜5 カウントほどぶれるため（磁石が弱く、AGC=255・MAGL=1）。0.5ms で割ると、1 カウントのぶれが
 * 5.4mm/s になってしまう。走っていても、本当の速さ（位置の増え方）は一定なのに、測った速さがゆれ、
 * それに合わせて電圧も ±0.15V ほどゆれていた。
 * 4 回前との差にすると、ぶれは同じ 4〜5 カウントのまま、割る時間が 4 倍になるので、速さのゆれは約 1/4 になる。
 * そのかわり、速さが約 1ms 遅れて伝わる（2ms のあいだの平均の速さなので、そのまん中の 1ms 前の速さになる）。
 *
 * ■ 4 回前のカウントの覚え方
 * 配列 s_left_history（右も同じ）に、最近 4 回分のカウントを入れておく。s_history_index は、
 * その中でいちばん古い（4 回前の）カウントが入っている場所。毎回、
 *   1. s_history_index の場所から 4 回前のカウントを読み、速さを求める
 *   2. 同じ場所に今のカウントを書く（いちばん古いものを、いちばん新しいもので上書きする）
 *   3. s_history_index を 1 つ進める。配列の終わり（4）まで来たら 0 に戻す
 * こうすると、s_history_index は 0→1→2→3→0→… と回り、いつも 4 回前のカウントを指している。
 *
 * ■ 走った距離
 * 走った距離は、これまでと同じく、1 回前からの差を足していく（遅れはない）。
 *
 * ■ 割り込み
 * odometry_update() は制御周期の割り込み（sys/scheduler.c）で、encoder_update() のあとに呼ぶ。
 */
#include "ctrl/odometry.h"

#include <stdint.h>

#include "config/robot_params.h"
#include "drv/encoder.h"

#define PI_VALUE 3.14159265f

/* 1 カウントで進む距離 [mm] */
#define MM_PER_COUNT (TIRE_DIAMETER_MM * PI_VALUE / ENCODER_COUNT_PER_REV)

/* 速さを、何回前のカウントとの差で求めるか（4 回 = 2ms） */
#define SPEED_WINDOW_COUNT 4
/* その時間 [s]（4 × 0.5ms = 0.002 秒） */
#define SPEED_WINDOW_S (SPEED_WINDOW_COUNT * CTRL_PERIOD_S)

/* 割り込みの中で書き換える（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile int32_t s_last_left_count = 0;   /* 1 回前の左のカウント（走った距離に使う） */
static volatile int32_t s_last_right_count = 0;
static volatile int32_t s_left_history[SPEED_WINDOW_COUNT];   /* 最近 4 回分の左のカウント（速さに使う） */
static volatile int32_t s_right_history[SPEED_WINDOW_COUNT];
static volatile int s_history_index = 0;  /* s_*_history の中で、いちばん古い（4 回前の）カウントがある場所 */
static volatile float s_left_speed_mm_s = 0.0f;
static volatile float s_right_speed_mm_s = 0.0f;
static volatile float s_distance_mm = 0.0f;

/* 起動時に 1 回だけ呼ぶ（encoder_init() のあと）。今のカウントを、前の回の値にしておく
 * （最近 4 回分も全部今のカウントにしておくので、最初の速さは 0 になる） */
void odometry_init(void)
{
  int32_t left_count = encoder_get_left_total();
  int32_t right_count = encoder_get_right_total();

  s_last_left_count = left_count;
  s_last_right_count = right_count;
  for (int i = 0; i < SPEED_WINDOW_COUNT; i++) {
    s_left_history[i] = left_count;
    s_right_history[i] = right_count;
  }
  s_history_index = 0;
  s_left_speed_mm_s = 0.0f;
  s_right_speed_mm_s = 0.0f;
  s_distance_mm = 0.0f;
}

/* 1 回前からの差で、走った距離を足す */
static void update_distance(int32_t left_count, int32_t right_count)
{
  float left_moved_mm = (left_count - s_last_left_count) * MM_PER_COUNT;
  float right_moved_mm = (right_count - s_last_right_count) * MM_PER_COUNT;
  s_last_left_count = left_count;
  s_last_right_count = right_count;

  s_distance_mm = s_distance_mm + (left_moved_mm + right_moved_mm) / 2.0f;
}

/* 4 回前からの差で、左右の速さを求め、今のカウントを覚える（先頭のコメントの「4 回前のカウントの覚え方」） */
static void update_speed(int32_t left_count, int32_t right_count)
{
  /* 1. 4 回前のカウントとの差で速さを求める */
  int32_t left_oldest_count = s_left_history[s_history_index];
  int32_t right_oldest_count = s_right_history[s_history_index];
  s_left_speed_mm_s = (left_count - left_oldest_count) * MM_PER_COUNT / SPEED_WINDOW_S;
  s_right_speed_mm_s = (right_count - right_oldest_count) * MM_PER_COUNT / SPEED_WINDOW_S;

  /* 2. いちばん古いカウントを、今のカウントで上書きする */
  s_left_history[s_history_index] = left_count;
  s_right_history[s_history_index] = right_count;

  /* 3. 次にいちばん古くなる場所へ進める。終わりまで来たら 0 に戻す */
  s_history_index = s_history_index + 1;
  if (s_history_index >= SPEED_WINDOW_COUNT) {
    s_history_index = 0;
  }
}

/* 制御周期ごとに呼ぶ。エンコーダのカウントから、走った距離と左右の速さを求める */
void odometry_update(void)
{
  int32_t left_count = encoder_get_left_total();
  int32_t right_count = encoder_get_right_total();

  update_distance(left_count, right_count);
  update_speed(left_count, right_count);
}

/* 左のタイヤの速さ [mm/s] */
float odometry_get_left_speed_mm_s(void)
{
  return s_left_speed_mm_s;
}

/* 右のタイヤの速さ [mm/s] */
float odometry_get_right_speed_mm_s(void)
{
  return s_right_speed_mm_s;
}

/* 機体の速さ [mm/s]（左右の平均） */
float odometry_get_speed_mm_s(void)
{
  return (s_left_speed_mm_s + s_right_speed_mm_s) / 2.0f;
}

/* 電源を入れてから走った距離 [mm] */
float odometry_get_distance_mm(void)
{
  return s_distance_mm;
}
