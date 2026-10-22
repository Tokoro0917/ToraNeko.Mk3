/*
 * encoder_cal.c
 * エンコーダの歪みを測る（モード6 No.6。MODES.md）
 *
 * ■ 考え方
 * モーターを同じ電圧で回し続けると、ほぼ同じ速さで回る。同じ速さなら、本当の角度は時間に比例して増える。
 * そこで、読んだ角度が 16383 → 0 に戻る瞬間（1 回転の区切り）を 2 つ見つけ、そのあいだは
 * 「本当の角度は 0 から 16384 まで、時間に比例して増えた」と考える。
 * 読んだ角度 − 本当の角度 が、その場所のずれ。これを 1 回転を 64 に分けた場所ごとに足し合わせて平均する。
 * 何回転ぶんも平均するので、速さの小さなふらつきは打ち消し合う。
 * 最後に、64 個の平均を全部ならしたもの（全体のずれ）を引く。全体が同じだけずれているのは、
 * 回った量には関係ないため。
 * 同じやり方で、今の config/encoder_correction.h の表で直した角度のずれ（直したあとに残るずれ）も測る。
 * 表を入れたあとにもう一度測ると、表がどれだけ効いているかがわかる。
 *
 * ■ 流れ
 *   1. 左は -、右は + の電圧で回す（どちらも、読んだ角度が増える向き。drv/encoder.c の先頭のコメント）
 *   2. 0.5 秒待って、速さが落ち着いてから、1 秒ぶん（2000 回）の角度を記録する
 *      記録は、制御周期の割り込み（sys/scheduler.c）から encoder_cal_record() で行う
 *   3. 止めてから、記録を左右それぞれ計算して、config/encoder_correction.h に貼る表をシリアルに出す
 *
 * タイヤは必ず浮かせておく（その場で回る向きなので、床に置くと機体が回ってしまう）。
 */
#include "sys/encoder_cal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "config/encoder_correction.h"
#include "config/robot_params.h"
#include "drv/encoder.h"
#include "drv/motor.h"
#include "sys/ui.h"

#define CAL_VOLTAGE_V   1.0f  /* 回す電圧 [V]。1 回転の記録の数（シリアルに出る）が少なすぎたら下げる */
#define SPIN_UP_MS      500   /* 回し始めてから、記録を始めるまで待つ時間 [ms] */
#define SAMPLE_COUNT    2000  /* 記録する数（制御周期 2kHz で 1 秒ぶん） */
#define MAX_WRAP_COUNT  200   /* 1 回転の区切りを覚えておく数の上限（1 秒の記録なので、200 回転/秒まで。速すぎたら、そのあとは使わない） */
#define HALF_REV_COUNT  8192  /* 半回転のカウント数 */
#define MIN_REV_COUNT   10    /* 平均するのに、少なくともこれだけの回転数がほしい */

/* 割り込みの中で書き換える（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile bool s_recording = false;
static volatile int s_sample_count = 0;
static volatile uint16_t s_left_samples[SAMPLE_COUNT];
static volatile uint16_t s_right_samples[SAMPLE_COUNT];

/* 計算に使う（メインの処理だけで使う） */
static int s_wrap_index[MAX_WRAP_COUNT];    /* 16383 → 0 に戻った直後の記録の番号 */
static float s_wrap_time[MAX_WRAP_COUNT];   /* 角度がちょうど 16384（= 0）になった時刻 [記録 1 回ぶん = 0.5ms を 1 とする] */
static int s_wrap_count = 0;
static float s_bin_sum[ENCODER_CORRECTION_BIN_COUNT];  /* 場所ごとの、ずれの合計 [カウント] */
static int s_bin_samples[ENCODER_CORRECTION_BIN_COUNT];  /* 場所ごとの、足した数 */
static float s_bin_error[ENCODER_CORRECTION_BIN_COUNT];  /* 場所ごとの、ずれの平均（全体のずれを引いたもの） [カウント] */
static float s_bin_sum_after[ENCODER_CORRECTION_BIN_COUNT];    /* 今の表で直した角度の、場所ごとのずれの合計 [カウント] */
static float s_bin_error_after[ENCODER_CORRECTION_BIN_COUNT];  /* 今の表で直した角度の、場所ごとのずれの平均 [カウント] */

/* 制御周期ごとに（割り込みの中から）呼ぶ。記録中なら、左右の今の角度を 1 つずつ記録する */
void encoder_cal_record(void)
{
  if (!s_recording) {
    return;
  }
  s_left_samples[s_sample_count] = encoder_get_left_angle();
  s_right_samples[s_sample_count] = encoder_get_right_angle();
  s_sample_count = s_sample_count + 1;
  if (s_sample_count >= SAMPLE_COUNT) {
    s_recording = false;  /* 記録し終わった */
  }
}

/* 記録を始めて、終わるまで待つ */
static void record_samples(void)
{
  s_sample_count = 0;
  s_recording = true;  /* ここから、割り込みが記録を始める */
  while (s_recording) {
    ui_wait_ms(UI_POLL_MS);
  }
}

/* samples の中から、角度が 16383 → 0 に戻ったところ（1 回転の区切り）を全部見つける。
 * 区切りの時刻は、前後 2 つの記録のあいだを直線でつないで、角度がちょうど 16384 になる時刻を求める */
static void find_wraps(const volatile uint16_t samples[])
{
  s_wrap_count = 0;
  for (int i = 1; i < SAMPLE_COUNT; i++) {
    float before = samples[i - 1];
    float after = samples[i];
    /* 角度は増えていくので、半回転より大きく減っていたら、16383 → 0 に戻ったところ */
    if (after + HALF_REV_COUNT < before && s_wrap_count < MAX_WRAP_COUNT) {
      after = after + ENCODER_COUNT_PER_REV;  /* 戻らなかったとしたときの角度にする */
      float fraction = (ENCODER_COUNT_PER_REV - before) / (after - before);  /* 0〜1：前の記録から、どれだけ進んだところか */
      s_wrap_index[s_wrap_count] = i;
      s_wrap_time[s_wrap_count] = (i - 1) + fraction;
      s_wrap_count = s_wrap_count + 1;
    }
  }
}

/* 場所ごとの合計を 0 にする */
static void clear_bins(void)
{
  for (int i = 0; i < ENCODER_CORRECTION_BIN_COUNT; i++) {
    s_bin_sum[i] = 0.0f;
    s_bin_sum_after[i] = 0.0f;
    s_bin_samples[i] = 0;
  }
}

/* 読んだままの角度 raw_angle を、今の表で直した角度にする。is_left が true なら左の表、false なら右の表を使う */
static int32_t corrected_angle(bool is_left, uint16_t raw_angle)
{
  if (is_left) {
    return encoder_correct_left_angle(raw_angle);
  }
  return encoder_correct_right_angle(raw_angle);
}

/* 直した角度のずれ error_after を、-8192〜8192 の中にする。直した角度は 0 と 16383 の境目をまたぐことがあり
 * （例：読んだ角度 5 が、直すと 16380 になる）、そのときは 1 回転ぶんずれて見えるため */
static float wrap_error(float error_after)
{
  if (error_after > HALF_REV_COUNT) {
    error_after = error_after - ENCODER_COUNT_PER_REV;
  }
  if (error_after < -HALF_REV_COUNT) {
    error_after = error_after + ENCODER_COUNT_PER_REV;
  }
  return error_after;
}

/* 区切り wrap と wrap + 1 のあいだ（1 回転ぶん）の記録の、ずれを場所ごとに足す。
 * 読んだままの角度のずれと、今の表で直した角度のずれの両方を足す（場所は、どちらも読んだままの角度で決める） */
static void add_revolution(const volatile uint16_t samples[], bool is_left, int wrap)
{
  float start_time = s_wrap_time[wrap];
  float end_time = s_wrap_time[wrap + 1];
  for (int i = s_wrap_index[wrap]; i < s_wrap_index[wrap + 1]; i++) {
    /* 本当の角度：区切りで 0、次の区切りで 16384 になるように、時間に比例させる */
    float true_angle = ENCODER_COUNT_PER_REV * (i - start_time) / (end_time - start_time);
    float error = samples[i] - true_angle;
    int bin = samples[i] / ENCODER_CORRECTION_BIN_WIDTH;  /* 0〜63：どの場所か */
    float error_after = wrap_error(corrected_angle(is_left, samples[i]) - true_angle);
    s_bin_sum[bin] = s_bin_sum[bin] + error;
    s_bin_sum_after[bin] = s_bin_sum_after[bin] + error_after;
    s_bin_samples[bin] = s_bin_samples[bin] + 1;
  }
}

/* 合計 sum から場所ごとの平均を出して error に入れ、全体のずれ（64 個の平均）を引く */
static void average_one(const float sum[], float error[])
{
  float total = 0.0f;
  for (int i = 0; i < ENCODER_CORRECTION_BIN_COUNT; i++) {
    error[i] = sum[i] / s_bin_samples[i];
    total = total + error[i];
  }
  float mean = total / ENCODER_CORRECTION_BIN_COUNT;
  for (int i = 0; i < ENCODER_CORRECTION_BIN_COUNT; i++) {
    error[i] = error[i] - mean;
  }
}

/* 読んだままの角度と、直した角度の、場所ごとのずれの平均を出す。記録が 1 つもない場所があれば false */
static bool average_bins(void)
{
  for (int i = 0; i < ENCODER_CORRECTION_BIN_COUNT; i++) {
    if (s_bin_samples[i] == 0) {
      return false;
    }
  }
  average_one(s_bin_sum, s_bin_error);
  average_one(s_bin_sum_after, s_bin_error_after);
  return true;
}

/* 小数を、いちばん近い整数にする（0.5 は 0 から遠いほうへ）。(int) は小数点より下を切り捨てて整数にする書き方 */
static int round_to_int(float value)
{
  if (value >= 0.0f) {
    return (int)(value + 0.5f);
  }
  return (int)(value - 0.5f);
}

/* 場所ごとのずれ error の、いちばん大きいところと、いちばん小さいところの差 [カウント] */
static float error_peak_to_peak(const float error[])
{
  float min_error = error[0];
  float max_error = error[0];
  for (int i = 1; i < ENCODER_CORRECTION_BIN_COUNT; i++) {
    if (error[i] < min_error) {
      min_error = error[i];
    }
    if (error[i] > max_error) {
      max_error = error[i];
    }
  }
  return max_error - min_error;
}

/* config/encoder_correction.h に貼る表を出す（1 行に 8 個ずつ、8 行） */
static void print_table(const char *table_name)
{
  printf("static const int16_t %s[ENCODER_CORRECTION_BIN_COUNT] = {\r\n", table_name);
  for (int row = 0; row < 8; row++) {
    printf(" ");
    for (int column = 0; column < 8; column++) {
      int bin = row * 8 + column;
      printf(" %d,", round_to_int(s_bin_error[bin]));
    }
    printf("\r\n");
  }
  printf("};\r\n");
}

/* 1 つのエンコーダの記録から、ずれの表を作ってシリアルに出す。作れなかったら false */
static bool make_table(const volatile uint16_t samples[], bool is_left, const char *name, const char *table_name)
{
  find_wraps(samples);
  int rev_count = s_wrap_count - 1;  /* 区切りと区切りのあいだが 1 回転 */
  if (rev_count < MIN_REV_COUNT) {
    printf("Encoder %s: only %d revs. Not spinning? (need %d)\r\n", name, rev_count, MIN_REV_COUNT);
    return false;
  }

  clear_bins();
  for (int wrap = 0; wrap < rev_count; wrap++) {
    add_revolution(samples, is_left, wrap);
  }
  if (!average_bins()) {
    printf("Encoder %s: some bins have no samples\r\n", name);
    return false;
  }

  float samples_per_rev = (s_wrap_time[rev_count] - s_wrap_time[0]) / rev_count;
  float rev_per_s = CTRL_FREQ_HZ / samples_per_rev;
  float raw_p_p = error_peak_to_peak(s_bin_error);          /* 読んだままの角度のずれ */
  float after_p_p = error_peak_to_peak(s_bin_error_after);  /* 今の表で直したあとに残るずれ */
  printf("Encoder %s: %d revs, %.1f rev/s, %.1f samples/rev\r\n", name, rev_count, rev_per_s, samples_per_rev);
  printf("  raw error p-p = %.1f counts (%.2f deg), after correction p-p = %.1f counts (%.2f deg)\r\n",
         raw_p_p, raw_p_p * 360.0f / ENCODER_COUNT_PER_REV,
         after_p_p, after_p_p * 360.0f / ENCODER_COUNT_PER_REV);
  print_table(table_name);
  return true;
}

/* エンコーダの歪みを測って、config/encoder_correction.h に貼る表をシリアルに出す（タイヤを浮かせておく） */
void encoder_cal_run(void)
{
  printf("Encoder calibration: %.2f V\r\n", CAL_VOLTAGE_V);
  motor_set_voltage(-CAL_VOLTAGE_V, CAL_VOLTAGE_V);  /* 左右とも、読んだ角度が増える向き */
  ui_wait_ms(SPIN_UP_MS);
  record_samples();
  motor_stop();

  bool left_ok = make_table(s_left_samples, true, "L", "ENCODER_LEFT_CORRECTION");
  bool right_ok = make_table(s_right_samples, false, "R", "ENCODER_RIGHT_CORRECTION");
  printf("Errors: L=%lu R=%lu\r\n", encoder_get_left_error_count(), encoder_get_right_error_count());

  if (left_ok && right_ok) {
    ui_signal_ok();
  } else {
    ui_signal_error();
  }
}
