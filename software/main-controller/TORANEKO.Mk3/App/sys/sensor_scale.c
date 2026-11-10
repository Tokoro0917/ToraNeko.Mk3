/*
 * sensor_scale.c
 * 会場ごとの感度合わせ（モード5 No.7・8。MODES.md。KOGUMA のモード3 No.5 と同じ考え方）
 *
 * 会場の迷路は、壁の塗装で光の跳ね返り方が変わり、壁センサの値が、どの距離でもほぼ同じ割合で大きく・小さくなる。
 * そこで、距離の表（ctrl/wall_dist.c）やしきい値は作り直さず、区画のまん中で 1 回測った「倍率」で直す：
 *   倍率 = 会場の区画のまん中で測った値 ÷ 家の迷路で、同じ置き方（前・左・右に壁）で測った値（下の HOME_*）
 *   使う値 = センサのそのままの値 ÷ 倍率（ctrl/wall_ctrl.c。家の迷路での値に直る）
 * 比べる 2 つは、必ず同じ置き方で測った値にする。横のセンサは、前に壁があると、前の壁で返った光も入って値が上がる
 * （家の迷路で、左 1460 → 1879、右 815 → 1137。ctrl/wall_ctrl.c の LEFT_WALL_REFERENCE のコメント）。
 * 2026-11-10 に最初に作ったときは、前に壁がある置き方で測った値を、前に壁がない置き方で作った距離の表の値と比べていたので、
 * 横の倍率が大きく出すぎていた（会場で 1.11・1.14。本当は 0.88・0.95 くらい）。そのため、横の値が小さく直されて、
 * 横の壁を「ない」と読みやすくなっていた
 * 壁センサの値は、全部 ctrl/wall_ctrl.c の wall_ctrl_get_left() などを通って使われるので、ここで直せば、
 * 壁のあり・なし、横の壁に沿う補正、前壁合わせ、壁切れ、斜めの補正、手の判定が、どれもそのまま使える。
 * 倍率は Flash に保存するので（sys/flash.c。ゴールと同じ場所）、電源を切っても残る。ビルドし直さなくてよい。
 *
 * ■ モード5 No.7 感度を測る（sensor_scale_measure_run）
 *   1. 区画のまん中に、前・左・右に壁がある向きで置く（スタート区画なら、後ろの壁のほうを向けて置く）
 *   2. スイッチで決定すると、スタート待ちをせずに始まる（前に壁があると、前センサは壁を見ていて、手をかざしたのがわからないため）
 *   3. 2 秒待ってから（スイッチを押した手を離して、機体が落ち着くまで。このあいだに置き直してもよい）、
 *      そのままの値を 1ms ごとに 1 秒（1000 回）測って平均する。測っているあいだは真ん中の 3 個が点く
 *   4. 4 つのセンサの倍率を出して、どれも SCALE_MIN〜SCALE_MAX なら、使い始めて Flash に保存し、「うまくいった」を出す。
 *      範囲の外のものがあれば（壁がない、置き方がおかしい）、保存せずに「できなかった」を出す
 *   結果は、すぐにシリアルに出す（ケーブルを挿していれば見える）。あとからモード7 No.1 でも出す（sensor_scale_print）
 *
 * ■ モード5 No.8 感度を元に戻す（sensor_scale_reset_run）
 * 倍率を全部 1 に戻して保存する。表を作り直すとき・しきい値を測るとき（モード5 No.2）は、先に戻しておく
 * （表やしきい値は、倍率 1 の値で決めるため）
 */
#include "sys/sensor_scale.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "ctrl/wall_ctrl.h"
#include "drv/led.h"
#include "drv/timebase.h"
#include "sys/flash.h"
#include "sys/ui.h"

#define MEASURE_COUNT       1000   /* 測る回数（1ms ごとに 1000 回で 1 秒） */
#define MEASURE_INTERVAL_MS 1      /* 測る間隔 [ms] */
#define MEASURE_WAIT_MS     2000   /* 決定してから測り始めるまで待つ時間 [ms]（スイッチから手を離して、機体が落ち着くまで） */
#define SCALE_MIN           0.5f   /* これより小さい倍率は、壁がない・遠いとみなして使わない */
#define SCALE_MAX           2.0f   /* これより大きい倍率は、壁が近すぎる（置き方がおかしい）とみなして使わない */

/* 家の迷路で、区画のまん中に、前・左・右に壁がある向きで置いて測った値（倍率の基準。2026-11-02、センサを付け直したあと。
 * モード5 No.2 の 1 秒の平均）。前は ctrl/wall_ctrl.c の FRONT_LEFT_REFERENCE・FRONT_RIGHT_REFERENCE と同じ値、
 * 横は LEFT_WALL_REFERENCE のコメントにある「前の壁があるとき」の値。家でセンサを付け直したり、距離の表を作り直したりしたら、
 * No.8 で倍率を 1 に戻してから、同じ置き方でモード5 No.2 で測り直して、ここも直す */
#define HOME_FRONT_LEFT  3334.0f
#define HOME_LEFT        1879.0f
#define HOME_RIGHT       1137.0f
#define HOME_FRONT_RIGHT 3392.0f

/* 最後に測った値（モード7 No.1 で出すため。電源を切ると消える） */
static bool s_measured = false;
static float s_measured_front_left = 0.0f;
static float s_measured_left = 0.0f;
static float s_measured_right = 0.0f;
static float s_measured_front_right = 0.0f;

/* 電源を入れたときに呼ぶ。Flash に保存した倍率を読んで、使い始める（保存したことがなければ 1） */
void sensor_scale_init(void)
{
  wall_ctrl_set_sensor_scale(flash_load_sensor_scale());
}

/* 倍率が使える範囲（SCALE_MIN〜SCALE_MAX）にあるか */
static bool scale_is_usable(float scale)
{
  if (scale < SCALE_MIN || scale > SCALE_MAX) {
    return false;
  }
  return true;
}

/* 4 つの壁センサの、そのままの値を 1 秒測って平均し、s_measured_… に入れる */
static void measure_center_values(void)
{
  int32_t sum_front_left = 0;  /* 4095 × 1000 でも int32_t に入る */
  int32_t sum_left = 0;
  int32_t sum_right = 0;
  int32_t sum_front_right = 0;
  for (int i = 0; i < MEASURE_COUNT; i++) {
    sum_front_left = sum_front_left + wall_ctrl_get_front_left_raw();
    sum_left = sum_left + wall_ctrl_get_left_raw();
    sum_right = sum_right + wall_ctrl_get_right_raw();
    sum_front_right = sum_front_right + wall_ctrl_get_front_right_raw();
    timebase_wait_ms(MEASURE_INTERVAL_MS);
  }
  /* (float) で小数にしてから割る（整数どうしで割ると小数が切り捨てられる） */
  s_measured_front_left = (float)sum_front_left / MEASURE_COUNT;
  s_measured_left = (float)sum_left / MEASURE_COUNT;
  s_measured_right = (float)sum_right / MEASURE_COUNT;
  s_measured_front_right = (float)sum_front_right / MEASURE_COUNT;
  s_measured = true;
}

/* 測った値から、倍率（測った値 ÷ 家の迷路で同じ置き方で測った値）を出す */
static sensor_scale_t scale_from_measured(void)
{
  sensor_scale_t scale;
  scale.front_left = s_measured_front_left / HOME_FRONT_LEFT;
  scale.left = s_measured_left / HOME_LEFT;
  scale.right = s_measured_right / HOME_RIGHT;
  scale.front_right = s_measured_front_right / HOME_FRONT_RIGHT;
  return scale;
}

/* モード5 No.7：区画のまん中で測って倍率を決め、使える値なら使い始めて保存する（流れは先頭のコメント） */
void sensor_scale_measure_run(void)
{
  ui_wait_ms(MEASURE_WAIT_MS);

  /* 測っているあいだは、真ん中の 3 個（LED3・4・5）だけを点ける（センサ値の記録と同じ） */
  led_set(3, true);
  led_set(4, true);
  led_set(5, true);
  measure_center_values();
  ui_all_off();

  sensor_scale_t scale = scale_from_measured();
  bool usable = scale_is_usable(scale.front_left) && scale_is_usable(scale.left)
                && scale_is_usable(scale.right) && scale_is_usable(scale.front_right);
  if (!usable) {
    printf("\r\nSENSOR_SCALE NG: FL %.2f L %.2f R %.2f FR %.2f（%.1f〜%.1f の外がある。保存しない）\r\n",
           scale.front_left, scale.left, scale.right, scale.front_right, SCALE_MIN, SCALE_MAX);
    ui_signal_error();
    return;
  }

  wall_ctrl_set_sensor_scale(scale);
  flash_save_sensor_scale(scale);  /* 保存しているあいだは外側の 4 個が点く（sys/flash.c） */
  sensor_scale_print();            /* ケーブルを挿したまま測ったときのために、すぐにも出す */
  ui_signal_ok();
}

/* モード5 No.8：倍率を全部 1（直さない）に戻して、保存する */
void sensor_scale_reset_run(void)
{
  sensor_scale_t scale;
  scale.front_left = 1.0f;
  scale.left = 1.0f;
  scale.right = 1.0f;
  scale.front_right = 1.0f;
  wall_ctrl_set_sensor_scale(scale);
  flash_save_sensor_scale(scale);
  sensor_scale_print();
  ui_signal_ok();
}

/* 1 つのセンサの行を出す（名前,今の倍率,最後に測った値,家の迷路での値）。まだ測っていなければ、測った値は 0 */
static void print_row(const char *name, float scale, float measured, float home_value)
{
  printf("%s,%.4f,%.1f,%.1f\r\n", name, scale, measured, home_value);
}

/* 今の倍率をシリアルに出す。モード7 No.1（ログ出力）でも呼ぶ。
 * ビューア（tools/viewer.html）が「感度合わせ」（CENTER）として受け取れるように、次の形で出す。
 *   #BEGIN CENTER measured=この電源で測ったか（1 か 0）
 *   sensor,scale,measured,home
 *   FL,1.0213,3405.0,3334.0     （左前・左・右・右前の順。home は家の迷路で同じ置き方で測った値）
 *   #END CENTER */
void sensor_scale_print(void)
{
  sensor_scale_t scale = wall_ctrl_get_sensor_scale();
  int measured_flag = 0;
  if (s_measured) {
    measured_flag = 1;
  }
  printf("\r\n#BEGIN CENTER measured=%d\r\n", measured_flag);
  printf("sensor,scale,measured,home\r\n");
  print_row("FL", scale.front_left, s_measured_front_left, HOME_FRONT_LEFT);
  print_row("L", scale.left, s_measured_left, HOME_LEFT);
  print_row("R", scale.right, s_measured_right, HOME_RIGHT);
  print_row("FR", scale.front_right, s_measured_front_right, HOME_FRONT_RIGHT);
  printf("#END CENTER\r\n");
}
