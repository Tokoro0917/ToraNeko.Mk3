/*
 * venue.c
 * 会場で合わせるもの（モード7 No.2〜4。MODES.md）。どちらも Flash に保存するので、電源を切っても残る。ビルドし直さなくてよい
 *
 * ■ 2 つのことを、別々に測る（2026-11-10。作者の案）
 *   - 感度の倍率（No.2。区画の境目で測る）：会場の迷路は、壁の塗装で光の跳ね返り方が変わり、壁センサの値が、どの距離でも
 *     ほぼ同じ割合で大きく・小さくなる。距離の表（ctrl/wall_dist.c）やしきい値は作り直さず、倍率で直す（KOGUMA の WallDist_Scale）：
 *       倍率 = 会場の区画の境目で測った値 ÷ 家の迷路で同じ置き方で測った値（ctrl/wall_ctrl.h の WALL_HOME_EDGE_*）
 *       使う値 = センサのそのままの値 ÷ 倍率（ctrl/wall_ctrl.c。家の迷路での値に直る）
 *     壁センサの値は、全部 ctrl/wall_ctrl.c の wall_ctrl_get_left() などを通って使われるので、ここで直せば、
 *     壁のあり・なし、前壁合わせ、壁切れ、斜めの補正、手の判定が、どれも同じしきい値のまま使える。
 *     境目で測るのは、走っているときに壁を見る位置で、前の壁まで 87mm あって、横のセンサに前の壁の光があまり入らないため
 *     （区画のまん中で前・左・右に壁があると、横のセンサの値が上がる。家の迷路で左 1460 → 1879、右 815 → 1137）
 *   - 壁制御の目標（No.3。区画の中心で測る）：横の壁に沿って走る補正は、左右のセンサの値が「区画の中心にいるときの値」に
 *     なるように寄せる。会場の区画の中心に置いて測った、左右のそのままの値を、そのまま目標にする（ctrl/wall_ctrl.c の
 *     wall_reference_left()。倍率で直した値と比べるので、目標も同じ倍率で割ってから比べる。そのままの値どうしで比べるのと同じ）。
 *     測っていないあいだは、家の迷路の値（ctrl/wall_ctrl.c の LEFT_WALL_REFERENCE・RIGHT_WALL_REFERENCE）を使う
 *   倍率と目標は、どちらを先に測ってもよい（目標はそのままの値で覚えているので、あとから倍率が変わっても、寄せる位置は変わらない）
 *
 * ■ モード7 No.2 倍率合わせ（venue_measure_scale_run）
 *   1. 前・左・右に壁がある区画の、入口の辺（区画の境目）に、タイヤの軸を合わせて置く。左右の壁のまん中に、まっすぐ置く
 *      （基板の前の端から前の壁まで 55mm。センタリング板の境目版を使う）
 *   2. スイッチで決定すると、スタート待ちをせずに始まる（手をかざすと、置いた位置がずれやすいため）
 *   3. 2 秒待ってから、そのままの値を 1ms ごとに 1 秒（1000 回）測って平均する。測っているあいだは真ん中の 3 個が点く
 *   4. 4 つの倍率がどれも SCALE_MIN〜SCALE_MAX なら、使い始めて Flash に保存し、「うまくいった」を出す。
 *      範囲の外のものがあれば（壁がない、置き方がおかしい）、保存せずに「できなかった」を出す
 *
 * ■ モード7 No.3 壁制御の目標（venue_measure_reference_run）
 *   1. 左右に壁がある区画の中心に、まっすぐ置く（センタリング板のまっすぐ版を使う。前の区画は、前の壁がない（あいている）ようにする。
 *      壁に沿って走るのは、前の壁が近くにない直進のときがほとんどなので）
 *   2. スイッチで決定すると、スタート待ちをせずに始まる（No.2 と同じ）
 *   3. 2 秒待ってから、左右のそのままの値を 1 秒測って平均する
 *   4. 左右とも壁があると読めたら（倍率で直した値が、壁のあり・なしのしきい値より大きい）、使い始めて Flash に保存し、
 *      「うまくいった」を出す。読めなければ、保存せずに「できなかった」を出す
 *
 * ■ モード7 No.4 元に戻す（venue_reset_run）
 * 倍率を全部 1 に、壁制御の目標を家の値に戻して保存する。決定するとすぐ。家で表を作り直すとき・しきい値や目標を測るとき
 * （モード5 No.2）は、先に戻しておく（表やしきい値は、家の迷路で、倍率 1 の値で決めるため）
 *
 * 結果は、すぐにシリアルに出す（ケーブルを挿していれば見える）。あとからモード7 No.1 でも出す（venue_print）
 */
#include "sys/venue.h"

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

/* 最後に測った値（モード7 No.1 で出すため。電源を切ると消える） */
static bool s_measured = false;            /* No.2 で測ったか */
static float s_measured_front_left = 0.0f;
static float s_measured_left = 0.0f;
static float s_measured_right = 0.0f;
static float s_measured_front_right = 0.0f;

/* 電源を入れたときに呼ぶ。Flash に保存した倍率と壁制御の目標を読んで、使い始める
 * （保存したことがなければ、倍率は 1、目標は家の値） */
void venue_init(void)
{
  wall_ctrl_set_sensor_scale(flash_load_sensor_scale());
  wall_ctrl_set_wall_reference(flash_load_wall_reference());
}

/* 倍率が使える範囲（SCALE_MIN〜SCALE_MAX）にあるか */
static bool scale_is_usable(float scale)
{
  if (scale < SCALE_MIN || scale > SCALE_MAX) {
    return false;
  }
  return true;
}

/* 4 つの壁センサの、そのままの値を 1 秒測って平均し、s_measured_… に入れる。
 * 測っているあいだは、真ん中の 3 個（LED3・4・5）だけを点ける（センサ値の記録と同じ） */
static void measure_values(void)
{
  int32_t sum_front_left = 0;  /* 4095 × 1000 でも int32_t に入る */
  int32_t sum_left = 0;
  int32_t sum_right = 0;
  int32_t sum_front_right = 0;
  led_set(3, true);
  led_set(4, true);
  led_set(5, true);
  for (int i = 0; i < MEASURE_COUNT; i++) {
    sum_front_left = sum_front_left + wall_ctrl_get_front_left_raw();
    sum_left = sum_left + wall_ctrl_get_left_raw();
    sum_right = sum_right + wall_ctrl_get_right_raw();
    sum_front_right = sum_front_right + wall_ctrl_get_front_right_raw();
    timebase_wait_ms(MEASURE_INTERVAL_MS);
  }
  ui_all_off();
  /* (float) で小数にしてから割る（整数どうしで割ると小数が切り捨てられる） */
  s_measured_front_left = (float)sum_front_left / MEASURE_COUNT;
  s_measured_left = (float)sum_left / MEASURE_COUNT;
  s_measured_right = (float)sum_right / MEASURE_COUNT;
  s_measured_front_right = (float)sum_front_right / MEASURE_COUNT;
}

/* 測った値から、倍率（測った値 ÷ 家の迷路で同じ置き方で測った値）を出す */
static sensor_scale_t scale_from_measured(void)
{
  sensor_scale_t scale;
  scale.front_left = s_measured_front_left / WALL_HOME_EDGE_FRONT_LEFT;
  scale.left = s_measured_left / WALL_HOME_EDGE_LEFT;
  scale.right = s_measured_right / WALL_HOME_EDGE_RIGHT;
  scale.front_right = s_measured_front_right / WALL_HOME_EDGE_FRONT_RIGHT;
  return scale;
}

/* モード7 No.2：区画の境目で測って倍率を決め、使える値なら使い始めて保存する（流れは先頭のコメント） */
void venue_measure_scale_run(void)
{
  ui_wait_ms(MEASURE_WAIT_MS);
  measure_values();
  s_measured = true;

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
  venue_print();                   /* ケーブルを挿したまま測ったときのために、すぐにも出す */
  ui_signal_ok();
}

/* モード7 No.3：区画の中心で左右を測り、左右とも壁があると読めたら、壁制御の目標にして保存する（流れは先頭のコメント） */
void venue_measure_reference_run(void)
{
  ui_wait_ms(MEASURE_WAIT_MS);
  measure_values();

  /* 左右とも壁があるか（倍率で直した値で、壁のあり・なしのしきい値と比べる。測り終わったときの値で見る） */
  if (!wall_ctrl_has_wall_left() || !wall_ctrl_has_wall_right()) {
    printf("\r\nWALL_REFERENCE NG: L %.1f R %.1f（左右の壁が読めない。保存しない）\r\n", s_measured_left, s_measured_right);
    ui_signal_error();
    return;
  }

  wall_reference_t reference;
  reference.is_set = true;
  reference.left = s_measured_left;
  reference.right = s_measured_right;
  wall_ctrl_set_wall_reference(reference);
  flash_save_wall_reference(reference);
  venue_print();
  ui_signal_ok();
}

/* モード7 No.4：倍率を全部 1（直さない）に、壁制御の目標を家の値（測っていない）に戻して、保存する */
void venue_reset_run(void)
{
  sensor_scale_t scale;
  scale.front_left = 1.0f;
  scale.left = 1.0f;
  scale.right = 1.0f;
  scale.front_right = 1.0f;
  wall_reference_t reference;
  reference.is_set = false;
  reference.left = 0.0f;
  reference.right = 0.0f;
  wall_ctrl_set_sensor_scale(scale);
  wall_ctrl_set_wall_reference(reference);
  flash_save_sensor_scale_and_reference(scale, reference);
  venue_print();
  ui_signal_ok();
}

/* 倍率の 1 つのセンサの行を出す（名前,今の倍率,最後に測った値,家の迷路での値）。まだ測っていなければ、測った値は 0 */
static void print_scale_row(const char *name, float scale, float measured, float home_value)
{
  printf("%s,%.4f,%.1f,%.1f\r\n", name, scale, measured, home_value);
}

/* 今の倍率と壁制御の目標をシリアルに出す。モード7 No.1（ログ出力）でも呼ぶ。
 * ビューア（tools/viewer.html）が受け取れるように、次の形で出す。
 *   #BEGIN CENTER measured=この電源で No.2 を測ったか（1 か 0）
 *   sensor,scale,measured,home
 *   FL,1.0213,924.3,905.0       （左前・左・右・右前の順。home は家の迷路で同じ置き方で測った値）
 *   #END CENTER
 *   #BEGIN WALLREF set=会場で測った目標を使っているか（1 か 0）
 *   sensor,raw,home
 *   L,1350.0,1460.0             （left・right の順。raw は会場で測ったそのままの値〈測っていなければ 0〉、home は家の値）
 *   #END WALLREF */
void venue_print(void)
{
  sensor_scale_t scale = wall_ctrl_get_sensor_scale();
  int measured_flag = 0;
  if (s_measured) {
    measured_flag = 1;
  }
  printf("\r\n#BEGIN CENTER measured=%d\r\n", measured_flag);
  printf("sensor,scale,measured,home\r\n");
  print_scale_row("FL", scale.front_left, s_measured_front_left, WALL_HOME_EDGE_FRONT_LEFT);
  print_scale_row("L", scale.left, s_measured_left, WALL_HOME_EDGE_LEFT);
  print_scale_row("R", scale.right, s_measured_right, WALL_HOME_EDGE_RIGHT);
  print_scale_row("FR", scale.front_right, s_measured_front_right, WALL_HOME_EDGE_FRONT_RIGHT);
  printf("#END CENTER\r\n");

  wall_reference_t reference = wall_ctrl_get_wall_reference();
  int set_flag = 0;
  if (reference.is_set) {
    set_flag = 1;
  }
  printf("\r\n#BEGIN WALLREF set=%d\r\n", set_flag);
  printf("sensor,raw,home\r\n");
  printf("L,%.1f,%d\r\n", reference.left, WALL_HOME_CENTER_LEFT);
  printf("R,%.1f,%d\r\n", reference.right, WALL_HOME_CENTER_RIGHT);
  printf("#END WALLREF\r\n");
}
