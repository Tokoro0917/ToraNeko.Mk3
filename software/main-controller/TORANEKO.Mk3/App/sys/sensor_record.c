/*
 * sensor_record.c
 * センサ値の記録（モード5 No.2。MODES.md）
 *
 * 壁合わせの目標（ctrl/wall_ctrl.c の FRONT_LEFT_REFERENCE など）を決めるために、
 * 区画のまん中に置いたときの壁センサ 4 つの値を 1 秒測って、平均・いちばん小さい値・いちばん大きい値を覚える。
 *   1. スタート待ち（前センサに手をかざして離す）のあと、1 秒待ってから始まる（sys/mode.c と sys/ui.c）
 *   2. 1ms ごとに 1000 回、壁センサ 4 つ（ON−OFF）を読んで足していく
 *   3. 終わったら「うまくいった」を出す（測っているあいだは真ん中の 3 個、終わったら 7 個全部が 1 秒点く）
 * 走らせるときはケーブルを挿していないので、結果はモード7 No.1（ログ出力）で、走行ログの前に出す
 * （sensor_record_print()）。電源を切ると消える。
 */
#include "sys/sensor_record.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "ctrl/wall_ctrl.h"
#include "drv/led.h"
#include "drv/timebase.h"
#include "sys/ui.h"

#define RECORD_COUNT 1000      /* 測る回数（1ms ごとに 1000 回で 1 秒） */
#define RECORD_INTERVAL_MS 1   /* 測る間隔 [ms]（壁センサは 0.5ms ごとに新しい値になる） */

/* 1 つのセンサの記録 */
typedef struct {
  int32_t sum;  /* 足した値（4095 × 1000 でも int32_t に入る） */
  int32_t min;  /* いちばん小さい値 */
  int32_t max;  /* いちばん大きい値 */
} sensor_stat_t;

static bool s_recorded = false;     /* 一度でも測ったか */
static sensor_stat_t s_front_left;  /* 左前 */
static sensor_stat_t s_left;        /* 左 */
static sensor_stat_t s_right;       /* 右 */
static sensor_stat_t s_front_right; /* 右前 */

/* ここからの 3 つの関数は、記録（sensor_stat_t）の場所を受け取って、その中身を書きかえる・読む。
 * sensor_stat_t *stat は「記録の場所」で、&s_left のように & を付けて渡す。中身は stat->sum のように -> で使う。
 * 4 つのセンサで同じ関数を使い回すために、この書き方にした */

/* 記録を空にする（min は大きな値、max は小さな値から始めて、最初の値で置きかわるようにする） */
static void clear_stat(sensor_stat_t *stat)
{
  stat->sum = 0;
  stat->min = 100000;
  stat->max = -100000;
}

/* 記録に 1 つの値を足す */
static void add_value(sensor_stat_t *stat, int32_t value)
{
  stat->sum = stat->sum + value;
  if (value < stat->min) {
    stat->min = value;
  }
  if (value > stat->max) {
    stat->max = value;
  }
}

/* モード5 No.2：壁センサ 4 つを 1 秒測って、平均などを覚える */
void sensor_record_run(void)
{
  clear_stat(&s_front_left);
  clear_stat(&s_left);
  clear_stat(&s_right);
  clear_stat(&s_front_right);

  /* 測っているあいだは、真ん中の 3 個（LED3・4・5）だけを点ける。
   * 7 個全部だと、終わったときの「うまくいった」（7 個全部を 1 秒）と見分けにくいので */
  led_set(3, true);
  led_set(4, true);
  led_set(5, true);
  for (int i = 0; i < RECORD_COUNT; i++) {
    add_value(&s_front_left, wall_ctrl_get_front_left());
    add_value(&s_left, wall_ctrl_get_left());
    add_value(&s_right, wall_ctrl_get_right());
    add_value(&s_front_right, wall_ctrl_get_front_right());
    timebase_wait_ms(RECORD_INTERVAL_MS);
  }
  ui_all_off();

  s_recorded = true;
  sensor_record_print();  /* ケーブルを挿したまま測ったときのために、すぐにも出す */
  ui_signal_ok();
}

/* 1 つのセンサの記録を 1 行出す（センサの名前,平均,いちばん小さい値,いちばん大きい値） */
static void print_stat(const char *name, const sensor_stat_t *stat)
{
  float average = (float)stat->sum / RECORD_COUNT;  /* (float) で小数にしてから割る（整数どうしで割ると小数が切り捨てられる） */
  printf("%s,%.1f,%ld,%ld\r\n", name, average, stat->min, stat->max);
}

/* 最後に測ったセンサの値をシリアルに出す。モード7 No.1（ログ出力）でも呼ぶ。まだ測っていなければ何も出さない。
 * ビューア（tools/viewer.html）が 1 つのデータとして受け取って保存できるように、次の形で出す。
 *   #BEGIN SENSOR n=測った回数
 *   sensor,average,min,max
 *   FL,3243.9,3239,3249     （左前・左・右・右前の順）
 *   #END SENSOR */
void sensor_record_print(void)
{
  if (!s_recorded) {
    return;
  }
  printf("\r\n#BEGIN SENSOR n=%d\r\n", RECORD_COUNT);
  printf("sensor,average,min,max\r\n");
  print_stat("FL", &s_front_left);
  print_stat("L", &s_left);
  print_stat("R", &s_right);
  print_stat("FR", &s_front_right);
  printf("#END SENSOR\r\n");
}
