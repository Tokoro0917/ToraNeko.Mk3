/*
 * log.c
 * 走行ログ：制御周期ごとに機体の状態を記録し、あとで PC へ出す（KOGUMA_PORTING.md の B1〜B3）
 *
 * ■ 持ち方（KOGUMA の docs/log_design.md と同じ）
 * RAM を減らすため、走った距離のほかは、単位を決めた 2 バイトの整数（int16）にして持つ
 * （例：角速度は 0.1 deg/s 単位。12.34 deg/s なら 123）。出すときに元の単位に戻す。1 行 36 バイト × 1000 行。
 *
 * ■ 割り込みとの受け渡し
 * log_record() は制御周期の割り込みの中から、log_start()・log_stop()・log_print() は普通の処理から呼ぶ。
 * 両方から読み書きする変数には volatile を付ける（毎回メモリから読み書きさせ、順番も入れ替えさせないため）。
 *   - log_start() は、ほかの変数を全部書いてから、最後に s_recording を true にする
 *   - log_print() は、log_stop() のあと（記録していないとき）に呼ぶ
 *
 * ■ 出力の形（KOGUMA のビューア tools/viewer.html で読める）
 *   #BEGIN LOG dt=間隔[s] n=行数 div=間隔[ms] full=いっぱいになったか(1/0)
 *   列名の行
 *   値の行（CSV）
 *   #END LOG
 */
#include "sys/log.h"

#include <stdint.h>
#include <stdio.h>

#include "config/robot_params.h"
#include "drv/uart.h"

/* 値を int16 にするときに掛ける数（KOGUMA と同じ単位） */
#define SCALE_SPEED 1.0f    /* 速度：1 mm/s */
#define SCALE_OMEGA 10.0f   /* 角速度：0.1 deg/s、角度：0.1 deg */
#define SCALE_DIST  10.0f   /* 壁までの距離：0.1 mm */
#define SCALE_ERROR 10.0f   /* 横壁制御の誤差：0.1 */
#define SCALE_PID   100.0f  /* 横壁制御の P 項・D 項：0.01 */
#define SCALE_VOLT  10.0f   /* 電圧：0.1 V */
#define SCALE_BATT  100.0f  /* 電池：0.01 V */

/* 1 行の長さの上限 [文字]。出す前に、UART の送信待ちバッファにこれだけの空きができるのを待つ */
#define LINE_LENGTH_MAX 200

/* 記録した 1 行 */
typedef struct {
  float distance_mm;       /* 記録を始めてから走った距離 [mm] */
  int16_t speed;           /* 速度 [1 mm/s] */
  int16_t speed_target;    /* 目標の速度 [1 mm/s] */
  int16_t omega;           /* 角速度 [0.1 deg/s] */
  int16_t omega_target;    /* 目標の角速度 [0.1 deg/s] */
  int16_t angle;           /* 角度 [0.1 deg] */
  int16_t sensor_left;     /* 左の横センサの値 */
  int16_t sensor_right;    /* 右の横センサの値 */
  int16_t wall_dist_left;  /* 左の壁までの距離 [0.1 mm] */
  int16_t wall_dist_right; /* 右の壁までの距離 [0.1 mm] */
  int16_t wall_error;      /* 横壁制御の誤差 [0.1] */
  int16_t wall_p;          /* 横壁制御の P 項 [0.01] */
  int16_t wall_d;          /* 横壁制御の D 項 [0.01] */
  int16_t voltage_left;    /* 左のモーターの電圧 [0.1 V] */
  int16_t voltage_right;   /* 右のモーターの電圧 [0.1 V] */
  int16_t battery;         /* 電池の電圧 [0.01 V] */
  int8_t wall_used;        /* 使った壁（1 左、2 右、3 両方、0 なし） */
  int8_t phase;            /* 何の動作中か（log_phase_t） */
} log_row_t;

static log_row_t s_rows[LOG_ROW_MAX];

/* 割り込み（log_record）と普通の処理の両方から使う変数 */
static volatile bool s_recording = false;         /* 記録中か */
static volatile bool s_full = false;              /* いっぱいになって、記録をやめたか */
static volatile int s_count = 0;                  /* 記録した行数 */
static volatile int s_interval_ms = 1;            /* 何 ms ごとに 1 行残すか */
static volatile int s_periods_per_row = 2;        /* 何回の制御周期ごとに 1 行残すか */
static volatile int s_period_count = 0;           /* 前に残してから、何回呼ばれたか */
static volatile float s_start_distance_mm = 0.0f; /* 記録を始めたときの距離 [mm] */

/* 記録を始める。interval_ms [ms] ごとに 1 行残す（制御周期の何回ぶんかに直す。2kHz なら 1ms = 2 回）。
 * 前のログは消える */
void log_start(int interval_ms)
{
  s_recording = false;

  int periods = interval_ms * CTRL_FREQ_HZ / 1000;
  if (periods < 1) {
    periods = 1;
  }
  s_interval_ms = interval_ms;
  s_periods_per_row = periods;
  s_period_count = periods - 1;  /* 最初に呼ばれたときに 1 行目を残す */
  s_count = 0;
  s_full = false;

  s_recording = true;  /* 最後に記録中にする */
}

/* value × scale を四捨五入して、2 バイトの整数にする。入りきらない値は、入る値の端にそろえる */
static int16_t to_int16(float value, float scale)
{
  float scaled = value * scale;
  if (scaled > 32767.0f) {
    scaled = 32767.0f;
  }
  if (scaled < -32767.0f) {
    scaled = -32767.0f;
  }
  /* (int16_t) は、小数を切り捨てて整数にする。0.5 を足して（負なら引いて）から切り捨てると四捨五入になる */
  if (scaled >= 0.0f) {
    return (int16_t)(scaled + 0.5f);
  }
  return (int16_t)(scaled - 0.5f);
}

/* 今の値を記録する。制御周期の割り込みから毎回呼ぶ。間隔の回数に 1 回だけ、1 行残す。
 * いっぱいなら記録をやめるだけ（走行は止めない） */
void log_record(log_values_t values)
{
  if (!s_recording) {
    return;
  }

  /* 間隔の回数に 1 回だけ残す */
  s_period_count = s_period_count + 1;
  if (s_period_count < s_periods_per_row) {
    return;
  }
  s_period_count = 0;

  if (s_count >= LOG_ROW_MAX) {
    s_full = true;
    s_recording = false;
    return;
  }

  int i = s_count;
  if (i == 0) {
    s_start_distance_mm = values.distance_mm;
  }
  s_rows[i].distance_mm = values.distance_mm - s_start_distance_mm;
  s_rows[i].speed = to_int16(values.speed_mm_s, SCALE_SPEED);
  s_rows[i].speed_target = to_int16(values.speed_target_mm_s, SCALE_SPEED);
  s_rows[i].omega = to_int16(values.omega_deg_s, SCALE_OMEGA);
  s_rows[i].omega_target = to_int16(values.omega_target_deg_s, SCALE_OMEGA);
  s_rows[i].angle = to_int16(values.angle_deg, SCALE_OMEGA);
  s_rows[i].sensor_left = to_int16((float)values.sensor_left, 1.0f);
  s_rows[i].sensor_right = to_int16((float)values.sensor_right, 1.0f);
  s_rows[i].wall_dist_left = to_int16(values.wall_dist_left_mm, SCALE_DIST);
  s_rows[i].wall_dist_right = to_int16(values.wall_dist_right_mm, SCALE_DIST);
  s_rows[i].wall_error = to_int16(values.wall_error, SCALE_ERROR);
  s_rows[i].wall_p = to_int16(values.wall_p, SCALE_PID);
  s_rows[i].wall_d = to_int16(values.wall_d, SCALE_PID);
  s_rows[i].voltage_left = to_int16(values.voltage_left_v, SCALE_VOLT);
  s_rows[i].voltage_right = to_int16(values.voltage_right_v, SCALE_VOLT);
  s_rows[i].battery = to_int16(values.battery_v, SCALE_BATT);
  s_rows[i].wall_used = (int8_t)values.wall_used;
  s_rows[i].phase = (int8_t)values.phase;
  s_count = i + 1;
}

/* 記録をやめる（記録した行は残る） */
void log_stop(void)
{
  s_recording = false;
}

/* 記録した行があるか */
bool log_has_data(void)
{
  if (s_count > 0) {
    return true;
  }
  return false;
}

/* いっぱいになって、途中で記録をやめたか */
bool log_is_full(void)
{
  return s_full;
}

/* 記録した行数 */
int log_get_count(void)
{
  return s_count;
}

/* 記録した i 行目を、元の単位に戻して 1 行に出す。time_s はその行の時刻 [s] */
static void print_row(int i, float time_s)
{
  float wall_p = s_rows[i].wall_p / SCALE_PID;
  float wall_d = s_rows[i].wall_d / SCALE_PID;
  printf("%.3f,%.1f,%d,%d,%.1f,%.1f,%.1f,%d,%d,%.1f,%.1f,%d,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%d\r\n",
         time_s,
         s_rows[i].distance_mm,
         s_rows[i].speed,
         s_rows[i].speed_target,
         s_rows[i].omega / SCALE_OMEGA,
         s_rows[i].omega_target / SCALE_OMEGA,
         s_rows[i].angle / SCALE_OMEGA,
         s_rows[i].sensor_left,
         s_rows[i].sensor_right,
         s_rows[i].wall_dist_left / SCALE_DIST,
         s_rows[i].wall_dist_right / SCALE_DIST,
         s_rows[i].wall_used,
         s_rows[i].wall_error / SCALE_ERROR,
         wall_p,
         wall_d,
         wall_p + wall_d,
         s_rows[i].voltage_left / SCALE_VOLT,
         s_rows[i].voltage_right / SCALE_VOLT,
         s_rows[i].battery / SCALE_BATT,
         s_rows[i].phase);
}

/* 記録したログを、KOGUMA のビューアで読める形で出す（記録していないときに呼ぶ）。
 * 1 行ずつ、UART の送信待ちバッファに空きができるのを待ってから出す（一杯になって文字を捨てないように） */
void log_print(void)
{
  int count = s_count;
  int full = 0;
  if (s_full) {
    full = 1;
  }
  float interval_s = s_interval_ms / 1000.0f;

  uart_wait_free(LINE_LENGTH_MAX);
  printf("\r\n#BEGIN LOG dt=%.3f n=%d div=%d full=%d\r\n", interval_s, count, s_interval_ms, full);
  uart_wait_free(LINE_LENGTH_MAX);
  printf("t,x,v,v_target,w,w_target,angle,sensor_L,sensor_R,dist_L,dist_R,wall,"
         "wall_err,wall_p,wall_d,wall_out,volt_L,volt_R,batt,phase\r\n");

  for (int i = 0; i < count; i++) {
    uart_wait_free(LINE_LENGTH_MAX);
    print_row(i, i * interval_s);
  }

  uart_wait_free(LINE_LENGTH_MAX);
  printf("#END LOG\r\n");
}
