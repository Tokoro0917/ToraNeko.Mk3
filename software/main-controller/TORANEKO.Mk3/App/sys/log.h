/*
 * log.h
 * 走行ログ：制御周期ごとに機体の状態を記録し、あとで PC へ出す（KOGUMA_PORTING.md の B1〜B3）
 *
 * ■ 使い方
 *   1. 走り始める前に log_start(間隔 [ms]) を呼ぶ（ターンのテストは 1、直進のテストは 2、最短走行は 8）
 *   2. 制御周期の割り込みから、毎回 log_record(今の値) を呼ぶ。間隔ごとに 1 行だけ残す
 *   3. 走り終わったら log_stop() を呼ぶ
 *   4. 出力（モード7 No.1）で log_print() を呼ぶと、KOGUMA のビューアで読める形で出す
 *
 * LOG_ROW_MAX 行（最短走行なら 8 秒）でいっぱいになったら、記録をやめるだけで、走行はそのまま続ける。
 * 出力の 1 行目に full=1 と書いて、途中で記録をやめたことを知らせる。
 * ログは RAM だけに残す（電源を切ると消える。KOGUMA と同じ）。
 */
#ifndef SYS_LOG_H
#define SYS_LOG_H

#include <stdbool.h>

#define LOG_ROW_MAX 1000  /* 記録できる行数（RAM が足りないので KOGUMA の 2000 行の半分） */

/* 何の動作中か（ビューアが区間を色分けする。KOGUMA の LOG_PH_* と同じ番号） */
typedef enum {
  LOG_PHASE_OTHER     = 0,   /* そのほか */
  LOG_PHASE_STRAIGHT  = 1,   /* 直線 */
  LOG_PHASE_DIAG      = 2,   /* 斜めの直線 */
  LOG_PHASE_PRE       = 3,   /* 前距離（ターンの前の直進） */
  LOG_PHASE_TURN      = 4,   /* スラロームのターン */
  LOG_PHASE_POST      = 5,   /* 後距離（ターンの後の直進） */
  LOG_PHASE_PRE_DIAG  = 6,   /* 斜めからの前距離 */
  LOG_PHASE_POST_DIAG = 7,   /* 斜めへの後距離 */
  LOG_PHASE_PIVOT     = 8,   /* その場旋回 */
  LOG_PHASE_STOP      = 9,   /* 止まる */
  LOG_PHASE_ALIGN     = 10,  /* 壁合わせ */
} log_phase_t;

/* 記録する値（1 行ぶん）。記録する側（制御の処理）が、今の値を入れて log_record() に渡す */
typedef struct {
  float distance_mm;         /* 走った距離 [mm]（合計でよい。ログには記録を始めてからの距離を残す） */
  float speed_mm_s;          /* 速度 [mm/s] */
  float speed_target_mm_s;   /* 目標の速度 [mm/s] */
  float omega_deg_s;         /* 角速度 [deg/s] */
  float omega_target_deg_s;  /* 目標の角速度 [deg/s] */
  float angle_deg;           /* 角度 [deg]（動作ごとに 0 から） */
  int sensor_left;           /* 左の横センサの値（LPF だけ。1 回だけの落ち込みも見える） */
  int sensor_right;          /* 右の横センサの値 */
  float wall_dist_left_mm;   /* 左の壁までの距離 [mm]（横壁制御が使う値） */
  float wall_dist_right_mm;  /* 右の壁までの距離 [mm] */
  int wall_used;             /* 横壁制御に使った壁（1 左、2 右、3 両方、0 なし） */
  float wall_error;          /* 横壁制御の誤差 */
  float wall_p;              /* 横壁制御の P 項 */
  float wall_d;              /* 横壁制御の D 項 */
  float voltage_left_v;      /* 左のモーターにかけた電圧 [V] */
  float voltage_right_v;     /* 右のモーターにかけた電圧 [V] */
  float battery_v;           /* 電池の電圧 [V] */
  log_phase_t phase;         /* 何の動作中か */
} log_values_t;

void log_start(int interval_ms);
void log_record(log_values_t values);
void log_stop(void);
bool log_has_data(void);
bool log_is_full(void);
int log_get_count(void);
void log_print(void);

#endif /* SYS_LOG_H */
