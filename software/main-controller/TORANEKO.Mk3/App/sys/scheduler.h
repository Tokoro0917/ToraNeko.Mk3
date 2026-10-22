/*
 * scheduler.h
 * 制御周期（2kHz）の割り込みで、制御の処理を呼ぶ
 */
#ifndef SYS_SCHEDULER_H
#define SYS_SCHEDULER_H

#include <stdint.h>

/* 制御周期の割り込みの記録（scheduler_take_stats で取り出す） */
typedef struct {
  uint32_t tick_count;     /* 前に取り出してから、割り込みが来た回数 */
  uint32_t busy_max_us;    /* 1 回の処理にかかった時間の最大 [µs] */
  uint32_t period_min_us;  /* 割り込みの間隔の最小 [µs]（2kHz なら 500 のはず） */
  uint32_t period_max_us;  /* 割り込みの間隔の最大 [µs] */
} scheduler_stats_t;

void scheduler_start(void);
void scheduler_take_stats(scheduler_stats_t *stats);

#endif /* SYS_SCHEDULER_H */
