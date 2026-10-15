/*
 * mode.h
 * モードを選んで、走らせる（MODES.md）
 *
 * ■ 使い方
 *   起動時に mode_init() を 1 回呼び、そのあと mode_run_once() を繰り返し呼ぶ。
 *   mode_run_once() は、モードを選んでスタートし、走り終わったら戻ってくる。
 */
#ifndef SYS_MODE_H
#define SYS_MODE_H

#include <stdbool.h>

#define MODE_COUNT   8   /* モードは 0〜7 */
#define NUMBER_COUNT 16  /* No. は 0〜15 */

void mode_init(void);
void mode_run_once(void);
int mode_get_mode(void);
int mode_get_number(void);
bool mode_has_action(int mode, int number);

#endif /* SYS_MODE_H */
