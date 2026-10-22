/*
 * ui.h
 * 機体の操作：LED の表示と合図、タイヤ・スイッチ・前センサの読み取り（MODES.md の 1〜3）
 *
 * ToraNeko にはブザーがないので、合図はすべて LED で出す。
 * モードを選ぶ流れ（どのモードで何をするか）は sys/mode.c にある。ここは、その部品。
 */
#ifndef SYS_UI_H
#define SYS_UI_H

#include <stdbool.h>
#include <stdint.h>

/* 入力を読む間隔 [ms]。待つ処理は、この間隔でタイヤ・スイッチ・前センサを読む */
#define UI_POLL_MS 10

/* タイヤを何カウント回すと 1 段（モードや No. が 1 つ）変わるか。
 * エンコーダはモーターの軸で 1 回転 16384 カウント、タイヤ 1 回転はモーター約 3.7 回転（約 61000 カウント）。
 * 8192（モーター半回転 = タイヤ約 1/7.5 回転）を仮に置いた（KOGUMA の 300 は KOGUMA の数え方の値で、
 * ToraNeko ではタイヤ 1.8° ほどになり細かすぎる）。実機で回しやすい量に決め直す */
#define UI_TIRE_COUNT_PER_STEP 8192

void ui_init(void);

/* LED の表示 */
void ui_show_mode_number(int mode, int number);
void ui_show_value(int value);
void ui_all_on(void);
void ui_all_off(void);

/* 合図 */
void ui_blink_all(int times);
void ui_signal_ok(void);
void ui_signal_error(void);
void ui_show_loading(void);

/* 入力 */
void ui_forget_tire_rotation(void);
int ui_read_left_steps(void);
int ui_read_right_steps(void);
bool ui_switch_pushed(void);
void ui_wait_ms(uint32_t wait_ms);

/* 待つ */
bool ui_wait_start(int mode, int number);
void ui_wait_failsafe_reset(void);
void ui_stop_for_low_battery(void);

#endif /* SYS_UI_H */
