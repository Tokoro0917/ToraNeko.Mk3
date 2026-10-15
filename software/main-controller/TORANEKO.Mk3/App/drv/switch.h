/*
 * switch.h
 * ユーザースイッチ SW1 の読み取り
 */
#ifndef DRV_SWITCH_H
#define DRV_SWITCH_H

#include <stdbool.h>

bool switch_is_pressed(void);

#endif /* DRV_SWITCH_H */
