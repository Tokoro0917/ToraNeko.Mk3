#ifndef DRV_TIMEBASE_H
#define DRV_TIMEBASE_H

#include <stdint.h>

uint32_t timebase_get_ms(void);        /* 起動からの経過時間 [ms]（SysTick） */
uint32_t timebase_get_sysclk_hz(void); /* システムクロック [Hz]。クロック設定の確認用 */

#endif /* DRV_TIMEBASE_H */
