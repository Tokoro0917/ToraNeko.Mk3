#ifndef DRV_UART_H
#define DRV_UART_H

#include <stddef.h>
#include <stdint.h>

/* USART1（PA9 TX → STLINK-V3MODS の仮想COMポート）へのログ送信。
 * リングバッファに積んで DMA で送るので、呼び出し側は待たされない（割り込みの中からも呼べる）。
 * printf の出力もここに流れる（_write を上書きしている）。 */

void uart_init(void);
size_t uart_write(const uint8_t *data, size_t len);  /* 積めたバイト数を返す。バッファが一杯なら残りは捨てる */
uint32_t uart_get_dropped(void);                      /* バッファ溢れで捨てたバイト数の累計 */

#endif /* DRV_UART_H */
