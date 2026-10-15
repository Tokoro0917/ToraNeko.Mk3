/*
 * uart.h
 * ログ送信（USART1 の TX = PA9 → STLINK-V3MODS の仮想COMポート → PC）
 *
 * printf で書いた文字もここから送られる。
 */
#ifndef DRV_UART_H
#define DRV_UART_H

#include <stdint.h>

void uart_init(void);
void uart_write(const uint8_t *data, uint32_t len);
void uart_wait_free(uint32_t count);
uint32_t uart_get_dropped_count(void);

#endif /* DRV_UART_H */
