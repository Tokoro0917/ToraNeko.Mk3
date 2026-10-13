#include "drv/uart.h"

#include <stdio.h>

#include "usart.h"

#define UART_TX_BUF_SIZE 2048U  /* 115200bps で約 0.18 秒分 */

static uint8_t s_buf[UART_TX_BUF_SIZE];
static volatile uint32_t s_head;     /* 次に書き込む位置 */
static volatile uint32_t s_tail;     /* DMA で送信中（または次に送る）先頭 */
static volatile uint32_t s_dma_len;  /* 送信中のバイト数。0 なら DMA は止まっている */
static volatile uint32_t s_dropped;

/* 割り込み禁止の中で呼ぶこと */
static void start_dma(void)
{
  if (s_dma_len != 0 || s_head == s_tail) {
    return;
  }
  /* リングの終端をまたぐときは終端までを先に送り、残りは完了割り込みで送る */
  uint32_t len = (s_head > s_tail) ? (s_head - s_tail) : (UART_TX_BUF_SIZE - s_tail);
  s_dma_len = len;
  if (HAL_UART_Transmit_DMA(&huart1, &s_buf[s_tail], (uint16_t)len) != HAL_OK) {
    s_dma_len = 0;
  }
}

void uart_init(void)
{
  s_head = 0;
  s_tail = 0;
  s_dma_len = 0;
  s_dropped = 0;
  setvbuf(stdout, NULL, _IONBF, 0);  /* newlib 側ではバッファせず、すぐ _write に渡す */
}

size_t uart_write(const uint8_t *data, size_t len)
{
  size_t written = 0;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (size_t i = 0; i < len; i++) {
    uint32_t next = (s_head + 1U) % UART_TX_BUF_SIZE;
    if (next == s_tail) {
      s_dropped += (uint32_t)(len - i);
      break;
    }
    s_buf[s_head] = data[i];
    s_head = next;
    written++;
  }
  start_dma();
  __set_PRIMASK(primask);
  return written;
}

uint32_t uart_get_dropped(void)
{
  return s_dropped;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1) {
    return;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_tail = (s_tail + s_dma_len) % UART_TX_BUF_SIZE;
  s_dma_len = 0;
  start_dma();
  __set_PRIMASK(primask);
}

/* printf の出力先（Core/Src/syscalls.c の weak な _write を上書き） */
int _write(int file, char *ptr, int len)
{
  (void)file;
  uart_write((const uint8_t *)ptr, (size_t)len);
  return len;  /* 溢れた分も「書けた」ことにして、newlib に再送させない */
}
