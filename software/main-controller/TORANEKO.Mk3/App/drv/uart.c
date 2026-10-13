/*
 * uart.c
 * ログ送信（USART1 の TX = PA9 → STLINK-V3MODS の仮想COMポート → PC）
 *
 * ■ しくみ
 * UART は 1 文字送るのに約 87µs（115200bps）かかる。送り終わるまで待っていると、
 * 制御の処理が遅れてしまう。そこで次のようにして、呼び出した側を待たせないようにしている。
 *
 *   1. uart_write() は、送りたい文字を「送信待ちバッファ」に入れるだけで、すぐに戻る
 *   2. DMA（CPU の代わりにデータを運んでくれる機能）が、バッファの文字を UART に順に送る
 *   3. DMA が送り終わると HAL_UART_TxCpltCallback() が呼ばれるので、残りがあれば続きを送る
 *
 * ■ 送信待ちバッファ（リングバッファ）
 * 配列 s_buf を輪のように使う。書き込み位置 s_write_pos と送信位置 s_send_pos があり、
 * どちらも配列の最後まで進んだら 0 に戻る。
 *   - s_write_pos：次に文字を入れる場所
 *   - s_send_pos ：次に送る文字の場所（DMA で送信中の文字の先頭）
 *   - 2 つが同じ位置なら、送る文字はない
 * バッファが一杯のときに来た文字は捨てて、捨てた数を s_dropped_count に数える。
 */
#include "drv/uart.h"

#include <stdio.h>

#include "usart.h"

#define UART_BUF_SIZE 2048  /* 送信待ちバッファの大きさ [文字]。115200bps で約 0.18 秒分 */

static uint8_t s_buf[UART_BUF_SIZE];

/* 下の変数は、通常の処理と割り込みの両方から読み書きするので volatile を付ける
 * （volatile：コンパイラに「いつ書き換わるかわからないので、毎回メモリから読むこと」と伝える） */
static volatile uint32_t s_write_pos = 0;     /* 次に文字を入れる位置 */
static volatile uint32_t s_send_pos = 0;      /* 次に送る文字の位置 */
static volatile uint32_t s_sending_len = 0;   /* DMA で送信中の文字数。0 なら DMA は止まっている */
static volatile uint32_t s_dropped_count = 0; /* バッファが一杯で捨てた文字数（累計） */

/* DMA が止まっていて、送る文字が残っていれば、DMA の送信を始める。
 * 割り込みを禁止した状態で呼ぶこと */
static void start_sending(void)
{
  if (s_sending_len != 0) {
    return;  /* 送信中。終わったら HAL_UART_TxCpltCallback からまた呼ばれる */
  }
  if (s_send_pos == s_write_pos) {
    return;  /* 送る文字がない */
  }

  /* DMA は配列の連続した範囲しか送れないので、送る範囲が配列の最後をまたぐときは、
   * まず配列の最後までを送る。残り（配列の先頭側）は、送り終わったあとに送る */
  uint32_t len;
  if (s_write_pos > s_send_pos) {
    len = s_write_pos - s_send_pos;
  } else {
    len = UART_BUF_SIZE - s_send_pos;
  }

  s_sending_len = len;
  if (HAL_UART_Transmit_DMA(&huart1, &s_buf[s_send_pos], len) != HAL_OK) {
    s_sending_len = 0;  /* 送信を始められなかった。次の uart_write でやり直す */
  }
}

/* 起動時に呼ぶ */
void uart_init(void)
{
  /* printf は通常、文字を溜めてからまとめて出す。ここではすぐ uart_write に渡すように設定する
   * （溜める処理はこのファイルのバッファで行う） */
  setvbuf(stdout, NULL, _IONBF, 0);
}

/* 文字を送信待ちバッファに入れる。すぐに戻る（送信の完了は待たない）。割り込みの中から呼んでもよい */
void uart_write(const uint8_t *data, uint32_t len)
{
  /* 制御周期などの割り込みが途中で入って、同時にバッファを書き換えると壊れるので、
   * バッファを触っている間だけ割り込みを禁止する */
  __disable_irq();

  for (uint32_t i = 0; i < len; i++) {
    uint32_t next_pos = s_write_pos + 1;
    if (next_pos >= UART_BUF_SIZE) {
      next_pos = 0;
    }
    if (next_pos == s_send_pos) {
      /* バッファが一杯。残りの文字は捨てる */
      s_dropped_count = s_dropped_count + (len - i);
      break;
    }
    s_buf[s_write_pos] = data[i];
    s_write_pos = next_pos;
  }

  start_sending();

  __enable_irq();
}

/* バッファが一杯で捨てた文字数（累計）。0 でなければ、ログの量が多すぎる */
uint32_t uart_get_dropped_count(void)
{
  return s_dropped_count;
}

/* DMA の送信が終わったときに HAL から呼ばれる（割り込みの中） */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1) {
    return;
  }

  __disable_irq();

  /* 送り終わった分だけ送信位置を進める */
  s_send_pos = s_send_pos + s_sending_len;
  if (s_send_pos >= UART_BUF_SIZE) {
    s_send_pos = 0;
  }
  s_sending_len = 0;

  /* 残りがあれば続きを送る */
  start_sending();

  __enable_irq();
}

/* printf が文字を出すときに呼ぶ関数。
 * Core/Src/syscalls.c にも同じ名前の関数があるが、そちらは「ほかに定義があればそちらを使う」
 * 指定（weak）になっているので、こちらが使われる */
int _write(int file, char *ptr, int len)
{
  (void)file;  /* 使わない引数（警告を消すため） */
  uart_write((const uint8_t *)ptr, len);
  return len;
}
