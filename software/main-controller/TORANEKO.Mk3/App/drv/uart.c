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
 *
 * ■ 割り込み禁止
 * バッファの位置（s_write_pos など）は、通常の処理と割り込みの両方から書き換える。途中で割り込みが入って
 * 同時に書き換えると壊れるので、書き換えている間だけ割り込みを禁止する。
 *   - 禁止する前の状態を覚えておき、終わったら元に戻す（すでに禁止されている場所から呼ばれても、
 *     勝手に許可しないため。例：割り込みを禁止した中で printf を呼んだとき）
 *   - 長い文字列でも、禁止するのは 16 文字ずつにする（制御周期や壁センサの割り込みを長く待たせないため）
 *
 * ■ printf は割り込みの中で使わない
 * printf（newlib）は割り込みの中から呼ぶことを考えて作られていない。割り込みの中でログを出すときは
 * uart_write() を直接使う。
 */
#include "drv/uart.h"

#include <stdio.h>

#include "usart.h"

#define UART_BUF_SIZE 2048    /* 送信待ちバッファの大きさ [文字]。115200bps で約 0.18 秒分 */
#define UART_CHUNK_SIZE 16    /* 割り込みを禁止して一度に入れる文字数 */

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
   * （溜める処理はこのファイルのバッファで行う）。
   * こうすると printf は 1 文字ずつ _write を呼ぶ（1 回の printf で 20〜30 回）が、1 回あたりは
   * バッファに 1 文字入れるだけなので軽い。行ごとに溜める設定（_IOLBF）にすると、改行が来るまで
   * 送られない文字が残ったり、printf 用の共通のバッファを割り込みと取り合ったりするので使わない */
  setvbuf(stdout, NULL, _IONBF, 0);
}

/* 文字を最大 count 文字、送信待ちバッファに入れる。割り込みを禁止した状態で呼ぶこと。
 * バッファが一杯で入らなかった文字は捨てて数える */
static void put_chars(const uint8_t *data, uint32_t count)
{
  for (uint32_t i = 0; i < count; i++) {
    uint32_t next_pos = s_write_pos + 1;
    if (next_pos >= UART_BUF_SIZE) {
      next_pos = 0;
    }
    if (next_pos == s_send_pos) {
      /* バッファが一杯。残りの文字は捨てる */
      s_dropped_count = s_dropped_count + (count - i);
      return;
    }
    s_buf[s_write_pos] = data[i];
    s_write_pos = next_pos;
  }
}

/* 文字を送信待ちバッファに入れる。すぐに戻る（送信の完了は待たない）。割り込みの中から呼んでもよい */
void uart_write(const uint8_t *data, uint32_t len)
{
  uint32_t done = 0;  /* 入れ終わった文字数 */
  while (done < len) {
    uint32_t count = len - done;
    if (count > UART_CHUNK_SIZE) {
      count = UART_CHUNK_SIZE;
    }

    /* 割り込みを禁止する前の状態（PRIMASK）を覚えておき、終わったら元に戻す */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    put_chars(&data[done], count);
    start_sending();
    __set_PRIMASK(primask);

    done = done + count;
  }
}

/* 送信待ちバッファの空き [文字]。
 * s_send_pos は DMA の送信完了の割り込みで進む（空きが増える）。読んでいるあいだに進んでも、
 * 空きを少なめに数えるだけなので困らない（割り込みは禁止しない）。
 * ただし、割り込みの中から uart_write() で書くと、そのぶん空きが減るので、uart_wait_free() で待った空きより
 * 少なくなることがある（今は割り込みの中から書いていない。書くようにしたら、捨てた文字の数で確かめる） */
static uint32_t get_free_count(void)
{
  uint32_t write_pos = s_write_pos;
  uint32_t send_pos = s_send_pos;
  uint32_t used;
  if (write_pos >= send_pos) {
    used = write_pos - send_pos;
  } else {
    used = UART_BUF_SIZE - send_pos + write_pos;
  }
  /* バッファは「書き込み位置の次が送信位置」になったら一杯（1 文字ぶんは使わない） */
  return UART_BUF_SIZE - 1 - used;
}

/* 送信待ちバッファに count 文字ぶんの空きができるまで待つ。
 * 出力（迷路・走行ログ。全部で 100KB ほど）のように、たくさんの文字を続けて送るときに、
 * バッファが一杯になって文字を捨てないように、1 行ずつ書く前に呼ぶ。
 * 割り込みの中から呼んではいけない（送信完了の割り込みが来ないと、空きが増えないため） */
void uart_wait_free(uint32_t count)
{
  while (get_free_count() < count) {
    /* DMA が送り終わって、空きが増えるのを待つ */
  }
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

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  /* 送り終わった分だけ送信位置を進める */
  s_send_pos = s_send_pos + s_sending_len;
  if (s_send_pos >= UART_BUF_SIZE) {
    s_send_pos = 0;
  }
  s_sending_len = 0;

  /* 残りがあれば続きを送る */
  start_sending();

  __set_PRIMASK(primask);
}

/* UART か DMA でエラーが起きたときに HAL から呼ばれる（割り込みの中）。
 * HAL は送信を止めて「送信していない」状態に戻すが、送信完了（HAL_UART_TxCpltCallback）は呼ばれない。
 * そのままだと s_sending_len が 0 に戻らず二度と送れなくなるので、送信中だった分は捨てて（捨てた数に数える）、
 * 残りの送信を続ける。同じ分を送り直さないのは、エラーが続いたときに割り込みが止まらなくなるのを防ぐため */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1) {
    return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  s_dropped_count = s_dropped_count + s_sending_len;
  s_send_pos = s_send_pos + s_sending_len;
  if (s_send_pos >= UART_BUF_SIZE) {
    s_send_pos = 0;
  }
  s_sending_len = 0;
  start_sending();

  __set_PRIMASK(primask);
}

/* printf が文字を出すときに呼ぶ関数。
 * Core/Src/syscalls.c にも同じ名前の関数があるが、そちらは「ほかに定義があればそちらを使う」
 * 指定（weak）になっているので、こちらが使われる */
int _write(int file, char *ptr, int len)
{
  (void)file;  /* 使わない引数（警告を消すため） */
  /* printf からは char の並びで来るが、uart_write は uint8_t（0〜255 の数）の並びで受け取るので、型を読み替えて渡す。
   * どちらも 1 文字 1 バイトなので、中身は変わらない */
  uart_write((const uint8_t *)ptr, len);
  return len;
}
