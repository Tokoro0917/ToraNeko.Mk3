/*
 * flash_memory.h
 * Flash の保存場所を消す・書く・読む（HAL の Flash の関数を使うのはここだけ）
 *
 * 何をどの順番で書くか（迷路やゴールの形）は sys/flash.c が決める。ここは、決められた場所に
 * 4 バイトの数（語）を 1 つずつ書く・読むだけ。
 *
 * ■ 保存場所
 * STM32F411 の Flash（512KB）は「セクタ」という区切りに分かれていて、消すときはセクタ単位でしか消せない。
 * プログラムは Flash の前から入るので、いちばん後ろの 2 つのセクタ（128KB ずつ）を保存に使う。
 *   FLASH_AREA_GOAL  セクタ6（0x08040000〜）：ゴールの位置
 *   FLASH_AREA_MAZE  セクタ7（0x08060000〜）：迷路
 * ゴールと迷路を別のセクタに分けておくと、片方を保存するときに、もう片方を消さずに済む。
 * そのかわり、プログラムは前の 256KB（セクタ0〜5）に収める（2026-10-15 で約 42KB）。超えていないかは CI（firmware-build の「サイズ」）で調べる。
 *
 * ■ Flash の性質
 *   - 消すと、どの語も 0xFFFFFFFF になる
 *   - 書けるのは、消したあとの語に 1 回だけ。書き直すときは、セクタごと消してから書く
 *   - 消しているあいだ（128KB で約 1〜2 秒）は、CPU が止まる（割り込みも動かない）
 */
#ifndef DRV_FLASH_MEMORY_H
#define DRV_FLASH_MEMORY_H

#include <stdbool.h>
#include <stdint.h>

/* 保存場所 */
typedef enum {
  FLASH_AREA_GOAL = 0,  /* セクタ6：ゴールの位置 */
  FLASH_AREA_MAZE = 1,  /* セクタ7：迷路 */
} flash_area_t;

/* 1 つの保存場所に入る語（4 バイト）の数（128KB ÷ 4 バイト） */
#define FLASH_AREA_WORD_COUNT 32768

/* 消したあとの語の値 */
#define FLASH_ERASED_WORD 0xFFFFFFFF

bool flash_memory_erase(flash_area_t area);
bool flash_memory_write_word(flash_area_t area, int index, uint32_t value);
uint32_t flash_memory_read_word(flash_area_t area, int index);

#endif /* DRV_FLASH_MEMORY_H */
