/*
 * flash_memory.c
 * Flash の保存場所を消す・書く・読む（保存場所と Flash の性質は flash_memory.h）
 *
 * ■ HAL で Flash を消す・書くときの流れ
 *   1. HAL_FLASH_Unlock()：Flash への書き込みを許す（ふだんは、間違えて書かないように鍵がかかっている）
 *   2. 前の操作で残ったエラーの印を消す（残っていると、次の操作が失敗したことになるため）
 *   3. HAL_FLASHEx_Erase()（セクタを消す）または HAL_FLASH_Program()（1 語を書く）
 *   4. HAL_FLASH_Lock()：鍵をかけ直す
 */
#include "drv/flash_memory.h"

#include "stm32f4xx_hal.h"

#define GOAL_AREA_ADDRESS 0x08040000  /* セクタ6 の先頭の番地 */
#define MAZE_AREA_ADDRESS 0x08060000  /* セクタ7 の先頭の番地 */

/* 保存場所 area の先頭の番地 */
static uint32_t area_address(flash_area_t area)
{
  if (area == FLASH_AREA_GOAL) {
    return GOAL_AREA_ADDRESS;
  }
  return MAZE_AREA_ADDRESS;
}

/* 保存場所 area のセクタの番号（HAL の FLASH_SECTOR_6 など） */
static uint32_t area_sector(flash_area_t area)
{
  if (area == FLASH_AREA_GOAL) {
    return FLASH_SECTOR_6;
  }
  return FLASH_SECTOR_7;
}

/* 前の操作で残ったエラーの印を消す（1 つずつ消す） */
static void clear_error_flags(void)
{
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
}

/* 保存場所 area（セクタ 1 つ）を消す。どの語も 0xFFFFFFFF になる。うまくいったら true。
 * 消しているあいだ（約 1〜2 秒）は CPU が止まるので、モーターを止めているときだけ呼ぶ */
bool flash_memory_erase(flash_area_t area)
{
  FLASH_EraseInitTypeDef erase;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;  /* セクタを消す */
  erase.Banks = FLASH_BANK_1;                 /* STM32F411 の Flash は 1 つだけ */
  erase.Sector = area_sector(area);
  erase.NbSectors = 1;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 電源が 2.7〜3.6V のときの設定（32 ビットずつ消す・書く） */
  uint32_t error_sector = 0;                  /* 失敗したときに、そのセクタの番号が入る（使わない） */

  HAL_FLASH_Unlock();
  clear_error_flags();
  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &error_sector);
  HAL_FLASH_Lock();

  return status == HAL_OK;
}

/* 保存場所 area の index 番目の語（0 から数える）に value を書く。うまくいったら true。
 * 書けるのは、消したあとの語だけ */
bool flash_memory_write_word(flash_area_t area, int index, uint32_t value)
{
  if (index < 0 || index >= FLASH_AREA_WORD_COUNT) {
    return false;  /* 保存場所の外 */
  }
  uint32_t address = area_address(area) + index * 4;  /* 1 語は 4 バイト */

  HAL_FLASH_Unlock();
  clear_error_flags();
  HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, value);
  HAL_FLASH_Lock();

  return status == HAL_OK;
}

/* 保存場所 area の index 番目の語を読む。保存場所の外なら、消したあとの値を返す */
uint32_t flash_memory_read_word(flash_area_t area, int index)
{
  if (index < 0 || index >= FLASH_AREA_WORD_COUNT) {
    return FLASH_ERASED_WORD;
  }
  /* Flash は、RAM と同じように番地を指定して読める。
   * 保存場所の先頭の番地を「uint32_t の配列」とみなして、その index 番目を読む */
  const uint32_t *words = (const uint32_t *)area_address(area);
  return words[index];
}
