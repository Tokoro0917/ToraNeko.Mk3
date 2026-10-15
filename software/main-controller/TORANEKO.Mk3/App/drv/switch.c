/*
 * switch.c
 * ユーザースイッチ SW1 の読み取り
 *
 * SW1 は PA12（CubeMX のラベル SW_USER）。押すと High、離すと Low（R9 10kΩ でプルダウン）。
 * ピン名（SW_USER_GPIO_Port、SW_USER_Pin）は main.h に定義されている。
 * チャタリング（押した瞬間に High と Low が何回か入れ替わること）は、読む側（sys/ui.c）で取り除く。
 */
#include "drv/switch.h"

#include "main.h"

/* 今、スイッチが押されていれば true */
bool switch_is_pressed(void)
{
  if (HAL_GPIO_ReadPin(SW_USER_GPIO_Port, SW_USER_Pin) == GPIO_PIN_SET) {
    return true;
  }
  return false;
}
