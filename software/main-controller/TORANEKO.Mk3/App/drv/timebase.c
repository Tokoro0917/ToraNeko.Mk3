#include "drv/timebase.h"

#include "main.h"

uint32_t timebase_get_ms(void)
{
  return HAL_GetTick();
}

uint32_t timebase_get_sysclk_hz(void)
{
  return HAL_RCC_GetSysClockFreq();
}
