#include "drv/led.h"

#include "main.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} led_pin_t;

/* 添字 0 = LED1。ピン名は .ioc のラベル（main.h） */
static const led_pin_t s_leds[LED_COUNT] = {
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
  { LED3_GPIO_Port, LED3_Pin },
  { LED4_GPIO_Port, LED4_Pin },
  { LED5_GPIO_Port, LED5_Pin },
  { LED6_GPIO_Port, LED6_Pin },
  { LED7_GPIO_Port, LED7_Pin },
};

void led_init(void)
{
  led_set_mask(0);
}

void led_set(uint8_t num, bool on)
{
  if (num < 1 || num > LED_COUNT) {
    return;
  }
  const led_pin_t *led = &s_leds[num - 1];
  HAL_GPIO_WritePin(led->port, led->pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void led_set_mask(uint8_t mask)
{
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    led_set(i + 1, (mask >> i) & 1U);
  }
}
