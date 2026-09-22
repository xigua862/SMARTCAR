#include "led.h"
#include "app_config.h"
#include "main.h"

static const struct {
  GPIO_TypeDef* port;
  uint16_t      pin;
} kLed[LED_COUNT] = {
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
  { LED3_GPIO_Port, LED3_Pin },
};

static uint8_t led_mask_cache = 0;

void led_init(void)
{
  led_all(0);
}

void led_set(uint8_t id, uint8_t on)
{
  if (id >= LED_COUNT) return;

  if (on) led_mask_cache |=  (uint8_t)(1u << id);
  else    led_mask_cache &= (uint8_t)~(1u << id);

  HAL_GPIO_WritePin(kLed[id].port, kLed[id].pin, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

void led_all(uint8_t on)
{
  for (uint8_t i = 0; i < LED_COUNT; i++) led_set(i, on);
}

void led_mask(uint8_t mask)
{
  for (uint8_t i = 0; i < LED_COUNT; i++) led_set(i, (mask >> i) & 1u);
}

uint8_t led_get_mask(void)
{
  return led_mask_cache;
}
