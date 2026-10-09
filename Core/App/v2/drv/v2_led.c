#include "v2_led.h"
#include "main.h"

#define V2_LED_ON   GPIO_PIN_SET
#define V2_LED_OFF  GPIO_PIN_RESET

static const struct { GPIO_TypeDef* port; uint16_t pin; } kV2Led[V2_LED_COUNT] = {
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
  { LED3_GPIO_Port, LED3_Pin },
};

static uint8_t s_mask = 0u;

void v2_led_set(uint8_t id, uint8_t on)
{
  if (id >= V2_LED_COUNT) return;
  if (on) s_mask |=  (uint8_t)(1u << id);
  else    s_mask &= (uint8_t)~(1u << id);
  HAL_GPIO_WritePin(kV2Led[id].port, kV2Led[id].pin, on ? V2_LED_ON : V2_LED_OFF);
}

void v2_led_all(uint8_t on)
{
  uint8_t i;
  for (i = 0u; i < V2_LED_COUNT; i++) v2_led_set(i, on);
}

void v2_led_mask(uint8_t mask)
{
  uint8_t i;
  for (i = 0u; i < V2_LED_COUNT; i++) v2_led_set(i, (uint8_t)((mask >> i) & 1u));
}

uint8_t v2_led_get_mask(void) { return s_mask; }

void v2_led_init(void) { v2_led_all(0u); }
