#include "key.h"
#include "app_config.h"
#include "main.h"

static uint8_t  stable      = 0;   /* 消抖后的电平: 1=按住 */
static uint8_t  last_raw    = 0;
static uint8_t  cnt         = 0;   /* 连续相同的采样次数 */
static uint8_t  press_evt   = 0;
static uint8_t  long_evt    = 0;
static uint8_t  long_fired  = 0;   /* 本次按住已经报过长按 */
static uint32_t press_tick  = 0;

static uint8_t key_raw_down(void)
{
  return (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

void key_init(void)
{
  last_raw   = key_raw_down();
  stable     = last_raw;
  cnt        = 0;
  press_evt  = 0;
  long_evt   = 0;
  long_fired = 0;
}

void key_update(void)
{
  uint8_t raw = key_raw_down();

  /* 采样计数：电平变了就重新数 */
  if (raw != last_raw) { last_raw = raw; cnt = 0; }
  else if (cnt < KEY_DEBOUNCE_TICKS) cnt++;

  /* 连续 KEY_DEBOUNCE_TICKS 次相同 → 认为电平稳定 */
  if ((cnt >= KEY_DEBOUNCE_TICKS) && (stable != raw))
  {
    stable = raw;
    if (stable)
    {
      press_tick = HAL_GetTick();
      long_fired = 0;
    }
    else if (!long_fired)
    {
      press_evt = 1;              /* 松开且没报过长按 = 短按 */
    }
  }

  /* 按住够久 → 长按事件(只报一次) */
  if (stable && !long_fired && (HAL_GetTick() - press_tick >= KEY_LONG_PRESS_MS))
  {
    long_evt   = 1;
    long_fired = 1;
  }
}

uint8_t key_take_press(void)
{
  uint8_t e = press_evt;
  press_evt = 0;
  return e;
}

uint8_t key_take_long(void)
{
  uint8_t e = long_evt;
  long_evt = 0;
  return e;
}

uint8_t key_is_down(void)
{
  return stable;
}

void key_clear_events(void)
{
  press_evt = 0;
  long_evt  = 0;
}
