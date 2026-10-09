#include "v2_key.h"
#include "main.h"

#define V2_KEY_DEBOUNCE_TICKS  3u        /* 30ms（10ms 节拍 × 3） */
#define V2_KEY_LONG_PRESS_MS   1000u
#define V2_KEY_BOOT_IGNORE_MS  300u      /* 上电头 300ms 不认按键（防压住/抖动） */

static uint8_t  s_stable = 0u, s_last_raw = 0u, s_cnt = 0u;
static uint8_t  s_press_evt = 0u, s_long_evt = 0u, s_long_fired = 0u;
static uint32_t s_press_tick = 0u, s_boot_tick = 0u;

static uint8_t v2_key_raw_down(void)
{
  return (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

void v2_key_init(void)
{
  s_boot_tick  = HAL_GetTick();
  s_press_tick = s_boot_tick;
  s_last_raw   = v2_key_raw_down();
  s_stable     = s_last_raw;
  s_cnt        = 0u;
  s_press_evt  = 0u;
  s_long_evt   = 0u;
  s_long_fired = 0u;
}

void v2_key_update(void)
{
  uint8_t raw = v2_key_raw_down();

  if (raw != s_last_raw) { s_last_raw = raw; s_cnt = 0u; }
  else if (s_cnt < V2_KEY_DEBOUNCE_TICKS) s_cnt++;

  if ((s_cnt >= V2_KEY_DEBOUNCE_TICKS) && (s_stable != raw))
  {
    s_stable = raw;
    if (s_stable)
    {
      s_press_tick = HAL_GetTick();
      s_long_fired = 0u;
    }
    else if (!s_long_fired)
    {
      s_press_evt = 1u;
    }
  }

  if ((HAL_GetTick() - s_boot_tick) < V2_KEY_BOOT_IGNORE_MS) return;
  if (s_stable && !s_long_fired && ((HAL_GetTick() - s_press_tick) >= V2_KEY_LONG_PRESS_MS))
  {
    s_long_evt   = 1u;
    s_long_fired = 1u;
  }
}

uint8_t v2_key_take_press(void) { uint8_t e = s_press_evt; s_press_evt = 0u; return e; }
uint8_t v2_key_take_long(void)  { uint8_t e = s_long_evt;  s_long_evt  = 0u; return e; }
uint8_t v2_key_is_down(void)    { return s_stable; }
void    v2_key_clear_events(void) { s_press_evt = 0u; s_long_evt = 0u; }
