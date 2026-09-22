#include "buzzer.h"
#include "app_config.h"
#include "main.h"
#include "tim.h"        /* htim1 */

/* TIM1 挂在 APB2 = 72MHz；预分频 72 → 计数时钟 1MHz，1 个计数 = 1µs */
#define BUZZER_TIM_CLK_HZ   72000000u
#define BUZZER_PRESCALER    72u
#define BUZZER_CNT_HZ       (BUZZER_TIM_CLK_HZ / BUZZER_PRESCALER)   /* 1MHz */

static uint8_t  beeping = 0;
static uint32_t off_at  = 0;

void buzzer_init(void)
{
  __HAL_TIM_SET_PRESCALER(&htim1, BUZZER_PRESCALER - 1u);
  __HAL_TIM_SET_AUTORELOAD(&htim1, (BUZZER_CNT_HZ / BUZZER_FREQ_HZ) - 1u);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);   /* 上电静音 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  beeping = 0;
  off_at  = 0;
}

void buzzer_on(uint32_t freq_hz)
{
  if (freq_hz < 100u) freq_hz = 100u;

  uint32_t arr = BUZZER_CNT_HZ / freq_hz;            /* 每周期的计数个数 */
  if (arr < 2u) arr = 2u;

  __HAL_TIM_SET_AUTORELOAD(&htim1, arr - 1u);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, arr / 2u);   /* 50% 占空比 */
}

void buzzer_off(void)
{
  beeping = 0;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

void buzzer_beep(uint16_t ms)
{
  buzzer_on(BUZZER_FREQ_HZ);
  beeping = 1;
  off_at  = HAL_GetTick() + ms;
}

void buzzer_update(void)
{
  if (beeping && (int32_t)(HAL_GetTick() - off_at) >= 0)
  {
    buzzer_off();
  }
}

uint8_t buzzer_is_on(void)
{
  return beeping;
}
