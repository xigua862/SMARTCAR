#include "v2_buzzer.h"
#include "main.h"
#include "tim.h"

/* TIM1：1MHz 计数（PSC=71）→ ARR=999 = 1kHz 更新中断 = v2 的系统节拍。
   CH4 占空比 50% = 1kHz 方波驱动无源蜂鸣器。
   ★与卡里"PSC=71/ARR=71"的偏差：那组数是 13.9kHz，与"1kHz 节拍"目标矛盾，
     本实现按 1kHz 意图取 ARR=999（回报里登记）。 */
#define V2_BUZZER_ARR   1000u
#define V2_BUZZER_DUTY  500u

static uint16_t s_beep_ticks = 0u;

void v2_buzzer_init(void)
{
  TIM1->PSC  = 72u - 1u;
  TIM1->ARR  = V2_BUZZER_ARR - 1u;
  TIM1->EGR  = TIM_EGR_UG;          /* 立刻装载 PSC/ARR */
  TIM1->CCR4 = 0u;                  /* 静音 */
  TIM1->CR1 |= TIM_CR1_ARPE;        /* ARR 预装载 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   /* 启动计数 + CH4 输出（含 MOE） */

  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void v2_buzzer_set(uint8_t on)
{
  if (on)
  {
    TIM1->CCR4 = V2_BUZZER_DUTY;
  }
  else
  {
    TIM1->CCR4 = 0u;               /* 占空比归 0：不响、不留直流 */
    s_beep_ticks = 0u;
  }
}

void v2_buzzer_beep(uint16_t ms)
{
  s_beep_ticks = ms;               /* 1kHz 节拍下 1 tick = 1ms */
  v2_buzzer_set(1u);
}

void v2_buzzer_tick(void)
{
  if (s_beep_ticks != 0u)
  {
    s_beep_ticks--;
    if (s_beep_ticks == 0u) TIM1->CCR4 = 0u;
  }
}

uint8_t v2_buzzer_is_on(void)
{
  return (TIM1->CCR4 != 0u) ? 1u : 0u;
}
