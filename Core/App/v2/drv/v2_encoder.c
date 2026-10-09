#include "v2_encoder.h"
#include "tim.h"

static int32_t s_prev[2] = { 0, 0 };

void v2_enc_init(void)
{
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  s_prev[V2_ENC_LEFT]  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
  s_prev[V2_ENC_RIGHT] = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
}

int32_t v2_enc_raw(int ch)
{
  if (ch == V2_ENC_LEFT) return (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
  return (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
}

int32_t v2_enc_take_delta(int ch)
{
  int32_t cur = (ch == V2_ENC_LEFT) ? (int32_t)__HAL_TIM_GET_COUNTER(&htim3)
                                    : (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
  /* 带符号 16 位差：16 位计数器回绕时得到正确的小增量 */
  int32_t d = (int32_t)(int16_t)((uint16_t)cur - (uint16_t)s_prev[ch]);
  s_prev[ch] = cur;
  return d;
}
