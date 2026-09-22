#include "encoder.h"
#include "app_config.h"
#include "drivers/motor.h"     /* MOTOR_LEFT / MOTOR_RIGHT */

#if USE_ENCODER
#include "tim.h"

/* ============================================================================
 * 编码器实现 (USE_ENCODER=1)
 *  ★ 2026-09-20 实况: CubeMX 已把 TIM3/TIM4 配成 Encoder Mode(TI12 四倍频)
 *      电机1 = 左轮 → 编码器接 TIM3_CH1/CH2 = PA6/PA7
 *      电机2 = 右轮 → 编码器接 TIM4_CH1/CH2 = PB6/PB7
 *    PPR 见 app_config.h 的 ENCODER_PPR = 422 (= 11 × 9.6 × 4倍频), 装好手转一圈核对。
 * ==========================================================================*/
extern TIM_HandleTypeDef htim3;   /* 电机1(左轮)编码器定时器 — 按实际改 */
extern TIM_HandleTypeDef htim4;   /* 电机2(右轮)编码器定时器 — 按实际改 */

#define ENC1_TIM  (&htim3)
#define ENC2_TIM  (&htim4)

void encoder_init(void)
{
  HAL_TIM_Encoder_Start(ENC1_TIM, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(ENC2_TIM, TIM_CHANNEL_ALL);
}

int32_t encoder_get_count(int vol)
{
  if (vol == MOTOR_LEFT)
    return (int32_t)__HAL_TIM_GET_COUNTER(ENC1_TIM);
  return (int32_t)__HAL_TIM_GET_COUNTER(ENC2_TIM);
}

#else  /* USE_ENCODER=0: 空实现, 保证任何模式都能链接 */

void encoder_init(void) { }
int32_t encoder_get_count(int vol) { (void)vol; return 0; }

#endif /* USE_ENCODER */
