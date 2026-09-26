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

/* ============================================================================
 * ★★★ 2026-09-26【修一个"改了却没生效"的根因】编码器输入数字滤波 ★★★
 *
 *  背景：这个 bug 让"一抽一抽"的真正根因一直没被修掉。
 *  经过：2026-09-26 在 tim.c 的 MX_TIM3_Init/MX_TIM4_Init 里把
 *        sConfig.IC1Filter/IC2Filter 设成 0x0F —— 但设在了
 *        【CubeMX 生成的那几行 sConfig.ICxxFilter = 0 之前】，
 *        于是当场被覆盖回 0，再交给 HAL_TIM_Encoder_Init →
 *        真正写进 CCMR1 的仍然是 0 = 【完全不滤波】。
 *
 *  为什么能确定它没生效（实车铁证，不需要示波器）：
 *    · RPM 报 747，而 10ms 窗口的物理上限是 70 计数 ≈ 213rpm；
 *    · 上一拍 28、下一拍 462（相邻两拍只隔 10ms），
 *      双电机堵转也不可能一个跟斗翻 400rpm。
 *    ⇒ 编码器计数仍被电机 PWM 噪声污染 → 速度环吃垃圾反馈 → 输出乱摆
 *      → 表现为车"一顿一顿"和转弯时"疯狂摆头"。
 *
 *  这里在【启动之后再补写一次】，形成双保险：
 *    ① tim.c 里那两行已一并改成 0x0F（CubeMX 重新生成后依然正确）；
 *    ② 这里用 HAL 同样的位操作直接写 CCMR1 的 IC1F/IC2F。
 *  为什么要②：本项目有"CubeMX 重新生成破坏工程"的前车之鉴，
 *    而且 HAL_TIM_Encoder_Start 内部不碰滤波位，写在这里不会被覆盖。
 *
 *  写寄存器而不是用 sConfig 的原因：本函数看不到 MX_*_Init 的局部变量，
 *  而 ICxPSC 本来就是 0、CC1S/CC2S 由 HAL 配好，只动滤波这 8 位最安全。
 *  0x0F = f_DTS/32 采样率下连续 8 次一致才认账（可滤 <~4.3µs 毛刺）；
 *  而编码器最高约 6.5kHz（465rpm×14 沿）= 每脉冲 154µs，不会误滤真实脉冲。
 * ==========================================================================*/
#define ENC_INPUT_FILTER   0x0FU

static void encoder_apply_input_filter(TIM_HandleTypeDef* htim)
{
  uint32_t ccmr1 = htim->Instance->CCMR1;
  ccmr1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);
  ccmr1 |= ((uint32_t)ENC_INPUT_FILTER << 4U) | ((uint32_t)ENC_INPUT_FILTER << 12U);
  htim->Instance->CCMR1 = ccmr1;
}

void encoder_init(void)
{
  HAL_TIM_Encoder_Start(ENC1_TIM, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(ENC2_TIM, TIM_CHANNEL_ALL);
  encoder_apply_input_filter(ENC1_TIM);   /* ★双保险：启动后补写滤波位 */
  encoder_apply_input_filter(ENC2_TIM);
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
