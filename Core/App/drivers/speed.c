#include "speed.h"
#include "app_config.h"
#include "main.h"
#include "drivers/motor.h"     /* MOTOR_LEFT / MOTOR_RIGHT */
#if USE_ENCODER
#include "drivers/encoder.h"
#endif

/* 霍尔测速: 每轮1块磁铁, 每圈1脉冲(旧车用, 新车已退役) */
#define HALL_PULSES_PER_REV  1

#if USE_ENCODER
static uint32_t enc_prev[2] = {0, 0};
#else
static volatile uint32_t hall_count[2] = {0, 0};   /* [0]=左轮, [1]=右轮(霍尔, 已退役) */
#endif
static int16_t           rpm_arr[2]     = {0, 0};
static uint32_t          last_ms        = 0;       /* 上次算 RPM 的时刻 */

void speed_init(void)
{
#if USE_ENCODER
  encoder_init();     /* 启动 TIM3/TIM4 正交解码计数 */
#endif
  last_ms = HAL_GetTick();
  speed_update();
}

/* ★2026-09-20 修：原来按"固定 500ms 窗口"算 RPM，遥测提速到 50ms 后
   RPM 只剩真值的 1/10。改成按【距上次调用的真实间隔】算，任何调用周期都正确。 */
void speed_update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t dt  = now - last_ms;
  if (dt == 0) dt = 1;
  last_ms = now;

#if USE_ENCODER
  for (int v = 0; v < 2; v++)
  {
    uint32_t c  = (uint32_t)encoder_get_count(v);
    int32_t  d  = (int32_t)c - (int32_t)enc_prev[v];   /* 本窗口增量(有符号, 近似处理绕回) */
    enc_prev[v] = c;
    if (d < 0) d = -d;
    /* rpm = 增量 × (60000/dt) / 每圈计数 */
    rpm_arr[v] = (int16_t)(((int32_t)d * 60000) / ((int32_t)ENCODER_PPR * (int32_t)dt));
  }
#else
  /* 霍尔: 同样按真实间隔算, 然后清零计数 */
  rpm_arr[0] = (int16_t)(((int32_t)hall_count[0] * 60000) / ((int32_t)HALL_PULSES_PER_REV * (int32_t)dt));
  rpm_arr[1] = (int16_t)(((int32_t)hall_count[1] * 60000) / ((int32_t)HALL_PULSES_PER_REV * (int32_t)dt));
  hall_count[0] = 0;
  hall_count[1] = 0;
#endif
}

int16_t speed_get_rpm(int vol)
{
  if (vol == MOTOR_LEFT)  return rpm_arr[0];
  if (vol == MOTOR_RIGHT) return rpm_arr[1];
  return 0;
}

uint16_t speed_get_count(int vol)
{
#if USE_ENCODER
  return (uint16_t)encoder_get_count(vol);
#else
  if (vol == MOTOR_LEFT)  return (uint16_t)hall_count[0];
  if (vol == MOTOR_RIGHT) return (uint16_t)hall_count[1];
  return 0;
#endif
}

#if !USE_ENCODER
/* 霍尔脉冲中断回调(旧车: PA11=左, PA12=右)。
   ⚠️ 2026-09-20 新车已无霍尔(PA11 改成蜂鸣器 PWM, EXTI 也已在 CubeMX 里删掉),
   这段只作历史参考, 用编码器时不会编译进来。
   ⚠️ 必须按位与(&)判断! EXTI15_10_IRQHandler 传进来的是合并掩码, 用 == 永远不相等(2026-08-19 修复) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin & GPIO_PIN_11) hall_count[MOTOR_LEFT]++;
  if (GPIO_Pin & GPIO_PIN_12) hall_count[MOTOR_RIGHT]++;
}
#endif
