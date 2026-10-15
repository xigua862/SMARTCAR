#include "speed.h"
#include "app_config.h"
#include "main.h"
#include "drivers/motor.h"
#if USE_ENCODER
#include "drivers/encoder.h"
#endif

/* 保留 G40 的转速幅值接口及每拍变化限制，不改变编码器方向约定。 */
#define SPEED_MAX_RPM          1000UL
#define SPEED_MAX_PERIOD_MS    100UL
#define SPEED_MAX_RPM_STEP       50    /* ★R80（2026-10-08）150 -> 50。理由：150 相当于"每 10ms 允许转速跳 70% 满量程"✗（文档自己写着 10ms 窗口物理上限≈213rpm），速度环照样看到大跳变 ⇒ 主干道/弯道都"一抽一抽"。收紧到 50 ⇒ 既挡垃圾值、又真正平滑反馈 ✓。★若发现响应变迟钝（加速变慢）⇒ 放到 80。 */

/* ★★★ 2026-10-08（1008-R81）【转速反馈 EMA（α=1/4，纯整数）】★★★
   为什么：7 计数/圈 + 10ms 窗口 ⇒ 130rpm 时每拍只有 ~2 个计数 ⇒ 量化噪声 ≈ ±50% ✗
         直接喂给 PID ⇒ 输出一跳一跳 ⇒ 车"一抽一抽"（主干道也抽 ✓）。
   做法：对返回给控制环的转速做一阶低通（rpm = (3*旧 + 新)/4 ✓），纯整数、无浮点 ✓。
   ★回退：USE_RPM_EMA 设 0（回到原始转速反馈）。★若发现响应变迟钝/加速变慢 ⇒ 把 3/4 改成 1/2。 */
#define USE_RPM_EMA              1
static int16_t rpm_ema[2] = {0, 0};   /* ★R81：转速反馈的低通状态 */
#define HALL_PULSES_PER_REV     1UL

#if USE_ENCODER
static uint16_t enc_prev[2] = {0, 0};
#else
static volatile uint32_t hall_count[2] = {0, 0};
static uint32_t hall_prev[2] = {0, 0};
#endif
static int16_t  rpm_arr[2] = {0, 0};
static uint32_t last_ms = 0;
static uint32_t period_ms = 0;
static uint8_t  feedback_valid = 0;

void speed_init(void)
{
  int v;
#if USE_ENCODER
  encoder_init();
#endif
  for (v = 0; v < 2; v++)
  {
#if USE_ENCODER
    enc_prev[v] = (uint16_t)encoder_get_count(v);
#else
    hall_prev[v] = hall_count[v];
#endif
    rpm_arr[v] = 0;
  }
  last_ms = HAL_GetTick();
  period_ms = 0;
  feedback_valid = 0;
}

void speed_update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t dt = now - last_ms;
  int v;

  /* 同一毫秒重复调用不能消耗计数，也不能伪造一个 1ms 的测速窗口。 */
  if (dt == 0u) return;

  last_ms = now;
  period_ms = dt;
  feedback_valid = (uint8_t)(dt <= SPEED_MAX_PERIOD_MS);

  for (v = 0; v < 2; v++)
  {
    uint32_t delta;
    uint32_t ppr;
    uint32_t max_count;
    int32_t rpm;
#if USE_ENCODER
    uint16_t count = (uint16_t)encoder_get_count(v);
    int32_t signed_delta = (int32_t)count - (int32_t)enc_prev[v];
    enc_prev[v] = count;

    /* TIM3/TIM4 为 16 位：先恢复跨 0/65535 的最短有符号增量，再取幅值。 */
    if (signed_delta > 32767) signed_delta -= 65536;
    if (signed_delta < -32768) signed_delta += 65536;
    delta = (uint32_t)((signed_delta < 0) ? -signed_delta : signed_delta);
    ppr = ENCODER_PPR;
#else
    /* 霍尔使用累计计数快照，避免读取后清零与中断加一之间丢失脉冲。 */
    uint32_t count = hall_count[v];
    delta = count - hall_prev[v];
    hall_prev[v] = count;
    ppr = HALL_PULSES_PER_REV;
#endif

    /* 超长窗口重新建立计数基准，不把过时的平均转速当成当前反馈。 */
    if (dt > SPEED_MAX_PERIOD_MS) continue;

    /* 按实际窗口放大计数上限；额外两个计数覆盖量化误差与采样边界。 */
    max_count = (SPEED_MAX_RPM * ppr * dt) / 60000UL + 2UL;
    if (delta > max_count)
    {
      feedback_valid = 0;
      continue;
    }

    rpm = (int32_t)((delta * 60000UL) / (ppr * dt));
#if USE_ENCODER
    /* 沿用 G40 的每拍变化限制，不额外增加低通滤波或改变正常响应。 */
    if (rpm > (int32_t)rpm_arr[v] + SPEED_MAX_RPM_STEP)
      rpm = (int32_t)rpm_arr[v] + SPEED_MAX_RPM_STEP;
    if (rpm < (int32_t)rpm_arr[v] - SPEED_MAX_RPM_STEP)
      rpm = (int32_t)rpm_arr[v] - SPEED_MAX_RPM_STEP;
#endif
    if (rpm > 30000) rpm = 30000;
#if USE_RPM_EMA
    /* ★R81：一阶低通（α=1/4）—— 抹掉 10ms 窗口的量化噪声（±50%），让速度环看到平滑反馈 ✓ */
    rpm_ema[v] = (int16_t)(((int32_t)rpm_ema[v] * 3 + (int32_t)rpm) / 4);
    rpm_arr[v] = rpm_ema[v];
#else
    rpm_arr[v] = (int16_t)rpm;
#endif
  }
}

int16_t speed_get_rpm(int vol)
{
  if (vol == MOTOR_LEFT) return rpm_arr[0];
  if (vol == MOTOR_RIGHT) return rpm_arr[1];
  return 0;
}

uint32_t speed_period_ms(void)
{
  return period_ms;
}

uint8_t speed_feedback_valid(void)
{
  /* 只能识别超时或不合理计数；断线造成的恒定零计数仍需单独诊断。 */
  return (uint8_t)(feedback_valid &&
                  ((uint32_t)(HAL_GetTick() - last_ms) <= SPEED_MAX_PERIOD_MS));
}

uint16_t speed_get_count(int vol)
{
  if ((vol != MOTOR_LEFT) && (vol != MOTOR_RIGHT)) return 0;
#if USE_ENCODER
  return (uint16_t)encoder_get_count(vol);
#else
  return (uint16_t)hall_count[vol];
#endif
}

#if !USE_ENCODER
/* 旧车霍尔中断兼容路径；合并掩码必须按位判断。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin & GPIO_PIN_11) hall_count[MOTOR_LEFT]++;
  if (GPIO_Pin & GPIO_PIN_12) hall_count[MOTOR_RIGHT]++;
}
#endif
