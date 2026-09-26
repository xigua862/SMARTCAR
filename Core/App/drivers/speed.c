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
    {
      /* ★★★ 2026-09-23 修溢出：原来直接 (int16_t) 强转，超过 32767 会【回绕成负数】。
         实车证据：架空测试打出 `RPM2=-17231`（转速不可能是负的）。
         触发条件：编码器计数【跳变】(接线抖动 / 计数器绕回) → d 突然变大 →
                  d*60000/(422*dt) 超过 int16 上限 → 回绕成负数，看起来像"倒转"。
         修法：先在 int32 里算完，再夹到 ±30000 才转 int16。
              （真实转速上限约 620rpm，这个钳位绝不会误伤正常读数。）
         ★只影响【显示】不影响行为：速度闭环当前是关的（USE_SPEED_LOOP=0）。 */
      int32_t rpm = ((int32_t)d * 60000) / ((int32_t)ENCODER_PPR * (int32_t)dt);

      /* ★★★ 2026-09-26 【修根因】可信性校验，两道，都在钳位之前 ★★★
         原来只有下面那对 ±30000 钳位 → "编码器计数跳变"产生的垃圾值(-30000)
         会【原封不动喂给速度 PID】→ 输出在 0↔60 之间 bang-bang
         → 小车一顿一顿往前窜。

         实测铁证（同一趟数据，两个事实互相矛盾）：
             OD 逐拍增量稳定 5~6mm、负增量 0 次（车实际走得很顺）
             RPM 却报 -30000、avg=-1620（负的）
          → 是【读数被污染】，不是车真的抖。
         ★2026-09-26 追加：这两道软件守卫【只是兜底】。污染源（编码器输入滤波
           从未真正生效，见 encoder.c 顶部长注释）已在当日修好；但守卫保留 ——
           它挡的是"计数跳变"，与污染源是否修好无关。

         第①道【计数跳变守卫】：10ms 内增量不可能超过 _max_d。
           本车实测上限约 465rpm(≈273rpm@60PWM × 余量)，即
             465/60 × 422 × 0.6 ≈ 1963 计数/秒 → 10ms 窗口 ≤ 33 计数。
           取 2 倍余量 = 70。超过这个数只可能是噪声毛刺/漏计数，
           此时【保留上一次的有效 rpm】而不是硬算一个垃圾值。
         第②道【变化率限制】：即便增量合法，转速在 10ms 内也不可能突变。
           物理上限约 620rpm，机电时间常数几十 ms → 每拍变化 ≤50rpm 量级。
           取 ±150rpm（比任何真实瞬态都宽），挡住跳变产生的上千级垃圾。

         ★为什么第①道用"保留上次值"而不是"丢弃更新"：
           保留 rpm_prev 既不产生垃圾、反馈也仍在推进，不会让 PID 拿到陈旧值。
         ★为什么不简单地 if(rpm<0) rpm=0：
           本车【有倒转工况】（丢线后原地找线用 LOST_SPIN_SPEED 负值），
           speed.c 无从知道车此刻的意图方向 → 不能一刀切掉负值。 */
      {
        static int32_t rpm_prev[2] = {0, 0};
        const int32_t  d_max = 70;          /* 10ms 窗口内的合法计数上限（2 倍余量） */
        if (d > d_max)
        {
          rpm = rpm_prev[v];                /* 计数跳变 → 沿用上次有效值 */
        }
        else
        {
          int32_t lo = rpm_prev[v] - 150;
          int32_t hi = rpm_prev[v] + 150;
          if (rpm > hi) rpm = hi;
          if (rpm < lo) rpm = lo;
          rpm_prev[v] = rpm;
        }
      }

      if (rpm >  30000) rpm =  30000;
      if (rpm < -30000) rpm = -30000;
      rpm_arr[v] = (int16_t)rpm;
    }
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
