#include "v2_app_task.h"
#include "main.h"
#include "tim.h"

#include "v2_uart.h"
#include "v2_line_sensor.h"
#include "v2_motor.h"
#include "v2_encoder.h"
#include "v2_odom.h"
#include "v2_imu.h"
#include "v2_pose.h"
#include "v2_buzzer.h"
#include "v2_led.h"
#include "v2_key.h"
#include "v2_track.h"
#include "v2_steering.h"
#include "v2_angulate.h"
#include "v2_speeds.h"
#include "v2_param.h"
#include "v2_tune.h"
#include "v2_telemetry.h"
#include "v2_selftest.h"
#include "v2_car_fsm.h"

/* ---- 跨拍共享（ISR 写 / 主循环读；读侧一律关中断快照） ---- */
static volatile line_reading_t s_line     = { 0u, 0u, 0 };
static volatile uint32_t       s_isr_ticks    = 0u;
static volatile uint32_t       s_isr_us_sum   = 0u;
static volatile uint32_t       s_isr_us_last  = 0u;
static volatile uint32_t       s_isr_overlong = 0u;

static uint32_t s_t_ui  = 0u;
static uint32_t s_t_tel = 0u;

/* ============================================================================
 * ★1kHz 节拍：TIM1 更新中断（NVIC 优先级 5）
 *   顺序固定：DWT 起表 → 感知 → 编码器/里程 → 内环 stub → 哨兵
 *   铁律：无浮点、无 printf、无 I2C、无长循环（预算 <150µs，>300µs 记 overlong）
 * ==========================================================================*/
void TIM1_UP_IRQHandler(void)
{
  if ((TIM1->SR & TIM_SR_UIF) != 0u)
  {
    uint32_t t0 = v2_dwt_cycles();

    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    s_line = v2_line_read();      /* 感知：8 路突发采样 + 多数表决 + 差比和误差 */
    v2_odom_tick();               /* 编码器增量 -> 里程（16 位回绕安全） */
    v2_buzzer_tick();             /* 蜂鸣器计时收尾（1 tick = 1ms） */

    /* 中环 / 内环 stub：M0 输出 0（电机不动）；M1 在此写 PWM（ARR 预装载） */
    s_isr_ticks++;

    {
      uint32_t dt = (uint32_t)(v2_dwt_cycles() - t0);
      uint32_t us = dt / (SystemCoreClock / 1000000u);
      s_isr_us_last = us;
      if (s_isr_us_sum < 0xFFFFFF00u) s_isr_us_sum += us;
      if (us > 300u) s_isr_overlong++;
    }
  }
}

/* ---------------- 快照（关中断短临界区） ---------------- */
void v2_app_line_take(line_reading_t* out)
{
  uint32_t pm = __get_PRIMASK();
  __disable_irq();
  if (out != 0) *out = s_line;
  if (!pm) __enable_irq();
}

void v2_app_pose_take(pose_t* out)
{
  uint32_t pm = __get_PRIMASK();
  __disable_irq();
  if (out != 0) *out = v2_pose_get();
  if (!pm) __enable_irq();
}

void v2_app_isr_take(uint32_t* us_avg, uint32_t* overlong, uint32_t* ticks)
{
  uint32_t pm = __get_PRIMASK();
  __disable_irq();
  {
    uint32_t t = s_isr_ticks;
    if (ticks != 0)    *ticks = t;
    if (overlong != 0) *overlong = s_isr_overlong;
    if (us_avg != 0)   *us_avg = (t != 0u) ? (s_isr_us_sum / t) : 0u;
    s_isr_ticks = 0u;
    s_isr_us_sum = 0u;
    s_isr_overlong = 0u;
  }
  if (!pm) __enable_irq();
}

uint32_t v2_app_isr_us_last(void) { return s_isr_us_last; }

/* ---------------- 初始化 / 主循环 ---------------- */
void v2_app_init(void)
{
  v2_uart_init();
  v2_line_init();          /* ★含 v2_dwt_init（ISR 计时 + 突发采样定间距都靠它） */
  v2_enc_init();
  v2_odom_init();
  v2_motor_init();
  v2_led_init();
  v2_key_init();
  v2_imu_init();           /* 含上电零偏标定（车必须静止） */
  v2_pose_init();
  v2_track_init();
  v2_steering_init();
  v2_angulate_init();
  v2_speeds_init();
  v2_param_init();
  v2_selftest_init();
  v2_car_fsm_init();

  v2_telemetry_banner();

  v2_buzzer_init();        /* ★最后开 1kHz 节拍（此前 DWT/编码器都已就绪） */
  v2_buzzer_beep(120);     /* 上电响一下 = 心跳，确认固件在跑 */
}

void v2_app_loop(void)
{
  uint32_t now;

  v2_uart_poll();          /* 高频收发：不许在任何地方长阻塞 */

  now = HAL_GetTick();

  if ((uint32_t)(now - s_t_ui) >= 10u)      /* 100Hz：人机 + IMU 轮询 */
  {
    s_t_ui = now;
    v2_key_update();
    v2_imu_poll();         /* I2C + 浮点只允许出现在这里（主循环） */
    v2_car_fsm_run(10u);
  }

  if ((uint32_t)(now - s_t_tel) >= 100u)    /* 10Hz：CSV 遥测 */
  {
    s_t_tel = now;
    v2_telemetry_csv();
  }

  v2_tune_poll();          /* 命令解析（非阻塞，每圈都收） */
}
