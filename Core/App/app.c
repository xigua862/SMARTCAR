#include "app.h"
#include "drivers/motor.h"
#include "drivers/line_sensor.h"
#include "drivers/speed.h"
#include "drivers/odom.h"        /* ★里程计: 量"跑了多少距离" */
#include "drivers/imu.h"          /* USE_IMU 开关 */
#include "drivers/led.h"
#include "drivers/buzzer.h"
#include "drivers/key.h"
#include "control/line_follow.h"
#include "fsm/car_fsm.h"
#include "telemetry.h"

/* ---- 运行时参数(在线可改), 供 line_follow / telemetry 共享 ---- */
int16_t kp_x10      = KP * 10;
int16_t kd_x10      = KD * 10;
int16_t base_speed  = BASE_SPEED;
int16_t sp_straight = SPEED_STRAIGHT;
int16_t sp_curve    = SPEED_CURVE;
uint8_t test_mode   = 0;

/* ★控制循环周期实测（2026-09-23 加）：遥测 DT= 字段
   为什么要它：说明文档里怀疑"IMU 阻塞式 I2C 读会把主循环从 100Hz 拖到 20Hz 级"。
   这里实测相邻两次控制拍的真实间隔(ms)，一眼就能看出主循环有没有被拖慢。
   同时它也是"过采样有没有把一拍撑爆"的判据：DT 应该稳定在 10 附近。 */
uint16_t loop_dt_ms = 0;

static uint32_t t_ctl = 0;
static uint32_t t_tel = 0;
static uint32_t t_ui  = 0;

/* 顶层初始化: 每个外设/模块按需初始化（顺序：硬件输出 → 传感器 → 人机 → 状态机 → 遥测） */
void app_init(void)
{
  motor_init();              /* 双路 PWM 输出 + 方向脚复位 */
  led_init();                /* 三灯全灭(高电平点亮, 见 app_config) */
  buzzer_init();             /* 蜂鸣器 PWM 启动(占空比 0 = 静音) */
  key_init();                /* 启动键(PB9)消抖状态复位 */
  line_sensor_init();        /* 循迹(引脚由 CubeMX gpio.c 配置) */
  speed_init();              /* 测速: 编码器(USE_ENCODER) / 霍尔(退役) */
  odom_init();               /* ★里程计复位(必须在 speed_init 之后: encoder 已启动) */
  imu_init();                /* MPU6050: 唤醒 + 器件自检 + 设置量程 */
  if (imu_is_present()) imu_calibrate();   /* 静止求陀螺零偏(开机一次, 约几十 ms) */
  car_fsm_init();            /* 状态机复位到 IDLE(待机等按键) */
  line_follow_init();        /* 复位循迹内部状态 */
  telemetry_init();          /* 串口在线调参: 使能 USART1 RXNE 中断 */
  telemetry_banner();        /* 上电横幅: 版本/编译时间/参数 → 一眼确认烧的是哪版 */
}

/* 顶层调度: 主循环每圈调用, 非阻塞。 */
void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  /* 人机节拍: 按键消抖 + 蜂鸣器计时(10ms) */
  if (now - t_ui >= UI_PERIOD_MS)
  {
    t_ui = now;
    key_update();
    buzzer_update();
    imu_update();            /* 读陀螺 Z 偏航率(给循迹用) */
    odom_update();           /* ★里程累加(与 UI 同节拍 = 10ms, 60ms 内就能发现 16 位回绕) */
  }

  /* 控制节拍：测速 + 状态机驱动循迹（单电机测试时，状态机让位给 T/M 指令） */
  if (now - t_ctl >= CTRL_PERIOD_MS)
  {
    uint32_t prev = t_ctl;
    t_ctl = now;
    if (prev != 0u) loop_dt_ms = (uint16_t)(now - prev);   /* ★实测周期, 遥测 DT= */

    /* ★2026-09-23 晚，修正闭环的一个致命细节：
       原来 speed_update() 【只在 telemetry_report() 里】被调用（全工程唯一调用点）→
       闭环虽然每 10ms 跑一次，却只能读到"50ms 前的旧转速"，**连续 5 拍用同一个值**。
       实测症状：闭环能把转速稳在目标附近，但稳态差 2~7%、修正很慢。
       现在挪到控制拍里（10ms 一次）→ 反馈新鲜；遥测只是把这个值读走打印。 */
    speed_update();
    telemetry_trace_tick();   /* ★2026-09-24 轨迹黑匣子: 按控制拍记录最近 600 拍(IR/GZ/OD) */

    if (test_mode == 0) car_fsm_run((uint32_t)CTRL_PERIOD_MS);
  }

  /* 测速 + 遥测打印：待机 500ms 一行；读秒/运行中 50ms 一行(20Hz, 看循迹过程) */
  {
    car_state_t st = car_fsm_state();
    uint32_t tel_period = (st == CAR_RUN || st == CAR_COUNTDOWN)
                          ? TELEMETRY_PERIOD_RUN_MS : TELEMETRY_PERIOD_MS;
    if (now - t_tel >= tel_period)
    {
      t_tel = now;
      telemetry_report();
    }
  }

  /* 串口命令立即处理(不受控制周期绑定) */
  telemetry_process_command();
}
