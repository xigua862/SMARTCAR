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
uint16_t test_run_ms = 0u;   /* ★`TEST <秒>` 的剩余毫秒（>0 = 测试态独占主循环） */

/* ★控制循环周期实测（2026-09-23 加）：遥测 DT= 字段
   为什么要它：说明文档里怀疑"IMU 阻塞式 I2C 读会把主循环从 100Hz 拖到 20Hz 级"。
   这里实测相邻两次控制拍的真实间隔(ms)，一眼就能看出主循环有没有被拖慢。
   同时它也是"过采样有没有把一拍撑爆"的判据：DT 应该稳定在 10 附近。 */
uint16_t loop_dt_ms = 0;

static uint32_t t_ctl = 0;
static uint32_t t_tel = 0;
static uint32_t t_ui  = 0;

#if TEST_AUTO_OPENLOOP
/* ★2026-09-26 开机自动开环测试的"毫秒延时"计数器。
   主循环 1ms 一圈 → 这里减到 0 就启动测试态。见 app_loop 里的说明。 */
static uint16_t ol_delay_ms = TEST_OL_DELAY_MS;
#endif

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

  /* ★2026-09-26 开机自动开环测试（TEST_AUTO_OPENLOOP，诊断用）：
     主循环 1ms 一圈 → 这个计数器就是"毫秒延时"，等 TEST_OL_DELAY_MS 之后再启动。
     走的是下面那个测试态分支，所以不需要按键/串口指令。 */
#if TEST_AUTO_OPENLOOP
  if (ol_delay_ms > 0u)
  {
    ol_delay_ms--;
    if (ol_delay_ms == 0u)
    {
      /* ★2026-09-26 修 bug：测试态窗口必须覆盖测试自己的"静置+直行"两段。
         模式 3（不对称）内部要先静置 TEST_ASYM_WAIT_MS 再直行 TEST_ASYM_RUN_MS，
         若这里只给 TEST_OL_MS(3秒) → 静置就把窗口吃光 → 车没动就被停车
         （用户实测："小车没动"）。所以按模式取对应总时长。 */
#if TEST_SPEEDLOOP_MODE == 3
      test_run_ms = (uint16_t)TEST_ASYM_MS;
#elif TEST_SPEEDLOOP_MODE == 4
      /* ★手的判别测试分 4 段（静置/基准/转左轮/转右轮），每段 3 秒 = 12 秒。
         窗口必须给够，否则又会出现"没跑完就被停车"。 */
      test_run_ms = (uint16_t)TEST_HAND_MS;
#elif TEST_SPEEDLOOP_MODE == 5
      /* ★闭环不对称测试：静置 + 直行 */
      test_run_ms = (uint16_t)TEST_ASYM_MS;
#else
      test_run_ms = (uint16_t)TEST_OL_MS;
#endif
      telemetry_trace_reset();
      telemetry_msg("AUTO TEST start");
    }
  }
#endif

  /* ★测试态（他要求）：`TEST <秒>` → 只循迹 n 秒 → 自动停车 + 回放这 n 秒的 100Hz 轨迹。
     用途：把车放在葫芦入口前跑 1 秒，用【实测数据】定"入口标志"。
     ★2026-09-26：加了 TEST_AUTO_OPENLOOP 分支 —— 同一条路径，但下发电机的是
       【固定 PWM】而不是循迹控制器，用来单独验证"电机带载能不能跑起来"。 */
  if (test_run_ms > 0u)
  {
    if (now - t_ctl >= CTRL_PERIOD_MS)
    {
      t_ctl = now;
      speed_update();
      telemetry_trace_tick();
#if TEST_AUTO_OPENLOOP
  #if TEST_SPEEDLOOP_MODE == 1
      /* ★【闭环测试】速度 PID 单独工作，不跑循迹 PD。
         目标 = TEST_OL_PWM × 5.50 RPM（与开环同一工作点）→ 拿到"同工作点下
         开环 vs 闭环"的对照：开环平顺、闭环抽 → 问题锁死在速度环。
         第二参 = 输出上限，单独给（不是复用目标），方便只扫目标不动余量。 */
      line_follow_test_speedloop(TEST_OL_PWM, TEST_SL_MAX_PWM);
      line_follow_test_stats_tick(TEST_OL_PWM);   /* ★统计（板子上算好，只打 4 行） */
  #elif TEST_SPEEDLOOP_MODE == 2
      /* ★【正常循迹】完整控制（速度环 + 循迹 PD）—— 与按键出发后跑的是同一条路径。
         用途：编码器滤波根因修好后，用它做实车验收。 */
      line_follow_control(base_speed);
  #elif TEST_SPEEDLOOP_MODE == 3
      /* ★【左右不对称专项】真车同 PWM 直行，清里程后只报两轮各走多少 mm。
         用途：区分"左轮机械偏弱" vs "转向本来就该差速"。
         实测数据显示两轮严重不对称（左 117 / 右 155 RPM），这个测试给最终判据。 */
      line_follow_test_asym(TEST_ASYM_PWM);
  #elif TEST_SPEEDLOOP_MODE == 4
      /* ★【手的判别测试】全程不动电机：用手把两轮各转一圈，比较里程。
         用途：区分"机械阻力"(①) vs "编码器计数偏少"(②)。 */
      line_follow_test_handturn();
  #elif TEST_SPEEDLOOP_MODE == 5
      /* ★【闭环直行+不对称】验证 WHEEL_GAIN 补偿。
         模式 3 是开环、走不到补偿代码；这个走完整速度环路径。 */
      line_follow_test_asym_cl(TEST_ASYM_PWM, TEST_SL_MAX_PWM);
  #else
      /* ★【开环测试】完全绕开速度 PID 与循迹 PD，直接给两轮固定 PWM。
         这样"跑得快不快/抽不抽"只取决于 电机+电源+机械，与控制器无关。 */
      motor_set_differential(TEST_OL_PWM, TEST_OL_PWM);
      line_follow_report_pwm(TEST_OL_PWM, TEST_OL_PWM);   /* 让遥测 PWM= 显示真实值 */
  #endif
#else
      line_follow_control(base_speed);
#endif
      if (test_run_ms > (uint16_t)CTRL_PERIOD_MS)
      {
        test_run_ms = (uint16_t)(test_run_ms - CTRL_PERIOD_MS);
      }
      else
      {
        test_run_ms = 0u;
        motor_stop();
        line_follow_report_pwm(0, 0);
        test_mode = 0;
        /* ★2026-09-26 修：原来无论哪种模式都打 "OPENLOOP TEST done"，
           跑闭环测试时会误导（用户上一轮的日志就是这么被带偏的）。
           现在按模式分别报，条件也要用 == 比较（原来写 #if TEST_SPEEDLOOP_MODE
           在 MODE=2 时会被当成"真"而误报成闭环）。 */
#if TEST_SPEEDLOOP_MODE == 1
        telemetry_msg("TEST done (closed-loop)");
        line_follow_test_stats_dump();   /* ★先打摘要（4 行，短、不易丢字符） */
#elif TEST_SPEEDLOOP_MODE == 2
        telemetry_msg("TEST done (line-follow)");
#elif TEST_SPEEDLOOP_MODE == 3
        telemetry_msg("TEST done (asymmetry)");
#else
        telemetry_msg("TEST done (open-loop)");
#endif
        telemetry_trace_dump();        /* 回放这段轨迹 */
      }
    }
    return;                            /* 测试态独占主循环这一圈 */
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
