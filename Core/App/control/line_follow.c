#include "line_follow.h"
#include "app.h"
#include "app_config.h"
#include "drivers/motor.h"
#include "drivers/line_sensor.h"
#include "control/filter.h"
#if USE_IMU
#include "drivers/imu.h"
#endif
#if USE_SPEED_LOOP
#include "control/pid.h"
#include "drivers/speed.h"
#endif

/* 丢线/上次误差状态 */
static int8_t   last_error  = 0;
static int8_t   last_e      = 0;   /* 上一次误差(PD 微分用) */
static uint16_t lost_cycles = 0;   /* ★uint8 会在 256 拍回绕, 把"停车防跑飞"抵消 */
static uint8_t  line_lost   = 0;

/* 宽图案判定状态（十字 / 圆出口 / 交叉） */
static uint8_t  wide_cnt    = 0;   /* 宽图案持续拍数 */
static uint8_t  on_cross    = 0;   /* 1 = 判定为十字（直行通过） */
static uint8_t  wide_long   = 0;   /* 1 = 宽图案持续过久 → 不是十字（圆出口/直角入口） */

/* ★直线提速（2026-09-22） */
static uint16_t straight_cycles = 0;   /* 连续"直线"拍数 */
static uint8_t  boost_active    = 0;   /* 1 = 已进入提速档 */

/* ★葫芦弯相切点状态机（2026-09-22 他提的方案） */
static uint8_t  gourd_latch  = 0;      /* 分离锁存：一段分离只记 1 次 */
static uint8_t  gourd_waves  = 0;      /* 已识别的相切点个数（到 3 清零 = 绕完一圈） */
#if GOURD_USE_FLIP
static uint8_t  gourd_flip   = 0;      /* "翻转修正"剩余拍数（仅 GOURD_USE_FLIP=1 时用） */
static int8_t   gourd_dir    = 1;      /* 进相切点前的误差方向 */
#endif

#if USE_D_FILTER
static lpf_t d_lpf;
#endif

#if USE_SPEED_LOOP
static pid_t spd_pid[2];   /* 左右轮速度环(编码器闭环) */
static float  tgt_rpm[2] = {0, 0};
#endif

void line_follow_control(int16_t base)
{
  line_reading_t r = line_read();
  int8_t e = r.error;

  /* ---- ★葫芦弯相切点检测（判据按 2026-09-22 实测数据定）----
     实测：相切点处传感条看到"图案分裂成两组、往两边分离"
        00110011 (3,4 + 7,8) → 11000011 (1,2 + 7,8) → 00000011 (只剩右2)
     而普通循迹是"连续一小片"(00011000 / 00011100)，
     十字是"一大片连着亮"(11111111, >=5 路) —— 三种特征天然分得开。
     规则：最左活跃 L 与最右活跃 R 之间空 >=2 路 → 判"分离"；分离期间只记 1 次（锁存）。*/
#if USE_GOURD_SM
  {
    uint8_t L = 0xFFu, R = 0u, cnt = 0u;
    for (uint8_t i = 0; i < LINE_CHANNELS; i++)
    {
      if (r.raw & (uint16_t)(1u << i)) { if (L == 0xFFu) L = i; R = i; cnt++; }
    }
    uint8_t separated = 0u;
    if ((L != 0xFFu) && (cnt >= 2u))
    {
      uint8_t span = (uint8_t)(R - L + 1u);
      if ((uint8_t)(span - cnt) >= 2u) separated = 1u;   /* 组内空隙 >=2 路 = 两组 */
    }
    if (separated)
    {
      if (!gourd_latch)                     /* 上升沿：一个相切点只记一次 */
      {
        gourd_latch = 1u;
#if GOURD_USE_FLIP
        gourd_dir   = (last_error >= 0) ? 1 : -1;
        gourd_flip  = GOURD_FLIP_CYCLES;
#else
        (void)last_error;      /* 翻转关掉时这两个变量不用 */
#endif
        if (gourd_waves < 200u) gourd_waves++;
        if (gourd_waves >= GOURD_TOTAL) gourd_waves = 0;   /* 3 个相切点 = 绕完 4 个圆 */
      }
    }
    else
    {
      gourd_latch = 0u;
    }
  }
#endif

  /* ---- 速度自适应: 直道快/弯道慢（目标速度是运行时变量, S 指令能同步改） ---- */
  int16_t sp = base;
#if USE_ADAPTIVE_SPEED
  if (r.active == 0)
  {
    sp = sp_curve;                    /* 丢线先减速 */
  }
  else if (e >= CURVE_ABS_ERR_THRESH || e <= -CURVE_ABS_ERR_THRESH)
  {
    sp = sp_curve;                    /* 误差大 = 弯道, 减速 */
  }
  else
  {
    sp = sp_straight;                 /* 直道 */
  }
#endif

  /* ---- ★宽图案判定 ---- 真十字: 25mm 横线扫过 → 只持续几拍
     葫芦弯出口/直角入口: "圆切线+垂直线" → 持续更久 → 不能直行 */
  if (r.active >= CROSS_ACTIVE_MIN)
  {
    if (wide_cnt < 200u) wide_cnt++;
  }
  else
  {
    wide_cnt = 0;
  }
  on_cross  = (wide_cnt >= CROSS_CONFIRM_CNT) && (wide_cnt <= CROSS_MAX_FRAMES);
  wide_long = (wide_cnt > CROSS_MAX_FRAMES);

  /* ---- ★直线提速：连续"直线"够久 → 直接冲到 99（慢起快落）---- */
#if USE_STRAIGHT_BOOST
  if ((r.active > 0u) && (e <= STRAIGHT_BOOST_ERR_MAX) && (e >= -STRAIGHT_BOOST_ERR_MAX)
      && (!on_cross) && (!wide_long) && (!line_lost))
  {
    if (straight_cycles < 60000u) straight_cycles++;
  }
  else
  {
    straight_cycles = 0;
    boost_active = 0;                 /* 误差一大立刻回落 */
  }
  if (straight_cycles >= STRAIGHT_BOOST_DELAY)
  {
    /* ★斜坡提速：每拍最多涨 STRAIGHT_BOOST_RAMP，避免瞬跳（原版直接跳 99）*/
    int16_t tgt = STRAIGHT_BOOST_SPEED;
    if (sp < tgt)
    {
      sp = (int16_t)(sp + STRAIGHT_BOOST_RAMP);
      if (sp > tgt) sp = tgt;
    }
    else
    {
      sp = tgt;
    }
    boost_active = 1;
  }
#endif

  /* ---- 丢线处理 ---- */
  if (r.active == 0)
  {
    lost_cycles++;
    if (lost_cycles <= LOST_SPIN_DELAY)
    {
      line_lost = 0;
      e = (int8_t)(last_error * 2);   /* 短暂丢线: 沿上次方向加强修正 */
#if USE_GOURD_SM
#if GOURD_USE_FLIP
      if (gourd_flip) e = (int8_t)(-gourd_dir * GOURD_FLIP_ERR);
#endif
#endif
    }
    else
    {
      line_lost = 1;                  /* 持续丢线: 原地旋转找线 */
      e = 0;
    }
  }
  else
  {
    last_error = r.error;
    lost_cycles = 0;
    line_lost = 0;
  }

  /* ---- 持续丢线: 原地旋转找线（超时停车防跑飞）---- */
  if (line_lost)
  {
    if (lost_cycles > LOST_MAX_CYCLES)
    {
      lost_cycles = (uint16_t)(LOST_MAX_CYCLES + 1);   /* 钉死，别让计数继续涨 */
      motor_stop();
    }
    else if (last_error >= 0)
    {
      motor_set_differential(LOST_SPIN_SPEED, -LOST_SPIN_SPEED);
    }
    else
    {
      motor_set_differential(-LOST_SPIN_SPEED, LOST_SPIN_SPEED);
    }
    return;
  }

  /* ---- 真十字: 强制直行（清 last_e，防出十字瞬间 D 项踢一脚）---- */
  if (on_cross)
  {
    e = 0;
    last_e = 0;
  }
  /* ---- 宽图案持续过久 → 不是十字（葫芦出口/直角入口）→ 接着转 ---- */
  else if (wide_long)
  {
    e = (int8_t)(last_error * 2);
#if USE_GOURD_SM
#if GOURD_USE_FLIP
    if (gourd_flip) e = (int8_t)(-gourd_dir * GOURD_FLIP_ERR);
#endif
#endif
  }

  /* ---- PD 差速校正 ---- */
#if USE_D_FILTER
  e = (int8_t)lpf_update(&d_lpf, (float)e);   /* 对误差低通, 压 D 项高频噪声 */
#endif
  int16_t d_term = (int16_t)(e - last_e);
  int16_t corr   = (int16_t)((kp_x10 * e) / 10 + (kd_x10 * d_term) / 10);
  last_e = e;
  if (corr >  99) corr =  99;
  if (corr < -99) corr = -99;

  int16_t m1 = sp + corr;         /* 左轮: 线偏右时加速 */
  int16_t m2 = sp - corr;         /* 右轮: 线偏右时减速 */
  if (m1 >  99) m1 =  99;
  if (m1 < -99) m1 = -99;
  if (m2 >  99) m2 =  99;
  if (m2 < -99) m2 = -99;

#if USE_SPEED_LOOP
  tgt_rpm[0] = (float)m1;
  tgt_rpm[1] = (float)m2;
  float dt = (float)CTRL_PERIOD_MS / 1000.0f;
  m1 = (int16_t)pid_update(&spd_pid[0], tgt_rpm[0], (float)speed_get_rpm(MOTOR_LEFT),  dt);
  m2 = (int16_t)pid_update(&spd_pid[1], tgt_rpm[1], (float)speed_get_rpm(MOTOR_RIGHT), dt);
  if (m1 >  99) m1 =  99; if (m1 < -99) m1 = -99;
  if (m2 >  99) m2 =  99; if (m2 < -99) m2 = -99;
#endif

  motor_set_differential(m1, m2);

#if USE_GOURD_SM && GOURD_USE_FLIP
  if (gourd_flip) gourd_flip--;   /* 翻转窗口倒计时 */
#endif
}

void line_follow_stop(void)
{
  motor_stop();
}

uint8_t line_follow_is_lost(void)
{
  return line_lost;
}

uint8_t line_follow_on_cross(void)
{
  return on_cross;
}

uint8_t line_follow_gourd_waves(void)
{
  return gourd_waves;
}

uint8_t line_follow_boost_active(void)
{
  return boost_active;
}

void line_follow_init(void)
{
  last_error  = 0;
  last_e      = 0;
  lost_cycles = 0;
  line_lost   = 0;
  wide_cnt    = 0;
  on_cross    = 0;
  wide_long   = 0;
  straight_cycles = 0;
  boost_active = 0;
  gourd_latch  = 0;
  gourd_waves  = 0;
  gourd_flip   = 0;
  gourd_dir    = 1;
#if USE_D_FILTER
  lpf_init(&d_lpf, D_FILTER_ALPHA);
#endif
#if USE_SPEED_LOOP
  pid_init(&spd_pid[0], 0.0f, 0.0f, 0.0f, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, 50.0f);
  pid_init(&spd_pid[1], 0.0f, 0.0f, 0.0f, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, 50.0f);
#endif
}
