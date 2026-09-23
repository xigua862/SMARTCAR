#include "line_follow.h"
#include "app.h"
#include "app_config.h"
#include "drivers/motor.h"
#include "drivers/line_sensor.h"
#include "control/filter.h"
#include "telemetry.h"        /* ★事件打印用 telemetry_msg("出葫芦弯道" 等) */
#include <stdio.h>            /* snprintf: 拼"十字/分支口"事件行 */
#if USE_IMU
#include "drivers/imu.h"
#endif
#if USE_SPEED_LOOP
#include "control/pid.h"
#include "drivers/speed.h"
#endif

/* ============================================================================
 * ★决策优先级（谁覆盖谁）—— 2026-09-23 补，改之前先读这段
 *
 * line_follow_control() 里跑了 9 层判断，顺序就是优先级；两个地方会提前 return：
 *
 *   1. 葫芦"图案分裂"检测        （只记事件，不直接控制）
 *   2. G7 最右一路上升沿计数      （只记事件）
 *   3. G7 硬转 → 直接 return     ★最高优先级：闭眼转，忽略传感器
 *   4. 速度自适应（直道/弯道/丢线）
 *   5. 十字：形状判据 + 预算层 + 事件防抖（只记事件/判定标志）
 *   6. 直线提速（斜坡）
 *   7. 丢线兜底 → 直接 return    ★次高：加强修正 / 原地旋转 / 超时停车
 *   8. on_cross 强制直行(e=0)  /  wide_long 继续转(e=last_error*2)
 *   9. PD + 低通 + 差速输出 → motor_set_differential()
 *
 * ⚠️ 已知耦合（别踩）：第 3 步会写死 last_error = 3（给退出后的 PD 铺路），
 *    而 last_error 还被下游两处用着 —— 丢线的 last_error*2、葫芦的 gourd_dir。
 *    也就是说"硬转过一次"会**污染**这两处的判断。目前 GOURD_USE_FLIP=0 影响有限，
 *    但以后要打开翻转、或改丢线修正之前，先回来想清楚这个耦合。
 *
 * ⚠️ 历史教训：2026-09-22 那 10 个提交里出现过 2 次"编译不过/加了又回退"，
 *    都是这类顺序/开关范围不同步造成的。加层时**先想清楚它插在哪一层、要不要 return**。
 * ==========================================================================*/

/* 丢线/上次误差状态 */
static int8_t   last_error  = 0;
static int8_t   last_e      = 0;   /* 上一次误差(PD 微分用) */
static uint16_t lost_cycles = 0;   /* ★uint8 会在 256 拍回绕, 把"停车防跑飞"抵消 */
static uint8_t  line_lost   = 0;

/* 宽图案判定状态（十字 / 圆出口 / 交叉） */
static uint8_t  wide_cnt    = 0;   /* 宽图案持续拍数 */
static uint8_t  on_cross    = 0;   /* 1 = 判定为十字（直行通过） */
static uint8_t  wide_long   = 0;   /* 1 = 宽图案持续过久 → 不是十字（圆出口/直角入口） */
/* ★2026-09-22 15:31 十字判据修正 + 事件计数/打印 */
static uint16_t cross_events = 0;  /* 一趟里"判定为十字"的次数（遥测 CX=） */
static uint16_t branch_events = 0; /* 一趟里"宽图案只贴一端 → 拒绝当十字"的次数（遥测 BR=） */
static uint8_t  one_end_prev = 0;  /* 上一拍是否"只贴一端的宽图案"（边沿，防刷屏） */
static uint8_t  det_prev     = 0;  /* 上一拍是否"落在十字时间窗"（边沿，给预算计数用） */
static uint8_t  wide_gap     = 0;  /* 宽图案已连续消失多少拍（★事件计数防抖，见 app_config） */
static uint8_t  wide_armed   = 1;  /* 本段宽图案"可否计数"：★在段首那一拍判定并锁存整段 */

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

/* ★弯道计数 → 强制右转（2026-09-22 他提的方案）
   ★这几个变量【无条件声明】（不放进 #if）—— 2026-09-22 那次"宏开关漏罩 init"的编译事故
   就是 #if 范围不一致造成的，这里一律全声明、只在真正使用的地方 #if。 */
static uint16_t g7_count   = 0;   /* 最右一路(bit7)累计触发次数（上升沿计数） */
static uint8_t  g7_prev    = 0;   /* 上一拍 bit7 电平（边沿检测用） */
static uint8_t  g7_lock    = 0;   /* 计数锁：>0 时不许再计（防抖 / 防一次点亮算多次） */
#if USE_G7_COUNT && USE_G7_TURN
/* ★这个变量只在"硬转开着"时才存在：声明/写/读必须在同一个 #if 条件下，
   否则关掉硬转时会出现 "declared but never referenced" / "set but never used" 告警。 */
static uint8_t  g7_waitmsg = 0;   /* "数够但位置不对"只打印一次 */
#endif
static uint8_t  g7_flag    = 0;   /* 1 = 已触发过"出葫芦弯道"（锁存，给遥测显示） */
static uint8_t  g7_turn    = 0;   /* 硬转剩余拍数（>0 = 正在硬转，期间忽略传感器） */

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

  /* ---- ★弯道计数：数"最右一路"(bit7)的【上升沿】次数（2026-09-22 他提的方案）----
     边沿计数 + 锁存间隔：一次持续点亮只算 1 次，且不会被 10ms 拍/速度绑死。
     遥测 G7= 就是它（跑一圈看它在葫芦出口读到多少 → 用它改 G7_TURN_TRIG）。 */
#if USE_G7_COUNT
  {
    uint8_t b7 = (uint8_t)((r.raw >> (LINE_CHANNELS - 1u)) & 1u);   /* bit7 = 最右一路 */
    if (g7_lock) g7_lock--;
    if (b7 && !g7_prev && (g7_lock == 0u))          /* 暗→亮 = 一次触发 */
    {
      if (g7_count < 60000u) g7_count++;
      g7_lock = G7_LOCKOUT;
    }
    g7_prev = b7;
  }
#endif

  /* ---- ★数到阈值 → 强制右转（默认先等"分支口"确认，见 G7_TURN_GATE_BRANCH）---- */
#if USE_G7_COUNT && USE_G7_TURN
  if ((g7_turn == 0u) && (g7_count >= G7_TURN_TRIG))
  {
    uint8_t fire = 1u;
#if G7_TURN_GATE_BRANCH
    /* 分支口 = 宽图案(>=G7_GATE_WIDE_MIN 路) 且 只贴一端（另一端空）= 葫芦出口/直角入口 */
    uint8_t L = 0xFFu, R = 0u, cnt = 0u;
    for (uint8_t i = 0; i < LINE_CHANNELS; i++)
    {
      if (r.raw & (uint16_t)(1u << i)) { if (L == 0xFFu) L = i; R = i; cnt++; }
    }
    if ((L == 0xFFu) || (cnt < G7_GATE_WIDE_MIN))  fire = 0u;
    else
    {
      uint8_t tL = (L == 0u) ? 1u : 0u;
      uint8_t tR = (R == (LINE_CHANNELS - 1u)) ? 1u : 0u;
      fire = (tL != tR) ? 1u : 0u;                 /* 只贴一端才算分支口 */
    }
#endif
    if (fire)
    {
      g7_turn    = G7_TURN_FRAMES;
      g7_count   = 0u;                             /* 触发后清零，重新开始数下一段 */
      g7_waitmsg = 0u;
      g7_flag    = 1u;
      telemetry_msg("GOURD EXIT (G7 count) -> FORCE RIGHT");
    }
    else if (!g7_waitmsg)
    {
      /* 数够了、但这一刻不是分支口 → 不转（防直道上拐出去），等分支口出现那一拍再转。
         这条打印是给你看的：出现它说明"阈值 12 和出口的位置对不上"，该改 G7_TURN_TRIG。 */
      g7_waitmsg = 1u;
      telemetry_msg("G7 HIT but not BRANCH (wait)");
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
     葫芦弯出口/直角入口: "圆切线+垂直线" → 持续更久 → 不能直行
     ★2026-09-22 15:31 修根因：十字还必须【两端都贴住】(CROSS_NEED_BOTH_ENDS) */
  {
    uint8_t wide    = (r.active >= CROSS_ACTIVE_MIN) ? 1u : 0u;
    uint8_t one_end = 0u;
#if CROSS_NEED_BOTH_ENDS
    if (wide)
    {
      uint8_t b0 = (uint8_t)( r.raw                  & 1u);   /* 最左一路 */
      uint8_t b7 = (uint8_t)((r.raw >> (LINE_CHANNELS - 1u)) & 1u);   /* 最右一路 */
      if (!(b0 && b7))
      {
        wide    = 0u;      /* 只贴一端 = 分支口/圆出口，不是十字 → 不直行，交给 PD 转 */
        one_end = 1u;
      }
    }
#endif
    /* ★2026-09-22 16:07 事件计数防抖（实跑：第 4 次真十字被判成假十字 → 车转弯）
       问题的形状：一次过十字若图案中间抖掉一帧，wide_cnt 归零 → 又冒出一个"上升沿"
       → 同一次过十字被数成 2 次 → 预算提前用完，把第 4 次真十字也拦了。
       ⚠️ 判定时机很讲究：必须在"宽图案段的**第一拍**"判 gap 并**锁存整段**。
         因为真正的上升沿出现在宽图案的第 2 拍（wide_cnt 到 CROSS_CONFIRM_CNT 才成事件），
         而第 1 拍已把 wide_gap 清零 —— 若等上升沿那拍才去看 gap，读到的永远是 0，
         会把**所有**事件一起按掉。（我第一次就写错了，靠模拟跑出来才发现。） */
    if (wide)
    {
      if (wide_cnt == 0u)                                    /* 段首：判定 + 锁存整段 */
        wide_armed = (wide_gap >= CROSS_REARM_FRAMES) ? 1u : 0u;
      if (wide_cnt < 200u) wide_cnt++;
      wide_gap = 0;
    }
    else
    {
      wide_cnt = 0;
      if (wide_gap < 250u) wide_gap++;
    }

    uint8_t detected = ((wide_cnt >= CROSS_CONFIRM_CNT) && (wide_cnt <= CROSS_MAX_FRAMES)) ? 1u : 0u;
    wide_long = (wide_cnt > CROSS_MAX_FRAMES) ? 1u : 0u;

    /* 每个"宽图案落在十字时间窗"的事件记一次（上升沿），供预算层比较。
       raw_rising 必须在 `det_prev = detected;` **之前**取 —— 否则编译器会发现
       `detected && !det_prev` 恒假，把下面"dup"那条打印优化掉（已验证过）。 */
    uint8_t raw_rising = (detected && !det_prev) ? 1u : 0u;    /* 上升沿（还没过防抖） */
    uint8_t det_rising = (raw_rising && wide_armed) ? 1u : 0u; /* 过了防抖才算真事件 */
    if (det_rising && (cross_events < 60000u)) cross_events++;
    det_prev = detected;

    uint8_t cross_now = detected;
#if USE_CROSS_BUDGET
    /* ★预算层：真十字触发次数有限（本赛道 = 2 个十字 × 2 次 = 4）→ 第 5 次起一律判为假十字
       （葫芦的假触发），交回 PD 转，不再强制直行。见 app_config.h 的说明与前提。 */
    if (cross_now && (cross_events > CROSS_BUDGET_MAX)) cross_now = 0u;
#endif

#if CROSS_PRINT
    char msg_cross[112];
    if (det_rising)                              /* 真事件打一行（上升沿，不刷屏） */
    {
      char irs[LINE_CHANNELS + 1];
      for (uint8_t i = 0; i < LINE_CHANNELS; i++) irs[i] = (r.raw >> i) & 1u ? '1' : '0';
      irs[LINE_CHANNELS] = '\0';
      if (cross_now)
      {
        snprintf(msg_cross, sizeof(msg_cross), "CROSS #%d IR=%s e=%+d wide=%d (straight)",
                 (int)cross_events, irs, (int)r.error, (int)wide_cnt);
      }
      else
      {
        snprintf(msg_cross, sizeof(msg_cross), "CROSS #%d IR=%s e=%+d BLOCKED(budget %d) -> turn",
                 (int)cross_events, irs, (int)r.error, (int)CROSS_BUDGET_MAX);
      }
      telemetry_msg(msg_cross);
    }
    else if (raw_rising)                         /* ★被防抖按下的重复事件 → 也打一行 */
    {
      char irs[LINE_CHANNELS + 1];
      for (uint8_t i = 0; i < LINE_CHANNELS; i++) irs[i] = (r.raw >> i) & 1u ? '1' : '0';
      irs[LINE_CHANNELS] = '\0';
      snprintf(msg_cross, sizeof(msg_cross), "CROSS dup IR=%s gap=%d (<%d) -> ignored",
               irs, (int)wide_gap, (int)CROSS_REARM_FRAMES);
      telemetry_msg(msg_cross);
    }
    if (one_end && !one_end_prev)                /* 宽图案只贴一端 → 形状层拦下了 → 打一行 */
    {
      char irs[LINE_CHANNELS + 1];
      for (uint8_t i = 0; i < LINE_CHANNELS; i++) irs[i] = (r.raw >> i) & 1u ? '1' : '0';
      irs[LINE_CHANNELS] = '\0';
      if (branch_events < 60000u) branch_events++;
      snprintf(msg_cross, sizeof(msg_cross), "BRANCH #%d IR=%s e=%+d -> NOT cross (turn)",
               (int)branch_events, irs, (int)r.error);
      telemetry_msg(msg_cross);
    }
#endif
    one_end_prev = one_end;
    on_cross     = cross_now;
  }

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

  /* ---- ★强制右转提交：硬转期间【闭着眼转】，忽略传感器，转完再交回 PD ----
     优先级高于丢线/PD：故意急转的时候传感器误差是误导的（会把自己拽回来 → 来回摆）。 */
#if USE_G7_COUNT && USE_G7_TURN
  if (g7_turn)
  {
    g7_turn--;
    int16_t a = (int16_t)(sp_curve + G7_TURN_PWM);   /* 左快右慢 = 右转 */
    int16_t b = (int16_t)(sp_curve - G7_TURN_PWM);
    if (a >  99) a =  99;  if (a < -99) a = -99;
    if (b >  99) b =  99;  if (b < -99) b = -99;
    motor_set_differential(a, b);
    last_error  = 3;                 /* 给退出后的 PD 一个合理起点（偏右） */
    lost_cycles = 0;
    line_lost   = 0;
    return;
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

/* ★弯道计数（2026-09-22 他提的方案）—— 这三个无条件提供，遥测/状态机直接用 */
uint16_t line_follow_g7_count(void)
{
  return g7_count;      /* 最右一路已触发次数（实时，跑一圈看它在出口读到多少） */
}

uint8_t line_follow_g7_flag(void)
{
  return g7_flag;       /* 1 = 触发过"出葫芦弯道"（锁存，看到它说明真的触发了） */
}

uint8_t line_follow_turn_left(void)
{
  return g7_turn;       /* 硬转剩余拍数（>0 = 正在硬转） */
}

/* ★十字事件计数（2026-09-22 15:31）—— 跑一圈看 CX= 就知道一趟里被判了几次十字 */
uint16_t line_follow_cross_events(void)
{
  return cross_events;
}

/* ★"宽图案只贴一端 → 拒绝当十字"的次数（证明"出口不再被误当十字"的修复在起作用） */
uint16_t line_follow_branch_events(void)
{
  return branch_events;
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
#if GOURD_USE_FLIP
  gourd_flip   = 0;
  gourd_dir    = 1;
#endif
  g7_count   = 0;      /* ★弯道计数（无条件复位，别放进 #if —— 上次编译事故就是这么来的） */
  g7_prev    = 0;
  g7_lock    = 0;
#if USE_G7_COUNT && USE_G7_TURN
  g7_waitmsg = 0;      /* 只有开启硬转时才被读（关掉时不写，否则告警 set-but-never-used） */
#endif
  g7_flag    = 0;
  g7_turn    = 0;
  cross_events  = 0;   /* ★十字事件计数（2026-09-22 15:31） */
  branch_events = 0;
  one_end_prev  = 0;
  det_prev      = 0;
  wide_gap      = 0;   /* ★事件计数防抖（2026-09-22 16:07） */
  wide_armed    = 1;
#if USE_D_FILTER
  lpf_init(&d_lpf, D_FILTER_ALPHA);
#endif
#if USE_SPEED_LOOP
  pid_init(&spd_pid[0], 0.0f, 0.0f, 0.0f, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, 50.0f);
  pid_init(&spd_pid[1], 0.0f, 0.0f, 0.0f, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, 50.0f);
#endif
}
