#include "car_fsm.h"
#include "app.h"
#include "app_config.h"
#include "control/line_follow.h"
#include "drivers/motor.h"
#include "drivers/led.h"
#include "drivers/buzzer.h"
#include "drivers/key.h"
#include "drivers/odom.h"       /* ★里程计: 停车时打印最终里程 */
#include "telemetry.h"          /* telemetry_msg(): 打印"为什么急停" */

#include <stdio.h>

#define CD_STEPS   3u   /* 读秒步数 = 灯数 */
#define EMG_EXIT_HOLD_MS 2000u   /* ★2026-09-22：在急停里按住 2 秒 → 回待机（不再只能断电） */

static car_state_t state   = CAR_IDLE;
static uint32_t    t_state = 0;   /* 进入当前状态的时刻 */
static uint8_t     cd_step = 0;   /* 读秒已完成步数 */
static uint32_t    emg_hold = 0;  /* 急停里按键按住的时长(ms) */
static uint8_t     emg_cause = 0; /* 0=无 1=按键长按 2=串口 X 3=其它 */

/* ★★★ 里程快照（2026-09-23 新增）★★★
   用途：把"小车跑了多少距离"打出来。目标是量出**从起点到葫芦圈出口**的里程，
        之后就能用里程做"到点强制右转"。
   打两处：
     · 进 CAR_RUN 时（计程起点，理论上应为 0）
     · 停车时（CAR_STOPPED 到站 / CAR_EMERGENCY 急停）← 这就是"按下暂停输出最后里程"
   OD   = 净位移(mm)，带符号，前进为正   ← 做距离触发用这个
   PATH = 累计路程(mm)，只加不减         ← 看总共跑了多少路（含来回蹭） */
static void print_odom(const char* tag)
{
  char buf[112];
  int n = snprintf(buf, sizeof(buf),
                   "ODOM %s: OD=%ldmm PATH=%ldmm L=%ld R=%ld\r\n",
                   tag,
                   (long)odom_distance_mm(),
                   (long)odom_path_mm(),
                   (long)odom_left_mm(),
                   (long)odom_right_mm());
  if (n > 0) telemetry_msg(buf);
}

/* 状态切换 + 进入动作 */
static void enter(car_state_t s)
{
  state   = s;
  t_state = HAL_GetTick();
  emg_hold = 0;

  /* ★★★ 2026-09-24 晚 新增：状态切换时丢弃【积压的】按键事件 ★★★
     为什么必须要有（真实的坑，不是理论问题）：
       急停里按住 2 秒解除 → enter(CAR_IDLE)。
       但这次长按产生的 long_evt **没有任何人取走**。
       而"长短按换过来"之后，待机状态的出发键【正是长按】——
       下一拍回到 `case CAR_IDLE`，`key_take_long()` 立刻读到这个残留事件
       → enter(CAR_COUNTDOWN) → **小车自己就跑起来了**。
     ★这和 4400 那一版踩的是同一个坑；换成长按出发后这个坑必然出现。
     在 enter() 里统一清一次，就彻底断掉"残留事件穿越状态"这一类问题。
     ★时序上安全：清空发生在"进入新状态的那一刻"，
       用户之后按下的按键仍然会被正常捕获。 */
  key_clear_events();

  switch (s)
  {
  case CAR_IDLE:
    led_all(0);
    break;
  case CAR_COUNTDOWN:
    led_all(0);
    cd_step = 0;
    break;
  case CAR_RUN:
    led_all(0);
    led_set(LED_ID_3, 1);          /* 运行中: 第 3 灯常亮 */
    print_odom("START");           /* ★计程起点: 理论上应为 0 */
    break;
  case CAR_STOPPED:
    buzzer_beep(600);              /* 到站: 长鸣一声 */
    telemetry_msg("STOP: arrived");
    print_odom("STOP");            /* ★到站时的最终里程 */
    break;
  case CAR_EMERGENCY:
    led_all(1);                    /* 急停: 三灯全亮 */
    if      (emg_cause == 1u) telemetry_msg("STOP by KEY(long press)");
    else if (emg_cause == 2u) telemetry_msg("STOP by SERIAL X");
    else                      telemetry_msg("STOP by OTHER");
    print_odom("STOP(EMG)");       /* ★按下暂停时的最终里程 */
    /* ★2026-09-24 急停时【自动回放最近 600 拍轨迹】(IR/GZ/OD)。
       为什么：PC→车 的串口方向不通（"只发不收"），TEST/Q 命令发不进来；
       有这一句就**只用按键取数据**：正常发车 → 跑到想看的地方 → 长按急停 1 秒
       → 车一停就把前 6 秒的 100Hz 轨迹打出来（"车→PC"方向一直是好的）。 */
    telemetry_trace_dump();
    telemetry_msg("  hold START key 2s to clear");
    break;
  default:
    break;
  }
}

void car_fsm_init(void)
{
  motor_stop();
  emg_cause = 0;
  enter(CAR_IDLE);
}

car_state_t car_fsm_state(void)
{
  return state;
}

void car_fsm_request_start(void)
{
  if (state == CAR_IDLE || state == CAR_STOPPED) enter(CAR_COUNTDOWN);
}

/* 急停：cause 0=未指明 1=按键 2=串口 3=其它 */
void car_fsm_emergency_stop_cause(uint8_t cause)
{
  emg_cause = cause;
  motor_stop();
  enter(CAR_EMERGENCY);
}

void car_fsm_emergency_stop(void)
{
  car_fsm_emergency_stop_cause(3u);
}

void car_fsm_finish(void)
{
  motor_stop();
  enter(CAR_STOPPED);
}

uint8_t car_fsm_emg_cause(void)
{
  return emg_cause;
}

/* 每控制周期调用（app_loop 里 10ms 一次；单电机测试模式时 app 层会跳过本函数） */
void car_fsm_run(uint32_t dt_ms)
{
  uint32_t now = HAL_GetTick();
  (void)dt_ms;

  /* ---- 急停状态：按住启动键 2 秒 → 解除（回待机，可重新读秒起跑）---- */
  if (state == CAR_EMERGENCY)
  {
    if (key_is_down())
    {
      emg_hold += (dt_ms ? dt_ms : CTRL_PERIOD_MS);
      if (emg_hold >= EMG_EXIT_HOLD_MS)
      {
        emg_cause = 0;
        telemetry_msg("EMERGENCY cleared -> IDLE");
        enter(CAR_IDLE);
        return;
      }
    }
    else
    {
      emg_hold = 0;
    }
  }
#if !AUTO_START_ENABLE
  else
  {
    /* ★★★ 2026-09-24 晚 用户要求：**长短按换一下** = 长按出发、短按暂停 ★★★
       （原来是"短按出发、长按急停"）

       为什么这么换不会打架：key.c 里长/短按是【天然互斥】的 ——
         长按会把 long_fired 置 1，松开时 `else if (!long_fired)` 不成立
         → 不会再补一个短按事件。所以一次按键只会产生一个事件。

       按键含义按状态分（在下面 switch 之前统一处理）：
         · 运行中 / 读秒中：**短按 = 暂停**（急停）
                            长按【也算暂停】—— 否则"按久了一点反应都没有"，
                            而且长按事件会残留在队列里，
                            等到回待机时被当成"出发"→ 小车自己跑起来。
         · 待机 / 到站    ：**短按【不做任何动作】**（只把事件吃掉）
                            —— 这两个状态没有"暂停"可言；而且摆放小车时
                            误碰一下就进急停、还要按住 2 秒才能解除，太折腾。
                            **长按不在这里消费** —— 留给下面的状态分支做"出发"。

       ⚠️ 两个 take 必须分开调用、不能写成 `a() || b()`：
          短路求值会让后一个【不被调用】→ 事件残留在队列里。 */
    if (state == CAR_RUN || state == CAR_COUNTDOWN)
    {
      uint8_t k_short = key_take_press();
      uint8_t k_long  = key_take_long();
      if (k_short || k_long)
      {
        car_fsm_emergency_stop_cause(1u);
        return;
      }
    }
    else
    {
      (void)key_take_press();     /* 待机/到站：短按吃掉、不动作 */
    }
  }
#endif

  switch (state)
  {
  case CAR_IDLE:
    motor_stop();
#if AUTO_START_ENABLE
    enter(CAR_COUNTDOWN);
#else
    /* ★2026-09-24 晚 用户要求"长短按换一下"：**长按 = 出发**（原来是短按）。
       短按在待机时已经被上面吃掉了（不动作）。 */
    /* if (key_take_press()) enter(CAR_COUNTDOWN); */   /* 旧行为：短按出发 —— 已停用 */
    if (key_take_long()) enter(CAR_COUNTDOWN);           /* 新行为：长按出发 */
#endif
    break;

  case CAR_COUNTDOWN:
  {
    motor_stop();
    uint32_t step = (now - t_state) / COUNTDOWN_STEP_MS;   /* 0,1,2,3... */

    if (step > cd_step)
    {
      uint8_t upto = (uint8_t)(step < CD_STEPS ? step : CD_STEPS);
      for (uint8_t i = 0; i < upto; i++) led_set(i, 1);    /* 逐个点亮 1→2→3 */
      buzzer_beep(COUNTDOWN_BEEP_MS);                      /* 每步"哔"一声 */
      cd_step = (uint8_t)step;
    }
    if (step >= CD_STEPS)
    {
      odom_reset();            /* ★起步瞬间清零: 之后 OD= 就是从起点算起的里程 */
      /* ★★★ 新的一趟：给"出圈右转"重新上膛（2026-09-23 加）★★★
         从起步起算的里程判据（GOURD_EXIT_ODOM_FROM_START=1）是
         "里程 >= 阈值"，一旦成立就永远成立，所以：
           · 不复位 -> 第二趟就不会再右转了；
           · 若改在"离开葫芦圈/进圈边沿"复位 -> 出圈后会被反复触发
             -> 车每隔一段就猛地右转一次，无限重复。
         所以只在【每趟起步】这一处复位。详见 line_follow.c 的
         line_follow_gourd_rearm() 与那两处 `#if !(...FROM_START)` 的说明。 */
      line_follow_gourd_rearm();
      enter(CAR_RUN);
    }
    break;
  }

  case CAR_RUN:
    /* LED2: 丢线指示（原有行为，不动） */
    if (line_follow_is_lost()) led_set(LED_ID_2, (uint8_t)((now / 100u) & 1u));
    else                       led_set(LED_ID_2, 0);

    line_follow_control(base_speed);

    /* TODO(终点): 图案 + 里程双条件成立 → car_fsm_finish() */
    break;

  case CAR_STOPPED:
    motor_stop();
    led_mask((uint8_t)(((now / 250u) & 1u) ? 0x7u : 0x0u));   /* 三灯 2Hz 闪 */
    /* ★2026-09-24 晚 用户要求"长短按换一下"：**长按 = 再跑一次**（原来是短按）。
       短按在到站时已经被上面吃掉了（不动作）。 */
    if (key_take_long()) enter(CAR_COUNTDOWN);                /* 长按再跑一次 */
    /* if (key_take_press()) enter(CAR_COUNTDOWN); */          /* 旧行为：短按再跑 —— 已停用 */
    break;

  case CAR_EMERGENCY:
    motor_stop();
    led_mask((uint8_t)(((now / 150u) & 1u) ? 0x7u : 0x0u));   /* 三灯快闪 = 急停 */
    break;

  default:
    motor_stop();
    break;
  }
}
