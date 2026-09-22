#include "car_fsm.h"
#include "app.h"
#include "app_config.h"
#include "control/line_follow.h"
#include "drivers/motor.h"
#include "drivers/led.h"
#include "drivers/buzzer.h"
#include "drivers/key.h"

#define CD_STEPS   3u   /* 读秒步数 = 灯数 */

static car_state_t state   = CAR_IDLE;
static uint32_t    t_state = 0;   /* 进入当前状态的时刻 */
static uint8_t     cd_step = 0;   /* 读秒已完成步数 */

/* 状态切换 + 进入动作 */
static void enter(car_state_t s)
{
  state   = s;
  t_state = HAL_GetTick();

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
    break;
  case CAR_STOPPED:
    buzzer_beep(600);              /* 到站: 长鸣一声(闪灯在 run 里做) */
    break;
  case CAR_EMERGENCY:
    led_all(1);                    /* 急停: 三灯全亮 */
    break;
  default:
    break;
  }
}

void car_fsm_init(void)
{
  motor_stop();
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

void car_fsm_emergency_stop(void)
{
  motor_stop();
  enter(CAR_EMERGENCY);
}

void car_fsm_finish(void)
{
  motor_stop();
  enter(CAR_STOPPED);
}

/* 每控制周期调用（app_loop 里 10ms 一次；单电机测试模式时 app 层会跳过本函数） */
void car_fsm_run(uint32_t dt_ms)
{
  uint32_t now = HAL_GetTick();
  (void)dt_ms;

#if !AUTO_START_ENABLE
  /* 长按按键 = 急停（调车保险；比赛禁止碰车，所以只作兜底）
     ★临时停用：老核心板 PB9 坏 → 按键不可用。新板子到了把 app_config 的
       AUTO_START_ENABLE 改回 0，这段自动恢复 */
  if (key_take_long() && (state != CAR_EMERGENCY))
  {
    car_fsm_emergency_stop();
    return;
  }
#endif

  switch (state)
  {
  case CAR_IDLE:
    motor_stop();
#if AUTO_START_ENABLE
    /* ★测试模式（临时）：上电自动进读秒（3 步：灯逐个亮 + 每步一哔）→ 开跑。
       老核心板 PB9 坏、按键用不了；新板子到了把 AUTO_START_ENABLE 改回 0，
       下面 #else 里的"按键启动"原样恢复。 */
    enter(CAR_COUNTDOWN);
#else
    if (key_take_press()) enter(CAR_COUNTDOWN);      /* 按键 → 开始读秒 */
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
    if (step >= CD_STEPS) enter(CAR_RUN);
    break;
  }

  case CAR_RUN:
    /* 丢线指示: 第 2 灯闪 */
    if (line_follow_is_lost()) led_set(LED_ID_2, (uint8_t)((now / 100u) & 1u));
    else                       led_set(LED_ID_2, 0);

    line_follow_control(base_speed);

    /* TODO(终点, 按他要求最后写): 图案 + 里程双条件成立 → car_fsm_finish() */
    break;

  case CAR_STOPPED:
    motor_stop();
    led_mask((uint8_t)(((now / 250u) & 1u) ? 0x7u : 0x0u));   /* 三灯 2Hz 闪 */
    break;

  case CAR_EMERGENCY:
    motor_stop();
    led_mask((uint8_t)(((now / 150u) & 1u) ? 0x7u : 0x0u));
    break;

  default:
    motor_stop();
    break;
  }
}
