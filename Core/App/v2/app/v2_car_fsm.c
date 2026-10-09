#include "v2_car_fsm.h"
#include "v2_key.h"
#include "v2_motor.h"
#include "v2_led.h"
#include "v2_buzzer.h"
#include "v2_uart.h"

static v2_car_state_t s_state = V2_CAR_IDLE;

void v2_car_fsm_init(void)
{
  s_state = V2_CAR_IDLE;
}

void v2_car_fsm_run(uint32_t dt_ms)
{
  (void)dt_ms;

  /* M0：按键只做提示/急停，不做发车（发车流程 M3 再写） */
  if (v2_key_take_long())
  {
    v2_motor_stop();
    s_state = V2_CAR_EMERGENCY;
    v2_led_all(1u);
    v2_buzzer_beep(300);
    v2_uart_puts("STOP by KEY long press\r\n");
    v2_key_clear_events();
    return;
  }

  if (v2_key_take_press())
  {
    v2_buzzer_beep(60);      /* IDLE 下短按 = 一声提示（证明按键链路通） */
  }
}

v2_car_state_t v2_car_fsm_state(void)
{
  return s_state;
}
