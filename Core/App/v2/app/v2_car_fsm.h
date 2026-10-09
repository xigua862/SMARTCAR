#ifndef V2_CAR_FSM_H
#define V2_CAR_FSM_H

#include <stdint.h>

/* v2 状态机（冻结@M0）：M0 只实现 IDLE + 长按急停（安全兜底）
   完整态（COUNTDOWN/RUN/STOPPED/终点停车）在 M3 落地 */
typedef enum {
  V2_CAR_IDLE = 0,
  V2_CAR_RUN,
  V2_CAR_STOPPED,
  V2_CAR_EMERGENCY
} v2_car_state_t;

void           v2_car_fsm_init(void);
void           v2_car_fsm_run(uint32_t dt_ms);
v2_car_state_t v2_car_fsm_state(void);

#endif /* V2_CAR_FSM_H */
