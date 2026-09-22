#ifndef CAR_FSM_H
#define CAR_FSM_H

#include <stdint.h>

/* 车辆状态机：
 *   IDLE ──按键──> COUNTDOWN(3s 读秒: 灯逐个亮 + 哔) ──> RUN(循迹)
 *                                                         │
 *              按键重启 <── STOPPED(到站停车) <── 终点/急停 ──> EMERGENCY
 */
typedef enum {
  CAR_IDLE = 0,      /* 待机(上电默认, 电机停, 等按键) */
  CAR_COUNTDOWN,     /* 启动读秒(3 步: LED1/2/3 逐个亮 + 每步一声) */
  CAR_RUN,           /* 循迹运行中(原名 CAR_LINE_FOLLOW) */
  CAR_STOPPED,       /* 到站停车(终点判定成立) */
  CAR_EMERGENCY      /* 急停(长按按键 / 异常) */
} car_state_t;

void        car_fsm_init(void);
void        car_fsm_run(uint32_t dt_ms);   /* 每控制周期调用, 内部按状态调度 */
car_state_t car_fsm_state(void);

/* 状态切换接口 */
void car_fsm_request_start(void);   /* IDLE/STOPPED → COUNTDOWN */
void car_fsm_emergency_stop(void);  /* 任意 → EMERGENCY */
void car_fsm_finish(void);          /* RUN → STOPPED(到站, 停车+长鸣+闪灯) */

#endif /* CAR_FSM_H */
