#ifndef APP_H
#define APP_H

#include "app_config.h"

/* 运行时参数(在线可改), 在 app.c 定义, 供 line_follow/telemetry 共享 */
extern int16_t kp_x10;      /* KP * 10 */
extern int16_t kd_x10;      /* KD * 10 */
extern int16_t base_speed;  /* 基础速度 0~99（关掉自适应时用它） */
extern int16_t sp_straight; /* 自适应: 直道目标速度（S 指令同步改这个） */
extern int16_t sp_curve;    /* 自适应: 弯道目标速度（= 直道 × CURVE_SPEED_RATIO） */
extern uint8_t test_mode;   /* 1=单电机测试(暂停循迹与状态机) */
extern uint16_t loop_dt_ms; /* ★实测控制周期(ms), 遥测 DT=。应为 10 附近;
                               被拖到 20+ 说明主循环里有阻塞(例如 IMU 的 I2C 读超时) */

/* 顶层入口 */
void app_init(void);
void app_loop(void);

#endif /* APP_H */
