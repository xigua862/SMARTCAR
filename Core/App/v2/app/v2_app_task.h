#ifndef V2_APP_TASK_H
#define V2_APP_TASK_H

#include <stdint.h>
#include "v2_mid_types.h"

/* ============================================================================
 * v2 应用入口（冻结@M0）
 *   v2_app_init()  上电初始化（最后一步才启动 1kHz 节拍）
 *   v2_app_loop()  主循环（非阻塞；每圈必须调 v2_uart_poll()）
 *   TIM1_UP_IRQHandler 在 .c 里定义 = 1kHz 感知/控制拍
 * ==========================================================================*/

void v2_app_init(void);
void v2_app_loop(void);

/* 观测快照（遥测用；内部关中断快照，保证跨拍一致） */
void v2_app_line_take(line_reading_t* out);
void v2_app_pose_take(pose_t* out);
void v2_app_isr_take(uint32_t* us_avg, uint32_t* overlong, uint32_t* ticks);
uint32_t v2_app_isr_us_last(void);

#endif /* V2_APP_TASK_H */
