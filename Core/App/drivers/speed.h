#ifndef SPEED_H
#define SPEED_H

#include <stdint.h>

/* 初始化编码器和全部测速状态，以当前计数作为首个窗口的起点。 */
void speed_init(void);

/* 每个控制周期调用一次；按实际间隔测速，同毫秒调用不更新。 */
void speed_update(void);

/* 返回最近有效窗口的转速幅值；无效窗口保留显示值，控制前须检查有效性。 */
int16_t speed_get_rpm(int vol);

/* 最近一次非零测速窗口的实际间隔；初始化后尚未采样时为 0。 */
uint32_t speed_period_ms(void);

/* 两轮最新窗口均合理且未超时才返回 1；不能识别断线造成的恒定零计数。 */
uint8_t speed_feedback_valid(void);

/* 原始累计脉冲计数，仅用于调试。 */
uint16_t speed_get_count(int vol);

#endif
