#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* 编码器(USE_ENCODER=1 时启用)
   使用前提: CubeMX 把编码器 A/B 两相配到某 TIM 的 CH1/CH2(Encoder Mode),
   生成工程(会新建 tim.c 里对应 handle, 如 htim3/htim4)。
   注意: 每圈脉冲数(ENCODER_PPR)填 motor 实际值(需结合减速比换算到轮子圈数)。 */

void encoder_init(void);              /* 启动编码器计数(USE_ENCODER=0 时为空实现) */
int32_t encoder_get_count(int vol);   /* 读某轮编码器当前计数值, 可为负 */

#endif /* ENCODER_H */
