#ifndef SPEED_H
#define SPEED_H

#include <stdint.h>

/* 测速抽象: 现在=霍尔EXTI计数; 新车(USE_ENCODER=1)=编码器正交解码。
   对上层只暴露 RPM 接口, 换测速方式只改这一层。 */

/* 测速初始化 */
void speed_init(void);

/* 计算本窗口(遥测周期500ms)的 RPM 并清零窗口计数。由遥测每周期调用一次 */
void speed_update(void);

/* 读最近一次 speed_update() 算好的 RPM(vol: MOTOR_LEFT/MOTOR_RIGHT) */
int16_t speed_get_rpm(int vol);

/* 原始脉冲计数(调试用) */
uint16_t speed_get_count(int vol);

#endif /* SPEED_H */
