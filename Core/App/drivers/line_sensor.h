#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

/* 一次循迹读取结果 */
typedef struct {
  int8_t  error;   /* 加权误差: 偏左负、偏右正、居中0 */
  uint8_t active;  /* 检测到线的路数(0=全丢线) */
  uint16_t raw;    /* 每路bit位图: bit i = 第 i 路激活(调试用) */
} line_reading_t;

/* 循迹初始化(引脚由 CubeMX gpio.c 配置; 预留 ADC 等后端) */
void line_sensor_init(void);

/* 读 5/8 路循迹并算加权误差。纯读, 不做丢线启发式(那是控制层的事) */
line_reading_t line_read(void);

#endif /* LINE_SENSOR_H */
