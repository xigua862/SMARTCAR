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

/* 读 5/8 路循迹并算加权误差。纯读, 不做丢线启发式(那是控制层的事)
   ★2026-09-23 起：本函数内部做【突发过采样】(见 app_config.h 的 LINE_OS_*)，
     返回的是表决后的结果。三个调用点(循迹 1 处 + 遥测 2 处)自动受益。 */
line_reading_t line_read(void);

/* ★过采样证据（读一次清一次）：窗口内"同一拍子采样位图不一致"的拍数。
   >0 = 快采样抓到了单次采样会漏掉的东西（过采样在起作用）
   =0 = 这几个 µs 内红外没变（模块跟不上 / 间距太小 / 过采样关了） */
uint32_t line_os_disagree_take(void);

/* ★过采样统计（累计, 不清零）：ticks=已过采样拍数, disagree=其中不一致的拍数 */
void line_os_stats(uint32_t* ticks, uint32_t* disagree);

#endif /* LINE_SENSOR_H */
