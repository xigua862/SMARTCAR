#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/* 一阶低通滤波器(用于压 D 项/传感器噪声, 解决高速时 10ms 周期 D 项放大蛇形的问题) */
typedef struct {
  float y;      /* 上一次输出 */
  float alpha;  /* 滤波系数(0~1): 越大越跟随, 越小越平滑 */
} lpf_t;

void  lpf_init(lpf_t* f, float alpha);
float lpf_update(lpf_t* f, float x);   /* 返回滤波后值 */

/* 滑动平均(循环缓冲): 返回当前窗口均值, 并更新缓冲 */
float mavg_update(float* buf, uint8_t size, uint8_t* head, float x);

#endif /* FILTER_H */
