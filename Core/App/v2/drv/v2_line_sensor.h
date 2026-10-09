#ifndef V2_LINE_SENSOR_H
#define V2_LINE_SENSOR_H

#include <stdint.h>
#include "v2_mid_types.h"

/* ============================================================================
 * v2 循迹驱动（冻结@M0）
 *   8 路：左->右 PA0 PA1 PA8 PA15 PB0 PB1 PB12 PB13
 *   权重：-7 -5 -3 -1 +1 +3 +5 +7（等比排布，见 app_config.h 定稿）
 *   极性：白线输出【高】（2026-09-20 实测定稿，与旧 5 路模块相反）
 *   采样：一拍内突发 8 次（间隔 1µs）→ 多数表决（照旧 line_sensor.c 算法搬）
 *   误差：err_x100 = 100 * SUM(w*s) / (SUM(s)+1)   （差比和"分母+1"，框架 §④）
 *
 * DWT：本文件兼管 DWT 周期计数器（与旧 line_sensor.c 同一做法，不再新开文件），
 *      供突发采样定间距 + ISR 耗时测量。
 * ==========================================================================*/

void            v2_line_init(void);
line_reading_t  v2_line_read(void);      /* 1kHz ISR 用：突发过采样 + 表决 + 误差 */
uint8_t         v2_line_raw_once(void);  /* 单次原始位图（不表决），诊断用 */

void            v2_dwt_init(void);       /* 使能 DWT->CYCCNT */
uint32_t        v2_dwt_cycles(void);     /* 原始周期计数 */
void            v2_dwt_delay_us(uint16_t us);

#endif /* V2_LINE_SENSOR_H */
