#ifndef V2_SELFTEST_H
#define V2_SELFTEST_H

#include <stdint.h>

/* v2 上电自检（冻结@M0，实现 M1）：红外黑白基准 / 陀螺零偏 / 编码器方向 / 链路体检
   结果用蜂鸣 + LED 报，不阻塞主流程 */
void    v2_selftest_init(void);
uint8_t v2_selftest_run(void);   /* 1 = 全过 */

#endif /* V2_SELFTEST_H */
