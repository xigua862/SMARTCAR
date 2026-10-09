#ifndef V2_FILTER_H
#define V2_FILTER_H

#include <stdint.h>

/* v2 一阶低通（冻结@M0）：y += (x - y) * k，k 用 Q8/Q16 定点
   M0 只冻结签名；M1 用于误差与陀螺读数平滑 */
typedef struct { int32_t y; int32_t k_q8; uint8_t inited; } v2_lpf_t;

void    v2_lpf_init(v2_lpf_t* f, int32_t k_q8);
void    v2_lpf_reset(v2_lpf_t* f);
int32_t v2_lpf_step(v2_lpf_t* f, int32_t x);

#endif /* V2_FILTER_H */
