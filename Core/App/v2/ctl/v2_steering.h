#ifndef V2_STEERING_H
#define V2_STEERING_H

#include <stdint.h>

/* v2 外环（转向，冻结@M0）：err_x100(+陀螺阻尼) -> 差速量
   限幅规则（M1 落地）：转向量限幅 = 当前基础速度（左右轮目标不为负） */

void    v2_steering_init(void);
void    v2_steering_reset(void);
int16_t v2_steering_step(int16_t err_x100, int16_t dps_x100, int16_t v_tgt);

#endif /* V2_STEERING_H */
