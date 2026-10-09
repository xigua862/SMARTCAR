#ifndef V2_SPEEDS_H
#define V2_SPEEDS_H

#include <stdint.h>

/* v2 内环（速度环，冻结@M0）：增量式 PI + 单拍限幅 + 堵转保护 + 前馈
   输入：目标 RPM、实测 RPM、前馈 PWM；输出：PWM 档(0..99)
   M0 只冻结签名（ISR 里返回 0 = 电机不动） */

void    v2_speeds_init(void);
void    v2_speeds_reset(void);
int16_t v2_speeds_step(int ch, int16_t rpm_tgt, int16_t rpm_meas, int16_t ff_pwm);

#endif /* V2_SPEEDS_H */
