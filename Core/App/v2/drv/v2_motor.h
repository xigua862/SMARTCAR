#ifndef V2_MOTOR_H
#define V2_MOTOR_H

#include <stdint.h>

/* ============================================================================
 * v2 电机驱动（冻结@M0）
 *   PWM：TIM2_CH3 = PA2（左）、TIM2_CH4 = PA3（右），10kHz，ARR 预装载
 *   方向：DIR1=PA4 / DIR2=PA5（左）；DIR3=PB3 / DIR4=PB4（右）
 *   反转：沿用实测定稿值 MOTOR_LEFT_INVERT = MOTOR_RIGHT_INVERT = 1（app_config.h）
 * ==========================================================================*/

#define V2_MOTOR_LEFT   0
#define V2_MOTOR_RIGHT  1
#define V2_MOTOR_MAX    99

void v2_motor_init(void);
void v2_motor_set(int ch, int16_t speed);   /* -99..99，正 = 前进，0 = 停 */
void v2_motor_stop(void);

#endif /* V2_MOTOR_H */
