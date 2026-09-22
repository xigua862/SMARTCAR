#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_SPEED_MAX   99   /* PWM 占空比上限(0~99) */
#define MOTOR_LEFT        (0)  /* 左轮 = 电机1 = PA2/TIM2_CH3 */
#define MOTOR_RIGHT       (1)  /* 右轮 = 电机2 = PA3/TIM2_CH4 */

/* 启动双路 PWM 并复位方向脚(调用一次) */
void motor_init(void);

/* 设置单侧电机速度: speed -99..99, 正=前进, 负=后退, 0=停止。vol: MOTOR_LEFT/MOTOR_RIGHT */
void motor_set_speed(int vol, int16_t speed);

/* 差速设置左右轮(循迹主接口) */
void motor_set_differential(int16_t left, int16_t right);

/* 双轮停止 */
void motor_stop(void);

#endif /* MOTOR_H */
