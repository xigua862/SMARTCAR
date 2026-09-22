#ifndef PID_H
#define PID_H

#include <stdint.h>

/* 通用 PID(带微分先行/抗积分饱和/输出限幅)。
   速度闭环和循迹转向环都可复用。 */
typedef struct {
  float  kp, ki, kd;   /* 比例/积分/微分 */
  float  integral;     /* 积分累积 */
  float  prev_meas;    /* 上一次测量值(微分先行, 避免设定值突变冲击) */
  float  out_min;      /* 输出下限 */
  float  out_max;      /* 输出上限 */
  float  i_max;        /* 积分限幅(抗饱和), 绝对值 */
  float  bias;         /* 输出偏置(可选) */
  uint8_t first;       /* 首拍标志(微分初值) */
} pid_t;

void pid_init(pid_t* p, float kp, float ki, float kd, float out_min, float out_max, float i_max);
void pid_reset(pid_t* p);

/* 一步更新: 返回限幅后的控制输出。dt 为控制周期(秒) */
float pid_update(pid_t* p, float setpoint, float meas, float dt);

#endif /* PID_H */
