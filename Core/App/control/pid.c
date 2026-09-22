#include "pid.h"

void pid_init(pid_t* p, float kp, float ki, float kd, float out_min, float out_max, float i_max)
{
  p->kp = kp; p->ki = ki; p->kd = kd;
  p->out_min = out_min; p->out_max = out_max; p->i_max = i_max;
  p->bias = 0.0f;
  pid_reset(p);
}

void pid_reset(pid_t* p)
{
  p->integral = 0.0f;
  p->prev_meas = 0.0f;
  p->first = 1;
}

float pid_update(pid_t* p, float setpoint, float meas, float dt)
{
  float err = setpoint - meas;

  /* P 项 */
  float p_term = p->kp * err;

  /* I 项(带抗积分饱和: 限幅积分累积) */
  p->integral += p->ki * err * dt;
  if (p->integral >  p->i_max) p->integral =  p->i_max;
  if (p->integral < -p->i_max) p->integral = -p->i_max;
  float i_term = p->integral;

  /* D 项(微分先行: 对测量值微分, 避免 setpoint 跳变冲击; 首拍不微分) */
  float d_term = 0.0f;
  if (!p->first)
  {
    d_term = p->kd * (p->prev_meas - meas) / dt;   /* 注意: 测量值导向, 与误差微分符号关系见注释 */
  }
  p->prev_meas = meas;
  p->first = 0;

  float out = p_term + i_term + d_term + p->bias;
  if (out > p->out_max) out = p->out_max;
  if (out < p->out_min) out = p->out_min;
  return out;
}
