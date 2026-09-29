#ifndef STEERING_TRIM_H
#define STEERING_TRIM_H

/* 两个方向使用相同的幅值规则；浮点状态保留每拍不足一个单位的变化。 */
static float steering_trim_update(float *state, float demand,
                                  float rise, float fall, float leak, float limit)
{
  float current = *state;
  float magnitude;
  float target_magnitude;

  /* 换向时立即释放旧方向记忆，新方向仍按增长系数建立。 */
  if (current * demand < 0.0f) current = 0.0f;
  magnitude = current < 0.0f ? -current : current;
  target_magnitude = demand < 0.0f ? -demand : demand;
  current += (demand - current) * (target_magnitude > magnitude ? rise : fall);

  /* 泄漏只回到零，不能跨零形成反方向的微调。 */
  if (current > 0.0f) current = current > leak ? current - leak : 0.0f;
  else if (current < 0.0f) current = current < -leak ? current + leak : 0.0f;
  if (current > limit) current = limit;
  if (current < -limit) current = -limit;
  *state = current;
  return current;
}

#endif
