#include "filter.h"

void lpf_init(lpf_t* f, float alpha)
{
  f->y = 0.0f;
  f->alpha = alpha;
}

float lpf_update(lpf_t* f, float x)
{
  f->y = f->alpha * x + (1.0f - f->alpha) * f->y;
  return f->y;
}

float mavg_update(float* buf, uint8_t size, uint8_t* head, float x)
{
  /* 覆盖最旧采样点 */
  buf[*head] = x;
  *head = (uint8_t)((*head + 1) % size);
  float sum = 0.0f;
  for (uint8_t i = 0; i < size; i++) sum += buf[i];
  return sum / (float)size;
}
