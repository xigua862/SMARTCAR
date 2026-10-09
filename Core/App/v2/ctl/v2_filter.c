#include "v2_filter.h"

void v2_lpf_init(v2_lpf_t* f, int32_t k_q8)
{
  if (f == 0) return;
  f->y = 0; f->k_q8 = k_q8; f->inited = 0u;
}

void v2_lpf_reset(v2_lpf_t* f)
{
  if (f == 0) return;
  f->y = 0; f->inited = 0u;
}

int32_t v2_lpf_step(v2_lpf_t* f, int32_t x)
{
  (void)x;
  if (f == 0) return 0;
  return f->y;   /* TODO(M1) */
}
