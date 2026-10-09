#include "v2_steering.h"

void v2_steering_init(void) { }
void v2_steering_reset(void) { }

int16_t v2_steering_step(int16_t err_x100, int16_t dps_x100, int16_t v_tgt)
{
  (void)err_x100; (void)dps_x100; (void)v_tgt;
  return 0;      /* TODO(M1) */
}
