#include "v2_speeds.h"

void v2_speeds_init(void) { }
void v2_speeds_reset(void) { }

int16_t v2_speeds_step(int ch, int16_t rpm_tgt, int16_t rpm_meas, int16_t ff_pwm)
{
  (void)ch; (void)rpm_tgt; (void)rpm_meas; (void)ff_pwm;
  return 0;      /* TODO(M1) */
}
