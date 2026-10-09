#include "v2_track.h"

/* M0：空壳。判据在 M2 落地（圆环五态机 / 十字 / 斑马线 / 失线）。 */

void v2_track_init(void) { }
void v2_track_reset(void) { }

track_event_t v2_track_poll(int32_t od_mm, int32_t yaw_mdeg, const line_reading_t* lr)
{
  (void)od_mm; (void)yaw_mdeg; (void)lr;
  return V2_TRK_NONE;      /* TODO(M2) */
}
