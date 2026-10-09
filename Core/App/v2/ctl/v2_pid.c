#include "v2_pid.h"

void v2_pid_init(v2_pid_t* p, int32_t kp_q8, int32_t ki_q8, int32_t kd_q8,
                 int32_t out_min, int32_t out_max)
{
  if (p == 0) return;
  p->kp_q8 = kp_q8; p->ki_q8 = ki_q8; p->kd_q8 = kd_q8;
  p->out_min = out_min; p->out_max = out_max;
  p->e_prev = 0; p->i_acc = 0;
}

void v2_pid_reset(v2_pid_t* p)
{
  if (p == 0) return;
  p->e_prev = 0; p->i_acc = 0;
}

int32_t v2_pid_step(v2_pid_t* p, int32_t err)
{
  (void)p; (void)err;
  return 0;      /* TODO(M1) */
}
