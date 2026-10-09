#ifndef V2_PID_H
#define V2_PID_H

#include <stdint.h>

/* v2 PID（冻结@M0）：全定点（F103 无 FPU，ISR 内禁止浮点）
   M0 只冻结签名；增量式 PI + 限幅在 M1 落地（抄国一 pid.c:284-296 的做法） */
typedef struct {
  int32_t kp_q8;      /* 增益定点（Q8 举例，M1 定标） */
  int32_t ki_q8;
  int32_t kd_q8;
  int32_t out_min;
  int32_t out_max;
  int32_t e_prev;
  int32_t i_acc;
} v2_pid_t;

void    v2_pid_init(v2_pid_t* p, int32_t kp_q8, int32_t ki_q8, int32_t kd_q8,
                    int32_t out_min, int32_t out_max);
void    v2_pid_reset(v2_pid_t* p);
int32_t v2_pid_step(v2_pid_t* p, int32_t err);

#endif /* V2_PID_H */
