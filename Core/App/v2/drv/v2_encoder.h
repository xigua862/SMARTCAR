#ifndef V2_ENCODER_H
#define V2_ENCODER_H

#include <stdint.h>

/* ============================================================================
 * v2 编码器驱动（冻结@M0）
 *   左轮 TIM3（PA6/PA7）、右轮 TIM4（PB6/PB7），TI12 四倍频，PPR = 422 计数/轮圈
 *   TIM3/TIM4 是 16 位（Period = 65535）→ 增量一律用【带符号 16 位差】算，回绕天然安全
 * ==========================================================================*/

#define V2_ENC_LEFT   0
#define V2_ENC_RIGHT  1

void    v2_enc_init(void);
int32_t v2_enc_take_delta(int ch);   /* 自上次调用起的带符号增量（可正可负） */
int32_t v2_enc_raw(int ch);          /* 原始计数值（0..65535） */

#endif /* V2_ENCODER_H */
