#ifndef V2_ANGULATE_H
#define V2_ANGULATE_H

#include <stdint.h>

/* v2 中环（角速度环，冻结@M0）：目标偏航率 -> 差速修正
   M0 只冻结签名（返回 0） */

void    v2_angulate_init(void);
void    v2_angulate_reset(void);
int16_t v2_angulate_step(int16_t dps_x100, int16_t dps_tgt_x100);

#endif /* V2_ANGULATE_H */
