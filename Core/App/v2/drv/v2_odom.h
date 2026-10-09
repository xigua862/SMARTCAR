#ifndef V2_ODOM_H
#define V2_ODOM_H

#include <stdint.h>

/* ============================================================================
 * v2 里程计（冻结@M0）
 *   1kHz 调用 v2_odom_tick()：读编码器增量 -> 微米积分 -> od_mm
 *   换算：轮径 66mm -> 周长 207345µm / 422 计数 = 491 µm/计数
 *   ★取绝对值累加（|dL| + |dR|)/2：与正交解码接线方向无关，不用标定符号
 *     （旧 odom.c 用实车抓的 bug 换来的结论；代价：倒退也当正数累加）
 * ==========================================================================*/

void    v2_odom_init(void);     /* 复位并记住当前计数 */
void    v2_odom_reset(void);
void    v2_odom_tick(void);     /* 1kHz ISR 调用 */
int32_t v2_odom_mm(void);       /* 里程(mm)，恒 >= 0 */

#endif /* V2_ODOM_H */
