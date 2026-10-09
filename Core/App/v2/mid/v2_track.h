#ifndef V2_TRACK_H
#define V2_TRACK_H

#include "v2_mid_types.h"

/* ============================================================================
 * v2 赛道判据层（冻结@M0，实现 M2）
 *   铁律：本层【只读不写】——只输出事件/状态，绝不调用 motor_*（旧屎山 1817 行的教训）
 *   每个元素只留一套判据；公共门 = 里程窗口 AND 传感形态 AND 陀螺角度
 * ==========================================================================*/

void          v2_track_init(void);
void          v2_track_reset(void);
track_event_t v2_track_poll(int32_t od_mm, int32_t yaw_mdeg, const line_reading_t* lr);

#endif /* V2_TRACK_H */
