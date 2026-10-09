#ifndef V2_MID_TYPES_H
#define V2_MID_TYPES_H

#include <stdint.h>

/* ============================================================================
 * v2 跨层共享数据类型（冻结@M0）
 *   改这里的结构体 = 方案级变更 = 回军师，不许就地改。
 * ==========================================================================*/

/* 一帧循迹读数（表决后） */
typedef struct {
  uint8_t  bitmap;     /* bit0 = 最左 … bit7 = 最右 */
  uint8_t  active;     /* 亮（白线）路数 */
  int16_t  err_x100;   /* 误差 ×100：100*SUM(w*s)/(SUM(s)+1)，w = -7..+7 */
} line_reading_t;

/* 位姿快照（ISR 写 / 主循环读） */
typedef struct {
  int32_t  od_mm;      /* 里程(mm)：取绝对值累加 -> 恒 >= 0 */
  int32_t  yaw_mdeg;   /* 航向角(毫度)：正 = 左转 */
  int16_t  dps_x100;   /* 偏航率 ×100 (°/s) */
} pose_t;

/* 赛道事件（M2 才用，M0 只占位） */
typedef enum {
  V2_TRK_NONE = 0,
  V2_TRK_RING_ENTER,
  V2_TRK_RING_INSIDE,
  V2_TRK_RING_EXIT,
  V2_TRK_RING_LEAVE,
  V2_TRK_CROSS,
  V2_TRK_LINE_LOST,
  V2_TRK_START_LINE,
  V2_TRK_FINISH
} track_event_t;

#endif /* V2_MID_TYPES_H */
