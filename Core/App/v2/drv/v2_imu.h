#ifndef V2_IMU_H
#define V2_IMU_H

#include <stdint.h>

/* ============================================================================
 * v2 IMU 驱动（冻结@M0）
 *   ★M0 不重写 I2C：内部包旧 drivers/imu.c（I2C2 软驱动 PB10/PB11，MPU-6500，WHO=0x70）
 *   分工：主循环 100Hz 调 v2_imu_poll()（允许 I2C/浮点）；ISR 只读缓存，禁止发起 I2C
 * ==========================================================================*/

void    v2_imu_init(void);          /* 包旧 imu_init + present 时 imu_calibrate */
void    v2_imu_poll(void);          /* 100Hz：刷缓存（读原始量 + 积分航向） */
int16_t v2_imu_dps_x100(void);      /* 偏航率 ×100 (°/s)，ISR 可读 */
int32_t v2_imu_yaw_mdeg(void);      /* 航向角(毫度)，正 = 左转 */
void    v2_imu_yaw_reset(void);
uint8_t v2_imu_valid(void);         /* 0 = IMU 不在/未标定 */

#endif /* V2_IMU_H */
