#ifndef IMU_H
#include <stdint.h>

#define IMU_H

/* MPU6050(USE_IMU=1 时启用)。I2C 由 CubeMX 配置(新建 i2c.c, 提供 hi2c1)。
   用途: 姿态/防翻(读俯仰/横滚角)。 */

void imu_init(void);
void imu_read_angles(float* pitch, float* roll);   /* 输出角度(度) */

void    imu_calibrate(void);      /* 静止采样求三轴陀螺零偏(开机调一次) */
void    imu_update(void);         /* 只读原始量 + 更新偏航率 + ★积分航向角(控制周期里调) */
uint8_t imu_who_am_i(void);       /* MPU6050 应返回 0x68 */
uint8_t imu_is_present(void);     /* 开机自检结果 */
float   imu_get_gyro_z(void);     /* 偏航率 (°/s, 已减零偏; 正=左转) */

/* ★航向角（2026-09-23 新增）：把偏航率积分成角度。
   正 = 左转，负 = 右转。转弯前先 imu_yaw_reset()。
   ⚠️ 纯积分、无绝对参考 → 会缓慢漂移（约 20 秒漂 1°）。
      只适合"转弯这几百毫秒内量转了多少度"，不能当整圈航向用。 */
float   imu_get_yaw(void);
void    imu_yaw_reset(void);
uint8_t imu_yaw_is_valid(void);   /* 0 = IMU 不在/没标定 → 别用它做控制, 走兜底 */

/* ★★★ 2026-09-26 【陀螺削顶观测】—— 只观测，不改控制 ★★★
   背景：±1000dps 档下实测 |GZ|>=990 占 7.4%（34/457），GZ=-995 重复 25 次 = 削顶铁证。
   而更宽的 ±2000dps 档【写不进去/直接坏掉】（试过两次，见 imu.c 里的长注释），
   所以削顶【改代码修不了】，只能先量清楚它到底是不是葫芦圈的瓶颈。
   遥测 `SAT=`（本拍是否削顶）+ `SATN=`（累计次数）就是为这个加的。 */
uint8_t  imu_gz_saturated(void);  /* 1 = 本拍陀螺 Z 削顶 */
uint32_t imu_gz_sat_count(void);  /* 开机以来累计削顶拍数 */
void     imu_gz_sat_reset(void);  /* 清零（每趟出发前可调） */
#endif /* IMU_H */
