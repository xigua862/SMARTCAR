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
/* ★★★ 2026-09-26 晚【新增：六轴原始量 + 纯加速度计角度】—— 遥测用 ★★★
   用户要求："加一下 mpu6050 和角度数据，我感觉小车走的时候污染有点严重"。
   · imu_get_raw：直接给出 MPU6050 的六个原始 LSB。
       加速度计 ±8g → 4096 LSB/g；陀螺 ±1000dps → 32.8 LSB/(°/s)。
       静止平放时 az ≈ +4096（1g），ax/ay ≈ 0；gyro 三轴 ≈ 0（若已标定则更接近 0）。
       ⇒ **行车时这个数组抖成什么样，就是"污染有多严重"的直接读数。**
   · imu_get_acc_angles：只用加速度计算 pitch/roll，【不做任何滤波】。
       故意不滤波：滤波会把抖动抹平，而我们要看的正是抖动本身。 */
void imu_get_raw(int16_t a[3], int16_t g[3]);
void imu_get_acc_angles(float* pitch_deg, float* roll_deg);

/* ★★★ 2026-09-26 晚【新增：滤波后的六轴 + 污染度量】—— 用户要求 ★★★
   用户原话："我要看的是滤波以后那些数据的污染值严不严重，看它对小车的不良影响程度"。
   ⇒ 光看原始量只能证明"有污染"，回答不了"影响多大"。
      本函数给三样东西：
        fa[3]/fg[3] = 100Hz 一阶低通后的六轴（就是"滤波以后的数据"）
        res_a/res_g = 本窗口内 平均|原始 − 滤波|（三轴平均，LSB）
                      = **被滤掉了多少** = 污染强度
        pitch/roll  = 用【滤波后】加速度算的角度
                      ⇒ **它抖多少，就是"滤波后还剩多少、对车有多大影响"**
   ★调用即清零统计窗口（读-清语义），所以调用周期 = 统计周期。
   ★本函数与低通【只读、不参与任何控制】—— 控制路径仍用未滤波的原始量。 */
void imu_get_filtered(int16_t fa[3], int16_t fg[3],
                      int16_t* res_a, int16_t* res_g,
                      float* pitch_deg, float* roll_deg);

/* ★2026-09-26 晚：实时读回 CONFIG(0x1A) 的低 3 位（DLPF_CFG）。
   03 = DLPF 已生效；00 = 写入没生效（这颗克隆有前科）。
   遥测 IMUn 行末尾的 C=xx 用的就是它。 */
uint8_t imu_get_cfg(void);
#endif /* IMU_H */
