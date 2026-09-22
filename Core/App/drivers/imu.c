#include "imu.h"
#include "app_config.h"

#if USE_IMU
/* ============================================================================
 * MPU6050 实现 (USE_IMU=1)  —— 2026-09-22 启用
 *  ★ 接线：I2C2（PB10=SCL / PB11=SDA），驱动在 Core/Src/i2c.c（我手写的，不经 CubeMX）
 *  ★ 本文件提供两层：
 *      1) 姿态：加速度计定姿 + 陀螺互补滤波 → pitch / roll（防翻、姿态监测）
 *      2) 原始量：WHO_AM_I 自检 + 陀螺 Z 轴角速度（偏航率）→ 给循迹用
 *         （陀螺 Z = 车头转得多快，可用来判"真的在走直线"、以及葫芦弯的相切点）
 * ==========================================================================*/
#include "main.h"
#include "i2c.h"               /* hi2c2 */
#include <math.h>
#include <string.h>

#define MPU_ADDR    (IMU_I2C_ADDR)
#define DEG_TO_RAD  (3.14159265358979f / 180.0f)
#define RAD_TO_DEG  (180.0f / 3.14159265358979f)
#define GYRO_2000   16.4f      /* ±2000dps 时陀螺灵敏度 LSB/(°/s) */
#define ACC_8G      4096.0f    /* ±8g 时加速度计灵敏度 LSB/g */

extern I2C_HandleTypeDef hi2c2;

static int16_t ax, ay, az;      /* 加速度计原始量 */
static int16_t gx, gy, gz;      /* 陀螺仪原始量 */
static float   gz_dps = 0.0f;   /* ★偏航率(度/秒)，已减零偏 */
static float   pitch = 0.0f, roll = 0.0f;
static uint8_t calibrated = 0;
static uint8_t present    = 0;  /* WHO_AM_I 自检通过 */
static float   gyro_bx = 0.0f, gyro_by = 0.0f, gyro_bz = 0.0f;  /* 三轴零偏 */

static void imu_write_reg(uint8_t reg, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, reg, 1, &val, 1, 50);
}

static uint8_t imu_read_reg(uint8_t reg)
{
  uint8_t v = 0;
  HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, reg, 1, &v, 1, 50);
  return v;
}

static void imu_read_raw(void)
{
  uint8_t d[14];
  if (HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x3B, 1, d, 14, 50) != HAL_OK) return;
  ax = (int16_t)((d[0] << 8) | d[1]);
  ay = (int16_t)((d[2] << 8) | d[3]);
  az = (int16_t)((d[4] << 8) | d[5]);
  gx = (int16_t)((d[8] << 8) | d[9]);
  gy = (int16_t)((d[10] << 8) | d[11]);
  gz = (int16_t)((d[12] << 8) | d[13]);
}

/* WHO_AM_I：MPU6050 应返回 0x68（读不到说明没接上/地址不对） */
uint8_t imu_who_am_i(void)
{
  return imu_read_reg(0x75);
}

uint8_t imu_is_present(void)
{
  return present;
}

float imu_get_gyro_z(void)
{
  return gz_dps;                 /* 度/秒，正 = 左转（右手系绕 Z） */
}

void imu_update(void)
{
  imu_read_raw();
  gz_dps = (float)gz / GYRO_2000;
  if (calibrated) gz_dps -= gyro_bz;
}

void imu_init(void)
{
  imu_write_reg(0x6B, 0x00);   /* PWR_MGMT_1: 唤醒 */
  imu_write_reg(0x1A, 0x00);   /* CONFIG: 关闭DLPF */
  imu_write_reg(0x1B, 0x10);   /* GYRO_CONFIG: ±2000dps */
  imu_write_reg(0x1C, 0x10);   /* ACCEL_CONFIG: ±8g */
  imu_write_reg(0x1D, 0x00);   /* 加速度计低通 */
  calibrated = 0;
  pitch = 0.0f; roll = 0.0f;
  gz_dps = 0.0f;
  present = (imu_who_am_i() == 0x68) ? 1u : 0u;
}

/* 三轴陀螺零偏: 静止采样 100 次取平均(开机时调用一次) */
void imu_calibrate(void)
{
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 100; i++)
  {
    imu_read_raw();
    sx += (float)gx / GYRO_2000;
    sy += (float)gy / GYRO_2000;
    sz += (float)gz / GYRO_2000;
  }
  gyro_bx = sx / 100.0f;
  gyro_by = sy / 100.0f;
  gyro_bz = sz / 100.0f;
  calibrated = 1;
}

void imu_read_angles(float* pitch_out, float* roll_out)
{
  imu_read_raw();
  float gxp = (float)gx / GYRO_2000;
  float gyp = (float)gy / GYRO_2000;
  if (calibrated) { gxp -= gyro_bx; gyp -= gyro_by; }

  /* 加速度计定姿(静止较准, 有震动噪声) */
  float ax_f = (float)ax / ACC_8G, ay_f = (float)ay / ACC_8G, az_f = (float)az / ACC_8G;
  float pitch_acc = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * RAD_TO_DEG;
  float roll_acc  = atan2f(ay_f, sqrtf(ax_f * ax_f + az_f * az_f)) * RAD_TO_DEG;

  /* 互补滤波: 角度 = 0.98*(角度+陀螺角速度*dt) + 0.02*角度(加速度计)  (调用周期约10ms) */
  float dt = 0.010f;
  pitch = 0.98f * (pitch + gyp * dt) + 0.02f * pitch_acc;
  roll  = 0.98f * (roll  + gxp * dt) + 0.02f * roll_acc;

  *pitch_out = pitch;
  *roll_out  = roll;
}

#else  /* USE_IMU=0: 空实现 */
void    imu_init(void) { }
void    imu_calibrate(void) { }
void    imu_update(void) { }
uint8_t imu_who_am_i(void) { return 0; }
uint8_t imu_is_present(void) { return 0; }
float   imu_get_gyro_z(void) { return 0.0f; }
void    imu_read_angles(float* pitch, float* roll) { (void)pitch; (void)roll; }
#endif
