#include "v2_imu.h"
#include "drivers/imu.h"     /* ★旧驱动原样复用（M0 不重写 I2C） */

static volatile int16_t s_dps_x100 = 0;
static volatile int32_t s_yaw_mdeg = 0;
static uint8_t s_valid = 0u;

void v2_imu_init(void)
{
  imu_init();
  if (imu_is_present())
  {
    imu_calibrate();            /* 上电静止求零偏（车必须静止） */
  }
  s_valid = imu_yaw_is_valid();
  s_dps_x100 = 0;
  s_yaw_mdeg = 0;
}

void v2_imu_poll(void)
{
  imu_update();                 /* 旧驱动：读原始量 + 更新偏航率 + 积分航向 */
  s_dps_x100 = (int16_t)(imu_get_gyro_z() * 100.0f);
  s_yaw_mdeg = (int32_t)(imu_get_yaw() * 1000.0f);
  s_valid    = imu_yaw_is_valid();
}

int16_t v2_imu_dps_x100(void) { return s_dps_x100; }
int32_t v2_imu_yaw_mdeg(void) { return s_yaw_mdeg; }
uint8_t v2_imu_valid(void)    { return s_valid; }

void v2_imu_yaw_reset(void)
{
  imu_yaw_reset();
  s_yaw_mdeg = 0;
}
