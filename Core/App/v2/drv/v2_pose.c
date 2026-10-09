#include "v2_pose.h"
#include "v2_odom.h"
#include "v2_imu.h"

void v2_pose_init(void)
{
  v2_odom_reset();
  v2_imu_yaw_reset();
}

void v2_pose_reset(void)
{
  v2_pose_init();
}

pose_t v2_pose_get(void)
{
  pose_t p;
  p.od_mm    = v2_odom_mm();
  p.yaw_mdeg = v2_imu_yaw_mdeg();
  p.dps_x100 = v2_imu_dps_x100();
  return p;
}
