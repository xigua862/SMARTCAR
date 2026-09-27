#include "pose.h"
#include "app_config.h"
#include "drivers/odom.h"
#include "drivers/imu.h"
#include <math.h>

/* ---- 实现规则（CARD-005 钉死）----
   每拍取增量：ΔL = odom_left_mm() - 上拍值、ΔR = odom_right_mm() - 上拍值
   航向：ψ += (-imu_get_gyro_z()) × dt   （GZ 左正 → ψ 右正，所以取负）
   中点航向法（照抄马铃薯 drive_odometry，二阶精度）：
     ψ_mid = ψ_old + (ψ_new - ψ_old)/2
     d = (ΔL + ΔR)/2
     x += d × cos(ψ_mid);   y += d × sin(ψ_mid)
   dt 用 CTRL_PERIOD_MS（与 imu.c 同源）。 */

#define POSE_PI  3.14159265358979f

static float   s_x_mm    = 0.0f;
static float   s_y_mm    = 0.0f;
static float   s_yaw_rad = 0.0f;   /* 顺时针为正(右转增) */
static int32_t s_prev_l  = 0;
static int32_t s_prev_r  = 0;
static uint8_t s_inited  = 0;

void pose_reset(void)
{
  s_x_mm    = 0.0f;
  s_y_mm    = 0.0f;
  s_yaw_rad = 0.0f;
  s_prev_l  = odom_left_mm();    /* 上拍值同步重采（odom_reset 后不会跳变） */
  s_prev_r  = odom_right_mm();
  s_inited  = 1u;
  imu_yaw_reset();               /* ★同步清零 IMU yaw 积分起点 */
}

void pose_step(void)
{
  int32_t cur_l, cur_r;
  float dl, dr, d, yaw_new, yaw_mid;

  if (!s_inited) { pose_reset(); return; }

  cur_l = odom_left_mm();
  cur_r = odom_right_mm();

  /* ★实测：本车 odom_left_mm/right_mm 前进时【为负】（编码器 A/B 接线反向，
     CARD-001/002 已记录，车体 OD 用绝对值绕过所以不受影响）。
     pose 需要"前进为正"的轮位移 → 取负。
     只翻这里一处，坐标系/航向公式仍按卡原样。 */
  dl = -(float)(cur_l - s_prev_l);
  dr = -(float)(cur_r - s_prev_r);
  s_prev_l = cur_l;
  s_prev_r = cur_r;

  /* ★FIX 2026-09-26：航向不再自积分（原用固定 dt=CTRL_PERIOD_MS 导致欠积分 ~18%，
     CARD-005 B1 实测左转 90° 只积到 72°）；
     改用 imu_get_yaw()（已用 HAL_GetTick 实测 dt + 零偏补偿 + 连续角度）；
     imu yaw 左正 → pose 右系顺时针为正，取负。 */
  yaw_new = -(float)imu_get_yaw() * (POSE_PI / 180.0f);

  /* 中点航向法（二阶精度） */
  yaw_mid = s_yaw_rad + (yaw_new - s_yaw_rad) * 0.5f;
  d = (dl + dr) * 0.5f;
  s_x_mm += d * cosf(yaw_mid);
  s_y_mm += d * sinf(yaw_mid);
  s_yaw_rad = yaw_new;
}

float pose_get_x_mm(void)    { return s_x_mm; }
float pose_get_y_mm(void)    { return s_y_mm; }
float pose_get_yaw_deg(void) { return s_yaw_rad * (180.0f / POSE_PI); }
