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
/* ★陀螺灵敏度 LSB/(°/s) —— ★2026-09-24 不再是编译期常量：
   imu_init() 会【读回 GYRO_CONFIG 的 FS_SEL】按实际生效的档位赋值。
   为什么必须这么做（实测证据）：车正常跑时遥测 GZ 读到 1520~1990°/s，
   而 0.82m/s 在 0.3m 半径上只有 ~157°/s —— 数值恰好差 131/16.4 ≈ **8 倍**，
   说明 ±2000dps 的写入【没生效】（芯片还在 ±250 默认档），而换算却按 16.4 走。
   后果：① IG 进圈判据(Σ|GZ|≥1000) 被任何弯道触发 → "在不在圈里"就不可信了
        ② USE_YAW 的"转到 50°"其实只转了 ~6° → 出圈那一下根本没转够
   下面这个初值只是兜底（±2000dps）。 */
static float gyro_lsb = 16.4f;
#define ACC_8G      4096.0f    /* ±8g 时加速度计灵敏度 LSB/g */

extern I2C_HandleTypeDef hi2c2;

static int16_t ax, ay, az;      /* 加速度计原始量 */
static int16_t gx, gy, gz;      /* 陀螺仪原始量 */
static float   gz_dps = 0.0f;   /* ★偏航率(度/秒)，已减零偏 */
static float   pitch = 0.0f, roll = 0.0f;
static uint8_t calibrated = 0;
static uint8_t present    = 0;  /* WHO_AM_I 自检通过 */
static float   gyro_bx = 0.0f, gyro_by = 0.0f, gyro_bz = 0.0f;  /* 三轴零偏 */

/* ============================================================================
 * ★★ 航向角（yaw）结算 —— 2026-09-23 新增
 *
 *  gz_dps 是【角速度】(°/s)，这里是把它【积分】成角度：
 *      yaw_deg += gz_dps × dt         (dt = CTRL_PERIOD_MS)
 *
 *  ⚠️ 关于漂移（必须知道，别指望它当"整圈航向"）：
 *     · pitch/roll 能用加速度计做互补滤波修正 —— 因为重力是个绝对参考。
 *     · **yaw 没有绝对参考**（加速度计测不出绕重力轴的转角，除非加磁力计）。
 *     · 所以 yaw 只能纯积分，残余零偏会累积：MPU6050 典型 ±0.05°/s → **约 20 秒漂 1°**。
 *     · 对"转 90° 只用 0.5 秒"这种【短时】用途，漂移 0.5s×0.05 = 0.025°，可忽略。
 *     · 想当整圈航向用 → 必须加磁力计或每圈重新对齐（本车没做）。
 *
 *  ★用法：转弯前 imu_yaw_reset()，然后读 imu_get_yaw() 看转了多少度。
 * ==========================================================================*/
static float   yaw_deg   = 0.0f;
static uint8_t yaw_valid = 0;     /* 1 = 积分有效（IMU 在、已标定） */
static uint32_t yaw_last_tick = 0;/* ★上一次积分的时间戳（用来算【实测 dt】，见 imu_update） */

static void imu_write_reg(uint8_t reg, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, reg, 1, &val, 1, IMU_I2C_TIMEOUT);
}

static uint8_t imu_read_reg(uint8_t reg)
{
  uint8_t v = 0;
  HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, reg, 1, &v, 1, IMU_I2C_TIMEOUT);
  return v;
}

static void imu_read_raw(void)
{
  uint8_t d[14];
  if (HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x3B, 1, d, 14, IMU_I2C_TIMEOUT) != HAL_OK) return;
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

/* ============================================================================
 * ★★★ 2026-09-26 【削顶计数器】—— 只观测，不改控制 ★★★
 *
 * 【为什么要它】实车日志（FW:0926-1800）里 `|GZ| >= 990` 出现 34/457 次（7.4%），
 *   其中 `GZ=-995` 重复 25 次 —— 反复落在同一个数 = 削顶铁证
 *   （真实测量不会反复等于同一个值）。而 995 ≈ 32767/32.8 正好是 ±1000dps 档满量程。
 *
 * 【为什么不能靠"换更宽的档位"解决】已试过并失败：
 *   · 09-24 写 0x18(±2000dps) → **写不进去**，芯片留在 ±250 默认档 → GZ 读数大 8 倍
 *   · 今天再写 0x18 → **GZ 恒为 0**（比写不进去更糟，已 revert）
 *   0x10(±1000) 与 0x18(±2000) 只差 FS_SEL 一个 bit，前者稳定可用、后者直接坏
 *   ⇒ **这颗芯片（大概率是国产 MPU6050 克隆）的 FS_SEL=3 位是坏的/未实现**，
 *     不是配置错误 ⇒ 【改代码修不了】，±1000dps 已是它给的最宽档位。
 *
 * 【所以改为先量清楚】在动手之前必须知道：削顶到底是不是葫芦圈的瓶颈？
 *   · 若只在"打滑自旋"时削顶，而正常穿越 S 形时不削 → 它不是瓶颈，别再折腾它
 *   · 若穿越过程中持续削顶 → 偏航积分确实少算，才值得为它设计软件补偿
 *   判据就是本计数器：遥测新增 `SAT=`（本拍是否削顶）与 `SATN=`（累计次数）。
 * ========================================================================== */
static uint32_t gz_sat_cnt   = 0u;   /* 累计削顶拍数 */
static uint8_t  gz_sat_now   = 0u;   /* 本拍是否削顶 */

uint8_t  imu_gz_saturated(void)     { return gz_sat_now; }
uint32_t imu_gz_sat_count(void)     { return gz_sat_cnt; }
void     imu_gz_sat_reset(void)     { gz_sat_cnt = 0u; }

void imu_update(void)
{
  imu_read_raw();
  gz_dps = (float)gz / gyro_lsb;
  if (calibrated) gz_dps -= gyro_bz;

  /* ★削顶判定：看【原始读数】|gz| 是否贴近满量程，而不是看 dps ——
     dps 会被 `- gyro_bz` 影响，原始值和量程是直接对应的。
     满量程 raw = 32767；取 97% = 31784 作阈值（真实测量几乎不会刚好贴着上限）。 */
  gz_sat_now = ((gz >= (int16_t)31784) || (gz <= (int16_t)-31784)) ? 1u : 0u;
  if (gz_sat_now && (gz_sat_cnt < 0xFFFFFFFFu)) gz_sat_cnt++;

  /* ★航向角积分
     ⚠️2026-09-23 修：原来用【固定 dt = CTRL_PERIOD_MS(10ms)】，这是错的。
        实际调用节奏是主循环里的时间戳节拍（now - t_ui >= UI_PERIOD_MS），
        而主循环会因为【遥测串口发送(约21ms/次)】和 IMU 的 I2C 读而抖动
        → 真实间隔可能 10、11、13、21ms 不等。
        用固定 10ms 积分 → 转 90° 的这几百毫秒里会累积 10~30% 的误差，
        而出口右转正是靠这个角度闭环停车的，误差直接变成"转不够/转过头"。
     改成【实测 dt】：每次调用用 HAL_GetTick() 的差值。
     ★另外把 dt 夹在 [1, 50]ms：时间戳异常/回绕时不会算出离谱的积分量。 */
  if (present && calibrated)
  {
    uint32_t now_t = HAL_GetTick();
    uint32_t dt_ms;
    if (yaw_last_tick == 0u) dt_ms = (uint32_t)CTRL_PERIOD_MS;   /* 第一次调用没有参考 */
    else                     dt_ms = now_t - yaw_last_tick;
    yaw_last_tick = now_t;

    if (dt_ms == 0u)       dt_ms = 1u;      /* 同一 ms 内被调两次 → 至少算 1ms */
    if (dt_ms > 50u)       dt_ms = 50u;     /* 异常/回绕保护 */

    /* ★正负号与方向无关：对称梯形积分用 (上一拍 + 这一拍)/2 更准。
       这里保留简单的矩形积分（10ms 级别足够），只用实测 dt 修正。 */
    yaw_deg  += gz_dps * ((float)dt_ms / 1000.0f);
    yaw_valid = 1u;
  }
}

/* ★当前航向角（度）。正 = 左转（右手系 Z），负 = 右转 */
float imu_get_yaw(void)
{
  return yaw_deg;
}

/* ★把航向角清零（转弯前调用，之后读到的就是"相对这个时刻转了多少度"）
   ★同时复位时间戳：否则下一次积分的 dt 会是"上一次 imu_update 到现在"的旧间隔，
     而中间可能隔了很久（比如用户按键、停车），会把一大段角度一次性积进来。 */
void imu_yaw_reset(void)
{
  yaw_deg       = 0.0f;
  yaw_last_tick = 0u;      /* 0 = 下次调用按一个标准周期算 */
}

/* ★航向角是否可信（IMU 在 + 已标定）。0 = 别用它做控制, 走兜底 */
uint8_t imu_yaw_is_valid(void)
{
  return yaw_valid;
}

void imu_init(void)
{
  imu_write_reg(0x6B, 0x00);   /* PWR_MGMT_1: 唤醒 */
  /* ★★★ 2026-09-24 晚：唤醒后延时 + 量程写后校验 + 重试 ★★★
     实车日志证据（FW:0924-2615）：遥测 GZ 反复出现【同一个值 -247】（还有 +250）——
     那是【削顶】的签名：±250dps 档满量程 = 32767/131 ≈ 250。
     说明 0x1B 的写入【没生效】，芯片留在 ±250 默认档；
     车转速一超过 250°/s，读数就被钉住 → 航向角积分【少算】→
     出圈"转到 85°"实际会转过头。这也是历史上一串"转过头/转不够"的来源之一。
     下面三句：① 等起振稳定再写；② 写完读回 FS_SEL 校验；③ 不对就重写一次。
     ★末尾"读回 FS_SEL 选 gyro_lsb"那段照旧保留 —— 兜底，
       保证即使写入始终不生效，换算也永远和实际档位一致。 */
  HAL_Delay(50);
  imu_write_reg(0x1A, 0x00);   /* CONFIG: 关闭DLPF */
  /* ★GYRO_CONFIG: 0x10 = FS_SEL=2 = **±1000 dps**
     ⚠️ 原注释写"±2000dps"是错的（±2000 要 FS_SEL=3 → 0x18）。
     这里【故意保留 ±1000】：灵敏度 32.8 LSB/(°/s)，分辨率比 ±2000 高一倍，
     而 ±1000°/s 对循迹车完全够（实测急转也就 250~300°/s，±250 档会削顶）。 */
  imu_write_reg(0x1B, 0x10);
  HAL_Delay(10);
  if (((imu_read_reg(0x1B) >> 3) & 0x03u) != 2u)
  {
    HAL_Delay(50);                     /* 再给一次起振时间，然后重写 */
    imu_write_reg(0x1B, 0x10);
    HAL_Delay(10);
  }
  imu_write_reg(0x1C, 0x10);   /* ACCEL_CONFIG: ±8g */
  imu_write_reg(0x1D, 0x00);   /* 加速度计低通 */
  /* ★2026-09-24 读回 GYRO_CONFIG 的 FS_SEL(bit4:3)，按【实际生效】的档位选灵敏度
     —— 见文件顶部 gyro_lsb 的说明：实测 GZ 比物理值大 8 倍 = 档位与换算不一致，
     这一句就是修它的（写入没生效时不至于把 GZ 放大 8 倍）。 */
  {
    uint8_t fs = (uint8_t)((imu_read_reg(0x1B) >> 3) & 0x03u);
    switch (fs)
    {
      case 0u: gyro_lsb = 131.0f; break;   /* ±250  dps */
      case 1u: gyro_lsb =  65.5f; break;   /* ±500  dps */
      case 2u: gyro_lsb =  32.8f; break;   /* ±1000 dps */
      default: gyro_lsb =  16.4f; break;   /* ±2000 dps（也是读不到时的兜底） */
    }
  }
  calibrated = 0;
  pitch = 0.0f; roll = 0.0f;
  gz_dps = 0.0f;
  /* ★★★ 2026-09-24 修 present：WHO_AM_I 只读一次，读挂了就永远是 0 ★★★
     实车证据（交接文档第三节）：遥测出现 `WHO=70 / IMU=0` —— WHO_AM_I 读到 0x70
     而不是 0x68 → present=0 → app_init 里 `if (imu_is_present()) imu_calibrate();`
     被跳过 → **陀螺零偏没标定** → GZ 带直流偏置 →
     相切点判据的"GZ ±8 拍内翻号"被这点偏移+噪声满足 → 圈外误报。
     两道修：① 重试 3 次（I2C 偶发失败很常见）；② 仍失败就用"原始数据能不能读回来"兜底。
     ★为什么兜底可信：imu_read_raw() 失败时会【提前 return】、全局量保持不动，
       而静止时 az 必然含着重力分量 ≠ 0 → 只要读到一个非 0 值就说明 I2C 通了。 */
  {
    uint8_t ok = 0u;
    for (uint8_t k = 0u; k < 3u; k++)
    {
      if (imu_who_am_i() == 0x68u) { ok = 1u; break; }
      HAL_Delay(5);
    }
    if (!ok)
    {
      imu_read_raw();                      /* 失败会提前返回，全局量不变 */
      if ((ax != 0) || (ay != 0) || (az != 0) || (gz != 0)) ok = 1u;
    }
    present = ok;
  }
}

/* 三轴陀螺零偏: 静止采样 100 次取平均(开机时调用一次) */
void imu_calibrate(void)
{
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 100; i++)
  {
    imu_read_raw();
    sx += (float)gx / gyro_lsb;
    sy += (float)gy / gyro_lsb;
    sz += (float)gz / gyro_lsb;
  }
  gyro_bx = sx / 100.0f;
  gyro_by = sy / 100.0f;
  gyro_bz = sz / 100.0f;
  calibrated = 1;
}

void imu_read_angles(float* pitch_out, float* roll_out)
{
  imu_read_raw();
  float gxp = (float)gx / gyro_lsb;
  float gyp = (float)gy / gyro_lsb;
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
float   imu_get_yaw(void) { return 0.0f; }
void    imu_yaw_reset(void) { }
uint8_t imu_yaw_is_valid(void) { return 0u; }
uint8_t  imu_gz_saturated(void) { return 0u; }
uint32_t imu_gz_sat_count(void) { return 0u; }
void     imu_gz_sat_reset(void) { }
#endif
