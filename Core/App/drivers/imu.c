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

/* ★2026-09-26 晚【DLPF 读回】：实时读 CONFIG(0x1A)。
   为什么必须能让外面看到：判断"打开 DLPF 有没有用"时，只看 Na 会二义 ——
     Na 没变既可能是"污染不是混叠"，也可能是"0x1A 根本没写进去"。
   这颗克隆有写入不生效的前科（写 0x18 到 0x1B 让 GZ 恒 0），所以必须分开。
   遥测 IMUn 行末尾的 `C=xx` 就是这个值：**C=03 = DLPF 生效；C=00 = 写入失败**。 */
uint8_t imu_get_cfg(void)
{
  return imu_read_reg(0x1A);
}

float imu_get_gyro_z(void)
{
  return gz_dps;                 /* 度/秒，正 = 左转（右手系绕 Z） */
}

/* ★★★ 2026-09-26 晚【遥测用：六轴原始量 + 纯加速度计角度】★★★
   实现说明见 imu.h 的声明处。两点必须写清楚：
   ① 这里【没有任何滤波】，是故意的 —— 用户要看的就是"抖得多厉害"。
   ② 角度用 atan2f 算：pitch 绕 Y（前后俯仰），roll 绕 X（左右侧倾）。
      分母做了保护（sqrtf 结果 < 1 时取 1），避免 fz≈0 时除零。
   ③ 这两个函数【只读 static，不碰 I2C、不碰状态】→ 可以在任何时刻安全调用。 */
void imu_get_raw(int16_t a[3], int16_t g[3])
{
  a[0] = ax; a[1] = ay; a[2] = az;
  g[0] = gx; g[1] = gy; g[2] = gz;
}

void imu_get_acc_angles(float* pitch_deg, float* roll_deg)
{
  float fx  = (float)ax;
  float fy  = (float)ay;
  float fz  = (float)az;
  float nyz = sqrtf(fy * fy + fz * fz);
  if (nyz < 1.0f) nyz = 1.0f;
  *pitch_deg = atan2f(-fx, nyz) * RAD_TO_DEG;
  *roll_deg  = atan2f( fy, (fz != 0.0f) ? fz : 1.0f) * RAD_TO_DEG;
}

/* ============================================================================
 * ★★★ 2026-09-26 晚【遥测用：100Hz 一阶低通 + 污染度量】★★★
 *
 * 【为什么要它】用户要求："我要看的是滤波以后那些数据的污染值严不严重，
 *   看它对小车的不良影响程度"。
 *   光给原始量只能证明"有污染"（实测 |a| 尖峰到 4.7g），但回答不了"影响多大"。
 *   要回答影响，必须看【滤波之后还剩多少抖动】——
 *   因为滤波后的量才是真正会进入角度/姿态判断的量。
 *
 * 【给三样东西】
 *   ① fa/fg      = 100Hz 低通后的六轴        → "滤波以后的数据"
 *   ② res_a/res_g= 平均|原始 − 滤波|（三轴平均）→ "滤掉了多少"（污染强度）
 *   ③ pitch/roll = 用【滤波后】加速度算的角度  → "还剩多少"（对车的实际影响）
 *
 * 【低通系数】一阶 IIR： y += a·(x − y)，a = IMU_LPF_ALPHA_X100/100。
 *   在 100Hz 下，−3dB 截止 ≈ a·fs/(2π)：
 *     a=0.1 → ~1.6Hz   a=0.2 → ~3.2Hz   a=0.3 → ~4.8Hz   a=0.5 → ~8Hz
 *   默认取 0.25（≈4Hz）—— 车身姿态这种慢量用 4Hz 足够，
 *   而 4.7g 那种几十~上百 Hz 的机械冲击正好会被它滤掉大半。
 *
 * 【读-清语义】imu_get_filtered() 读完就把统计清零 ⇒ 调用周期 = 统计周期。
 *   本项目由遥测每 200ms 调一次 ⇒ 稳定期内的 20 个采样。
 *
 * 【只读，不参与控制】控制路径仍用未滤波的原始量（imu_get_gyro_z / imu_get_yaw）。
 *   本块【不动】gz_dps / yaw_deg / pitch / roll 任何一个。
 * ==========================================================================*/
static float    tf[6]    = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t  tf_init  = 0u;
static uint32_t tf_sa    = 0u;      /* Σ|原始−滤波| 加速度 */
static uint32_t tf_sg    = 0u;      /* Σ|原始−滤波| 陀螺   */
static uint32_t tf_n     = 0u;      /* 本窗口采样数 */

/* 把滤波后的三轴加速度换算成角度（与 imu_get_acc_angles 同一套公式，只是换成滤波值） */
static void tf_angles(const float* f, float* pitch_deg, float* roll_deg)
{
  float nyz = sqrtf(f[1] * f[1] + f[2] * f[2]);
  if (nyz < 1.0f) nyz = 1.0f;
  *pitch_deg = atan2f(-f[0], nyz) * RAD_TO_DEG;
  *roll_deg  = atan2f( f[1], (f[2] != 0.0f) ? f[2] : 1.0f) * RAD_TO_DEG;
}

void imu_get_filtered(int16_t fa[3], int16_t fg[3],
                      int16_t* res_a, int16_t* res_g,
                      float* pitch_deg, float* roll_deg)
{
  fa[0] = (int16_t)tf[0]; fa[1] = (int16_t)tf[1]; fa[2] = (int16_t)tf[2];
  fg[0] = (int16_t)tf[3]; fg[1] = (int16_t)tf[4]; fg[2] = (int16_t)tf[5];
  if (tf_n > 0u)
  {
    uint32_t ma = tf_sa / tf_n;
    uint32_t mg = tf_sg / tf_n;
    *res_a = (int16_t)((ma > 30000u) ? 30000u : ma);
    *res_g = (int16_t)((mg > 30000u) ? 30000u : mg);
  }
  else
  {
    *res_a = 0; *res_g = 0;
  }
  tf_angles(tf, pitch_deg, roll_deg);
  tf_sa = 0u; tf_sg = 0u; tf_n = 0u;      /* ★读-清：下一窗口重新统计 */
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

  /* ★★★ 2026-09-26 晚【遥测用低通 + 残差统计】—— 只读，不参与控制 ★★★
     详细说明见上面 tf[] 那一段的注释。这里只做三件事：
       ① 六轴各跑一遍一阶低通（首拍直接跟随，避免从 0 慢慢爬）
       ② 累加 |原始 − 滤波|（三轴平均）→ 后面 imu_get_filtered() 取平均
       ③ 累计采样数
     ★控制用的 gz_dps / yaw_deg / pitch / roll 一个都不动。 */
  {
    const float al = (float)IMU_LPF_ALPHA_X100 / 100.0f;
    int16_t r6[6];
    int16_t k;

    r6[0] = ax; r6[1] = ay; r6[2] = az;
    r6[3] = gx; r6[4] = gy; r6[5] = gz;

    for (k = 0; k < 6; k++)
    {
      if (tf_init == 0u) tf[k]  = (float)r6[k];                   /* 首拍跟随 */
      else               tf[k] += al * ((float)r6[k] - tf[k]);    /* 一阶低通 */
    }
    tf_init = 1u;

    /* 三轴平均残差（先求和再除 3，避免逐轴截断误差） */
    tf_sa += (uint32_t)((fabsf((float)r6[0] - tf[0]) +
                         fabsf((float)r6[1] - tf[1]) +
                         fabsf((float)r6[2] - tf[2])) / 3.0f);
    tf_sg += (uint32_t)((fabsf((float)r6[3] - tf[3]) +
                         fabsf((float)r6[4] - tf[4]) +
                         fabsf((float)r6[5] - tf[5])) / 3.0f);

    /* 上限保护：万一遥测长时间不调用（比如没发车），也不会把累加器撑爆 */
    if (tf_n < 60000u) tf_n++;
    if (tf_sa > 2000000000u) tf_sa = 2000000000u;
    if (tf_sg > 2000000000u) tf_sg = 2000000000u;
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
  /* ★★★ 2026-09-26 晚【打开 DLPF：0x00(关闭) → 0x03】★★★
   *
   * 【为什么改】用户要求查"行车时污染有多严重"。静止/行车对比实测（FW:0926-G66）：
   *     静止本底：Na = 4~26    Ng = 0~13     az = 4100~4180（=1.01g，标定正确）
   *     行车    ：Na = 637~2799 Ng = 267~1096   |a| 尖峰到 4.7g
   *   ⇒ 污染放大 ~100 倍。
   *
   * 【怀疑这里就是主因】原来 0x1A = 0x00 = DLPF【关闭】⇒
   *   加速度计带宽 260Hz、陀螺 256Hz；而我们只用 100Hz 去读最新寄存器
   *   ⇒ **混叠（aliasing）**：260Hz 带宽里的高频振动被折叠进 0~50Hz，
   *      看起来就像"车真的在 4.7g 抖动"。
   *   这解释了为什么"低通滤波救不了"—— 干扰在采样那一刻就已经折进来了，
   *   后置数字滤波只能滤掉折进来之后落在高频的那部分。
   *
   * 【取值 0x03】DLPF_CFG=3（MPU6050 标准档）：
   *     加速度 44Hz 带宽 / 延迟 4.9ms ；陀螺 42Hz 带宽 / 延迟 4.8ms
   *   陀螺内部输出率变为 1kHz（>我们的 100Hz 采样，不会漏样）。
   *   选 3 而不是更狠的 6(5Hz)：延迟 4.9ms 在 10ms 控制周期里可接受，
   *   而 5Hz 档延迟 19ms 会明显拖慢陀螺对急转的响应（出圈转角要用它）。
   *
   * 【读回校验+重试】照抄上面 GYRO_CONFIG 那段的做法 ——
   *   这颗芯片【写入不生效】有前科（写 0x18 到 0x1B 直接让 GZ 恒 0）。
   *   写完读回确认 DLPF_CFG 位，不对就再写一次。
   *   ⚠️ 若这次读回【始终不对】，说明这颗克隆的 0x1A 也是坏的 ——
   *      那就把本行改回 0x00（行为 = 0926-G67），并转去做机械减震。
   *
   * 【怎么验证有没有生效】看遥测 IMUn 的 Na/Ng：
   *   静止时应从 ~5/1 变化不大；**行车时若 Na 从上千掉到几百** ⇒ 混叠是主因，有效；
   *   若 Na 基本不变 ⇒ 不是混叠，是真实机械冲击，代码无解，只能改结构。
   * 【回退】本行改回 0x00。 */
  /* ★★★ 2026-09-26 晚【DLPF 实验已做完：写入不生效，改回 0x00】★★★
     试过 0x03（加速度 44Hz / 陀螺 42Hz），FW:0926-G69 全程读回 `C=00`
     ⇒ 这颗克隆芯片的 0x1A 也是坏的，芯片内的滤波用不了。
     回退到 0x00（行为 = 0926-G67），不再多花启动时间在无用的重试上。
     ★结论记在这里，别再试第三次。 */
  imu_write_reg(0x1A, 0x00);   /* CONFIG: DLPF 关闭（实测写不进去，见上） */
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

  /* 互补滤波: 角度 = 0.98*(角度+陀螺角速度*dt) + 0.02*角度(加速度计)
     ★★★ 2026-09-26 修：原来 dt 是【硬编码 0.010f】，那是错的 ★★★
     和 yaw 积分（本文件 imu_update 里）当年犯的是【同一个毛病】：
       真实调用间隔并不等于 10ms —— 主循环会因【遥测串口发送(约21ms/次)】和
       IMU 的 I2C 读而抖动，实测 10、11、13、21ms 不等。
       真实间隔 21ms 时按 10ms 积分 → 陀螺贡献【少算一半】
       → 快速运动时姿态会被加速度计拽偏（等效互补系数从 0.98 变成 ~0.99 还偏）。
     yaw 那边早已改用实测 dt（见 imu_update 的注释），pitch/roll 这里漏改了，
     现在补齐：用本函数自己的时间戳算 dt，并夹在 [1, 50]ms 防异常/回绕。
     ★接口不变（仍无参），时间戳是文件内静态量 —— 不牵动调用方。 */
  {
    static uint32_t ang_last_tick = 0u;
    uint32_t now_t = HAL_GetTick();
    float dt;
    if (ang_last_tick == 0u) dt = (float)UI_PERIOD_MS / 1000.0f;   /* 首次无参考 */
    else                     dt = (float)(now_t - ang_last_tick) / 1000.0f;
    ang_last_tick = now_t;
    if (dt <= 0.0f)  dt = (float)UI_PERIOD_MS / 1000.0f;
    if (dt >  0.05f) dt = 0.05f;                                   /* 异常/回绕保护 */

    pitch = 0.98f * (pitch + gyp * dt) + 0.02f * pitch_acc;
    roll  = 0.98f * (roll  + gxp * dt) + 0.02f * roll_acc;
  }

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
/* ★2026-09-26 晚：遥测用的两个新接口在 USE_IMU=0 时也要有定义（否则链接失败）。
   返回全 0 —— 遥测会打出 "IMUa 0 0 0 | 0 0 0"，一眼就能看出"IMU 没编进来"。 */
void imu_get_raw(int16_t a[3], int16_t g[3])
{
  a[0] = 0; a[1] = 0; a[2] = 0;
  g[0] = 0; g[1] = 0; g[2] = 0;
}
void imu_get_acc_angles(float* pitch_deg, float* roll_deg)
{
  *pitch_deg = 0.0f;
  *roll_deg  = 0.0f;
}
/* ★2026-09-26 晚：滤波版接口在 USE_IMU=0 时也要有定义（否则链接失败）。
   返回全 0 且残差 = -1 —— 遥测打出来是 "Na=   -1 Ng=   -1"，
   一眼就能区分"IMU 没编进来"和"读数真的很干净(0)"。 */
void imu_get_filtered(int16_t fa[3], int16_t fg[3],
                      int16_t* res_a, int16_t* res_g,
                      float* pitch_deg, float* roll_deg)
{
  fa[0] = 0; fa[1] = 0; fa[2] = 0;
  fg[0] = 0; fg[1] = 0; fg[2] = 0;
  *res_a = -1; *res_g = -1;
  *pitch_deg = 0.0f;
  *roll_deg  = 0.0f;
}
uint8_t  imu_gz_saturated(void) { return 0u; }
uint32_t imu_gz_sat_count(void) { return 0u; }
void     imu_gz_sat_reset(void) { }
#endif
