#include "odom.h"
#include "app_config.h"

#if USE_ENCODER
#include "drivers/encoder.h"
#include "drivers/motor.h"     /* MOTOR_LEFT / MOTOR_RIGHT */
#endif

/* ---- 换算：脉冲 -> 毫米 (与 app_config.h 的注释保持一致) ----
   驱动轮直径 66mm（新车设计书：6.6cm）→ 周长 = π×66 = 207.345mm
   ENCODER_PPR = 422 计数/轮圈
   → 1 计数 = 207345 µm / 422 = 491.3 µm
   ★用【微米】做中间单位、最后再除 1000：
     这样不会因为整数除法提前把小数丢掉（毫米整数单位误差太大）。 */
#define ODOM_UM_PER_COUNT   491        /* ≈491.3 µm/计数 */
#define UM_PER_MM           1000

static int32_t  s_prev_l   = 0;        /* 上一拍左轮计数 */
static int32_t  s_prev_r   = 0;
static int64_t  s_net_um   = 0;        /* 净位移(µm), 带符号 */
static int64_t  s_path_um  = 0;        /* 累计路程(µm), 只加不减 */
static int64_t  s_l_um     = 0;        /* 左轮净位移(µm) */
static int64_t  s_r_um     = 0;        /* 右轮净位移(µm) */
static uint8_t  s_inited   = 0;

void odom_init(void)
{
#if USE_ENCODER
  s_prev_l = (int32_t)encoder_get_count(MOTOR_LEFT);
  s_prev_r = (int32_t)encoder_get_count(MOTOR_RIGHT);
#else
  s_prev_l = 0;
  s_prev_r = 0;
#endif
  s_net_um  = 0;
  s_path_um = 0;
  s_l_um    = 0;
  s_r_um    = 0;
  s_inited  = 1u;
}

void odom_reset(void)
{
  odom_init();
}

void odom_update(void)
{
#if USE_ENCODER
  int32_t cur_l, cur_r, d_l, d_r;
  int64_t step_um;

  if (!s_inited) { odom_init(); return; }

  cur_l = (int32_t)encoder_get_count(MOTOR_LEFT);
  cur_r = (int32_t)encoder_get_count(MOTOR_RIGHT);

  /* ★关键：用【带符号的 16 位差】。
     写成 (int16_t)(cur - prev) 就能天然处理 0..65535 的回绕：
     比如 prev=65530、cur=5 → 差 = 11，而不是 -65525。
     （TIM3/TIM4 是 16 位，Period=65535，一定会回绕。） */
  d_l = (int32_t)(int16_t)((uint16_t)cur_l - (uint16_t)s_prev_l);
  d_r = (int32_t)(int16_t)((uint16_t)cur_r - (uint16_t)s_prev_r);

  s_prev_l = cur_l;
  s_prev_r = cur_r;

  /* 单轮净位移（遥测 L= / R=）。乘方向符号只为让两个读数符号一致；
     ★它【不再影响】车体里程（见下面 step_um 的说明）。
     符号怎么定见 app_config.h 的 ODOM_SIGN_L / ODOM_SIGN_R。 */
  s_l_um += (int64_t)ODOM_SIGN_L * (int64_t)d_l * ODOM_UM_PER_COUNT;
  s_r_um += (int64_t)ODOM_SIGN_R * (int64_t)d_r * ODOM_UM_PER_COUNT;

  /* ==========================================================================
   * ★★★ 车体前进量 —— 2026-09-23 改成【取绝对值】★★★
   *
   *  实车抓到的 bug（用户实测）：架空后单独转两个轮子，**里程互相抵消**、
   *  OD 几乎不涨。原因：正交解码方向取决于 A/B 相【接线】，
   *  某一轮接反了 → 车往前走时那一轮计数是【减少】的 →
   *  旧的 `(d_l + d_r) / 2` 一正一负**正好抵消**。
   *  剩下的只有 (d_l - d_r)/2，而它正比于"车【转】了多少"，不是"走了多远" ——
   *  所以当时读到的 403/400mm **根本不是里程**（修好后实测全程 = 24847mm）。
   *
   *  修法：中心前进量直接用两轮转动量的【绝对值】求平均。
   *    ✔ 与接线方向完全无关 → 不用标定符号，也不会再猜错
   *    ✔ 量的就是"轮子转了多少" = 车走了多少【路】，
   *      正是"从发车到葫芦圈出口走了多远"要的那个量
   *    ⚠️ 代价：车如果【倒退】，它仍然当正数累加（净位移语义丢失）。
   *       对这个用途来说这恰恰是对的（要的是路程，不是位移）。
   *       要恢复带符号的净位移：把下面两行的 abs 去掉，
   *       并把 ODOM_SIGN_L / ODOM_SIGN_R 按实车标定好。
   * ========================================================================== */
  {
    int64_t a_l = (d_l < 0) ? -(int64_t)d_l : (int64_t)d_l;
    int64_t a_r = (d_r < 0) ? -(int64_t)d_r : (int64_t)d_r;
    step_um = (a_l + a_r) * ODOM_UM_PER_COUNT / 2;
  }
  s_net_um  += step_um;      /* 现在恒 >= 0：只增不减 */
  s_path_um += step_um;
#endif
}

int32_t odom_distance_mm(void)
{
  return (int32_t)(s_net_um / UM_PER_MM);
}

int32_t odom_path_mm(void)
{
  return (int32_t)(s_path_um / UM_PER_MM);
}

int32_t odom_left_mm(void)
{
  return (int32_t)(s_l_um / UM_PER_MM);
}

int32_t odom_right_mm(void)
{
  return (int32_t)(s_r_um / UM_PER_MM);
}
