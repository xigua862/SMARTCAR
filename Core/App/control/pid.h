#ifndef PID_H
#define PID_H

#include <stdint.h>

/* 通用 PID(带微分先行/抗积分饱和/输出限幅)。
 * ★★★ 2026-09-26 晚【改成增量式（用户方案）】★★★
 *   原来用位置式：`u = Kp·e + integral + Kd·d + bias`，其中 integral 是浮点累计量。
 *   病根：卡住时【内轮被 PWM 下限抬着达不到目标】→ 误差长期存在 →
 *         integral 一路顶到 i_max(50) → 叠加前馈后输出被打到 ±99 两边极限。
 *         表现出来就是用户说的"转向突变"：**指令算出来是温和的，电机收到的是满舵**。
 *
 *   增量式：`Δu = Kp(e_k − e_{k−1}) + Ki·e_k·dt + Kd·(...)`，`u_k = u_{k−1} + Δu + bias_k`
 *     · 结构上【没有积分累计量】→ 不会积分饱和
 *     · 输出只能按 Δu 逐拍变化 → 天然有速率限制
 *     · 仍然闭环 → 保留电压补偿（这正是关掉闭环后车"没力气"的原因）
 *   ★bias（前馈）仍然【立刻生效、不进累加器】：它就是"目标 PWM"，
 *     必须每拍跟踪目标；若把它也做成增量，起步/换向时会严重滞后。
 *   ★保护：i_max 不再是"积分限幅"，而是改成【累加器相对前馈的偏移上限】，
 *     防止累加器长期贴在极限上（否则工况一变就恢复不过来）。
 */
typedef struct {
  float  kp, ki, kd;   /* 比例/积分/微分 */
  float  prev_meas;    /* 上一次测量值(微分先行, 避免设定值突变冲击) */
  float  prev_err;     /* ★增量式：上一次误差 */
  float  acc;          /* ★增量式：累加器(= 输出 − 前馈)。取代原来的 integral */
  float  out_min;      /* 输出下限 */
  float  out_max;      /* 输出上限 */
  float  i_max;        /* ★偏移上限：|acc| 不得超过此值（原来叫积分限幅） */
  float  bias;         /* 输出偏置/前馈(可选) */
  uint8_t first;       /* 首拍标志(微分初值) */
} pid_t;

void pid_init(pid_t* p, float kp, float ki, float kd, float out_min, float out_max, float i_max);
void pid_reset(pid_t* p);

/* 一步更新: 返回限幅后的控制输出。dt 为控制周期(秒)。
   ★增量式：内部保存"上一次输出/误差"，不要在外面另做差分。 */
float pid_update(pid_t* p, float setpoint, float meas, float dt);

#endif /* PID_H */
