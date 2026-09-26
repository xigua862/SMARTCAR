#include "pid.h"

/* ============================================================================
 * ★★★ 2026-09-26 晚【改成增量式 PID（用户方案）】★★★
 *
 * 【为什么改】原来用位置式：`u = Kp·e + integral + Kd·d + bias`。
 *   实车病根（黑匣子 FW:0926-TRACE 实证）：卡住那 15 拍里右轮被钉在 −1~−21
 *   （倒转）、左轮 66~99，而按 PD 算只该给出 ≈−14 的修正（右轮该是 ≈38）。
 *   唯一解释是【积分顶满】：内轮被 WHEEL_PWM_FLOOR 抬着 → 达不到目标转速 →
 *   误差长期存在 → integral 一路顶到 i_max(50) → 叠加前馈后输出被打到 ±99。
 *   ⇒ 这就是用户说的"转向突变"：**指令是温和的，电机收到的是满舵。**
 *
 * 【增量式的三条好处，逐条对上我们的病】
 *   ① 结构上【没有积分累计量】→ 不可能积分饱和
 *   ② 输出只能按 Δu 逐拍变化 → 天然自带速率限制（不必再加 slew limiter）
 *   ③ 仍然闭环 → 保留电压补偿
 *      （反面教材：试过直接关掉闭环 → 命令 24 PWM 只跑出 15 PWM 的转速，
 *        车"没力气"，葫芦圈还是过不去）
 *
 * 【公式】Δu = Kp(e_k − e_{k−1}) + Ki·e_k·dt + Kd(e_k − 2e_{k−1} + e_{k−2})
 *          u_k = u_{k−1} + Δu + bias_k
 *
 * 【两个刻意的设计决定】
 *   ① bias（前馈）【立刻生效、不进累加器】
 *      它就是"目标 PWM"（见 line_follow.c 的 SPD_USE_FEEDFORWARD），必须每拍
 *      紧跟目标；若把 bias 也做成增量，起步/换向时输出会严重滞后于目标 → 车发软。
 *      所以：acc 只累计 PID 的修正量，bias 直接叠加到最终输出上。
 *   ② i_max 改成【累加器相对前馈的偏移上限】
 *      增量式虽然不会"饱和"，但累加器仍可能长期贴在一个大值上（例如内轮被
 *      下限抬着、误差恒定）→ 工况一变就恢复不过来。加了这一道，
 *      acc 恒被拉回 ±i_max 内 ⇒ 输出最多偏离前馈 i_max，不可能被推到满舵。
 *
 * 【回退】要回到位置式：git 里有旧版 pid.c；或把本函数体换成
 *      u = Kp·e + integral + Kd·d + bias 的老写法即可。
 * ==========================================================================*/

void pid_init(pid_t* p, float kp, float ki, float kd, float out_min, float out_max, float i_max)
{
  p->kp = kp; p->ki = ki; p->kd = kd;
  p->out_min = out_min; p->out_max = out_max; p->i_max = i_max;
  p->bias = 0.0f;
  pid_reset(p);
}

void pid_reset(pid_t* p)
{
  p->acc       = 0.0f;
  p->prev_meas = 0.0f;
  p->prev_err  = 0.0f;
  p->first     = 1;
}

float pid_update(pid_t* p, float setpoint, float meas, float dt)
{
  float err = setpoint - meas;

  /* ---- 增量式计算 ---- */
  /* P 项增量：只与误差的【变化】有关（这正是"逐渐增加"的来源）*/
  float d_p = p->kp * (err - p->prev_err);
  /* I 项增量：误差存在就持续加，但每次只加一点点 */
  float d_i = p->ki * err * dt;
  /* D 项增量：用测量值差分（微分先行），首拍为 0，避免设定值跳变冲击 */
  float d_d = 0.0f;
  if (!p->first)
  {
    d_d = -p->kd * (meas - p->prev_meas) / dt;
  }

  p->prev_err  = err;
  p->prev_meas = meas;
  p->first     = 0;

  /* ---- 累加器更新 + 相对前馈的偏移限幅（抗漂移，见文件头 ②）---- */
  p->acc += d_p + d_i + d_d;
  if (p->acc >  p->i_max) p->acc =  p->i_max;
  if (p->acc < -p->i_max) p->acc = -p->i_max;

  /* ---- 输出 = 前馈(立刻生效) + 累加器 ---- */
  float out = p->bias + p->acc;
  if (out > p->out_max) out = p->out_max;
  if (out < p->out_min) out = p->out_min;
  return out;
}
