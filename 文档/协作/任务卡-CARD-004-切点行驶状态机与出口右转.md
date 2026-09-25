# 任务卡-CARD-004-切点行驶状态机与出口右转（问题二·修订版）
- 日期：2026-09-25　写卡人：军师
- **注：本卡取代同日早前版**（切点行驶按用户方案重构）。若工人已按早前版开工：停手，以本卡为准。
- 背景：CARD-003 只解决切点"识别"（GW 三连对齐过完葫芦），"行驶"仍是 15 拍翻转脉冲+PD 蒙（用户目击裁定：顺序过圆=运气）。本卡把切点行驶改成受控状态机，并接上出口右转。
- 目标：车**受控**按序过 4 圆（每切点"进→出→转向"三步全有日志），GW 到 6 后减速，出口右转 85° 出圈，过直角弯，弯后直行 100 拍退出葫芦状态。

---

## 数据依据（工人不用查）

- run4 首过 3 切点**单打**三连（OD 1731/2425/3309）→ "出切点"不能靠第二次 TANGENT（首过不发生），须独立事件 = **切点宽图案结束**。
- run4 切点序列：宽贴一端 → 翻号 → 左2路(active=2) → 全灭 → 扫回 —— 宽结束后 active 跌破 5 稳定可见，是可靠的"出"标志。
- run4 绕圈重过切点出现 ~220mm 双打 ×5 对 → 不加配对锁存 GW 会被数爆。
- run2 出口段（OD 19896-19917）：IR `00111001→00011011→00011111→00011110`，GZ 全正**无翻号无全灭**，之后直接回普通线 → 出口判据 = 宽贴一端 + 候选期满未翻号 + 期内未全灭（与切点天然互斥）。

## 状态机总览（一图流）

```
IG=1 后开始数 GW（圈外 TANGENT 原有 ge_in_gourd 门不变）
  TANGENT 确认 ──→ GW 奇数 ++（进切点），锁存 pending，打印 side
       │
  宽图案结束(连续2拍 active<5) ──→ GW 偶数 ++（出切点）
       │                           + 切点序号 s_tan_index++
       │                           + 转向窗口：序号奇→左转 / 偶→右转（15拍×幅度3）
       ▼
  GW==6 ──→ 归零 + ge_done=1 ──→ 减速 65% ──→ 等出口特征（NOFLIP）
       │
  出口确认 ──→ 右转 85°（角度闭环）──→ ge_exit_done=1 ──→ 过直角弯
       │
  弯后连续 100 拍普通线 ──→ 清 IG 全套，回普通循迹
```

---

## Part A · 切点行驶状态机（line_follow.c + app_config.h）

### A1. app_config.h 葫芦块宏改动
```c
#define GOURD_TOTAL          6    /* 3→6：每切点计2（进+出），4圆3切点×2 */
#define GOURD_TANOUT_CONFIRM 2    /* "宽图案结束"确认拍数：连续2拍 active<CROSS_ACTIVE_MIN */
#define GOURD_TANOUT_TIMEOUT 30   /* pending 悬空兜底：30拍(300ms)未见宽结束→放弃不++ */
```
【不动】：`GOURD_FLIP_CYCLES(15)`、`GOURD_FLIP_ERR(3)`、`GOURD_WAVE_MIN_MM(200)`、`GOURD_NO_CROSS(1)`、`WIDE_ONE_GO_STRAIGHT(0)`、`GOURD_EXIT_TURN_PWM/DEG/SETTLE_FRAMES/TURN_DIST/TURN_FRAMES/COOLDOWN`、`GOURD_EXIT_USE_YAW(1)`、其余出圈触发开关保持 0。

### A2. 新静态变量（照 `gourd_wrap_evt` 模式：声明/复位/使用同进同出，放 `ge_confirmed` 那组旁边）
```c
static uint8_t  s_tan_pending = 0;  /* TANGENT 后等待"出切点"的锁存位 */
static uint8_t  s_gap_cnt     = 0;  /* 宽结束连续拍数计数 */
static uint8_t  s_tan_timeout = 0;  /* pending 悬空计时 */
static uint8_t  s_tan_index   = 0;  /* 切点序号 1..3（仅用于转向方向交替） */
```
复位点三处同步清（与 CARD-003 的四变量同处）：`s_ig_leave` 清 GW 处、每处 IG 清 0、`line_follow_init()`。

### A3. TANGENT 确认块改动（现有 `gourd_waves++` 处）
1. 计数条件加两道：`if (ge_in_gourd && !ge_done && (gourd_waves < 200u))`（ge_done=1 后冻结，防第四圆/出口区再数）。
2. `gourd_waves++` 后追加：`s_tan_pending = 1u; s_gap_cnt = 0u; s_tan_timeout = 0u;`
3. **删除旧的方向赋值两行**（`gourd_dir = (last_error >= 0) ? 1 : -1;` 和 `gourd_flip = GOURD_FLIP_CYCLES;`）——方向来源改 A4，转向起点从 TANGENT 挪到出切点。
4. 打印行加 side（贴哪端，wide_one 判定时的 tL/tR 现成）：
   `"GOURD-TANGENT: OD=%ld GW=%d side=%s"`（side="L" 或 "R"；OD/GW 原样保留）。

### A4. "出切点"事件（新块，放 TANGENT 块之后、IG 状态块之前，100Hz 拍路径上）
```c
if (s_tan_pending)
{
    if (r.active < (uint8_t)CROSS_ACTIVE_MIN)      /* 宽图案已结束 */
    {
        if (s_gap_cnt < 250u) s_gap_cnt++;
        if (s_gap_cnt >= (uint8_t)GOURD_TANOUT_CONFIRM)
        {
            s_tan_pending = 0u;  s_gap_cnt = 0u;  s_tan_timeout = 0u;
            if (gourd_waves < 200u) gourd_waves++;         /* 偶数++（出切点） */
            s_tan_index++;
            /* 方向：序号奇(1,3)→左转；偶(2)→右转。
               ★负号陷阱：三处消费点都是 e = -gourd_dir * GOURD_FLIP_ERR，
               所以 左转(e<0) ⇒ gourd_dir=+1，右转(e>0) ⇒ gourd_dir=-1 */
            gourd_dir  = (s_tan_index & 1u) ? 1 : -1;
            gourd_flip = (uint8_t)GOURD_FLIP_CYCLES;       /* 转向窗口从出切点起算 */
            /* GW 到 6 的归零+wrap_evt+ge_done 沿用现有逻辑（gourd_waves >= GOURD_TOTAL 处），
               若现有归零代码在 TANGENT 块内，把这段搬出来放两处共用的位置或复制到本块 */
            telemetry 打印一行："GOURD-TANOUT: idx=%d dir=%s OD=%ld GW=%d"
              （dir="L"/"R"；OD=odom_distance_mm()）
        }
    }
    else
    {
        s_gap_cnt = 0u;
        if (s_tan_timeout < 250u) s_tan_timeout++;
        if (s_tan_timeout >= (uint8_t)GOURD_TANOUT_TIMEOUT) /* 兜底：宽迟迟不结束 */
        {
            s_tan_pending = 0u;  s_tan_timeout = 0u;        /* 放弃，不++ */
            telemetry 打印一行："GOURD-TANOUT: timeout (OD=%ld GW=%d)"
        }
    }
}
```
- 归零处理注意：现有 `if (gourd_waves >= GOURD_TOTAL) { gourd_waves=0; gourd_wrap_evt=1; }` 在 TANGENT 块里——TOTAL=6 后**第 6 次 ++ 发生在本块**，把归零+wrap_evt+`ge_done=1` 逻辑放在本块 gourd_waves++ 之后（TANGENT 块里的原归零代码删除或改为共用函数，确保只有一处执行）。
- `ge_done` 置位（CARD-003 变量）在归零处同步置 1。

### A5. ge_done 减速（速度自适应段，sp 定型处之后）
```c
#if GOURD_SLOW_AFTER_DONE
if (ge_done && !ge_exit_done)
    sp = (int16_t)((sp * GOURD_SLOW_PCT) / 100);
#endif
```
（`GOURD_SLOW_PCT=65` 用现值；`GOURD_SLOW_AFTER_DONE` 是新宏，值 1。）

---

## Part B · 出口判据与出状态机（沿用早前版设计，前置自动对齐 GW==6）

### B1. app_config.h 新增
```c
#define GOURD_EXIT_USE_NOFLIP 1   /* 出口判据：宽贴一端+候选期满未翻号+未全灭+ge_done */
```

### B2. 新变量（照 gourd_wrap_evt 模式）
```c
static uint8_t s_cand_lost          = 0;  /* 本次候选期内出现过全灭 */
static uint8_t gourd_exit_noflip_evt = 0;  /* 出口事件位 */
static uint8_t ge_exit_done          = 0;  /* 出口转向已完成 */
```

### B3. 相切点候选块改动
1. 候选起处：`s_cand_lost = 0u;`
2. 候选期内全灭处（现有 `cand_win=0; cand_ok=0;`）：加 `s_cand_lost = 1u;`
3. 期满判定处（与 TANGENT 确认互斥）：期满时若 `!cand_ok && !s_cand_lost && ge_done && !ge_exit_done` → `gourd_exit_noflip_evt = 1u;`
4. **不许动 TANGENT 原判据一个字。**

### B4. 出圈触发块消费 + 完成置位
```c
#if GOURD_EXIT_USE_NOFLIP
if ((trig_now == 0u) && gourd_exit_noflip_evt) { trig_now = 5u; gourd_exit_noflip_evt = 0u; }
#endif
```
- 事件行来源串：`trig==5 ? "by NOFLIP-WIDE (exit, no flip)"`；转向动作/85°闭环/settle 复用现有代码一字不改。
- 现有 `GOURD-EXIT done:` 打印处：`ge_exit_done = 1u;`

### B5. Path① 改挂"出口转完"（CARD-003 A3 段）
`if (ge_done)` → `if (ge_done && ge_exit_done)`；清 IG 时同步清 `ge_exit_done`、`s_tan_index`、`s_tan_pending`；Path③ 兜底清与 `line_follow_init()` 复位同步清全部新变量。

---

## Part C · 验收

### C1. 回归跑（TEST 2：直道 + 弯道各一）
- ✅ 循迹肉眼无变化；IG 全程 0；**零** `GOURD-TANGENT/TANOUT/EXIT` 行；GZ 无削顶。

### C2. 葫芦全程（练习发车点 `R`，`_tmp/card002_run.ps1` 长录）
目击与日志双线验收：
- ✅ 目击受控过圆：每切点"贴行→出→转向→换圆"一气呵成，**同一圆不绕第二圈**；转向方向目击 = 1左 / 2右 / 3左；
- ✅ 日志：3 对 `GOURD-TANGENT`+`GOURD-TANOUT` 配对（idx=1/2/3），GW 1→2→…→6→归零，`side` 字段记下（军师验证"1左3右5左"假设）；
- ✅ GW==6 后明显减速（短行 RPM 可见）；
- ✅ 出口：恰好 1 行 `GOURD-EXIT ... by NOFLIP-WIDE ... ODE=xxx`（ODE 落最后圆切点前 60~200mm 量级）+ 1 行 `GOURD-EXIT done`；
- ✅ 出口后直角弯照常过、弯后出现 `mark cleared`（Path①'，OD 在直角弯之后）、此后无 mark set。

### C3. 失败分支（不改参数，原样回报）
- 切点后转错方向（咬旧圆/甩出丢线）→ 回报 TANOUT 行 dir + 丢线前 10 拍 IR+GZ；
- TANGENT 无 TANOUT 配对 / TANOUT timeout 行出现 → 回报悬空段短行；
- 出口不转 / 转错 / 提前转 → 回报出口段 IR+GZ+GW/IG/ge_done 状态行。

---

## 注意
- 动手前 main 干净先 commit；FW_TAG 真实时间；编译 0E0W。
- 【别动】：出圈触发其余开关（全 0）、`GOURD_WAVE_MIN_MM(200)`、`GOURD_NO_CROSS(1)`、`WIDE_ONE_GO_STRAIGHT(0)`、别动清单全套。
- 方向映射若实测整体反了（车全在切点后反向咬圆）：只翻 A4 里 `gourd_dir` 一处符号，其余不动——回报里说明即可，等军师下卡。
- 字符串字面量不写中文；烧录/串口照分工文档第七节。
