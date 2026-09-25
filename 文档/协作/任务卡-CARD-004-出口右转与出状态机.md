# 任务卡-CARD-004-出口右转与出状态机（问题二）
- 日期：2026-09-25　写卡人：军师
- 背景：CARD-003 后车已能顺序过完 4 圆（历史首次），卡在"过完葫芦后出不去"（出口动作全关，设计内）。数据判读（run2/run4）已定：出口在最后圆上、切点前 ~60mm；出口与切点的图案区分 = 切点必翻号+必全灭，出口两者皆无。
- 目标：车过完葫芦后减速，在出口处右转 85° 出圈，上直道过直角弯，弯后直行 100 拍退出"葫芦状态"（IG/ge_done 全清，回普通循迹）。

---

## 数据依据（工人不用查，为什么这么定）

- run2（CARD-002，100Hz 回放）最后圆一段：**出口** OD 19896-19917，IR 序列 `00111001→00011011→00011111→00011111→00011110`，GZ 全程 +172~+250 **无翻号**、**无全灭**，之后直接回普通线；**切点** OD 19955-19964，IR 分裂两组成对（`00110011` 等），其后 0924 直角弯实测必全灭。
- run4（CARD-003）绕圈过切点序列（67.2~67.8s）：分裂 → 宽贴右（TANGENT）→ **GZ 翻号** → 宽贴左 → 左 2 路（=旧出口图案的误报源）→ **全灭** → 扫回。
- ⇒ 出口可分特征：**宽贴一端 + 候选期满未翻号 + 期内未全灭**；再叠 `ge_done`（GW 归零过 = 已过完 3 切点在最后圆）挡住圈外与首过误报。

## Part A · 出口判据（line_follow.c 相切点块，复用候选机制）

### A1. app_config.h 葫芦块新增（值不许改）
```c
#define GOURD_EXIT_USE_NOFLIP   1   /* 出口判据：宽贴一端+候选期满未翻号+未全灭+ge_done */
#define GOURD_SLOW_AFTER_DONE    1   /* ge_done 后减速（备出口转向） */
```
【不动】：`GOURD_SLOW_PCT(65)`、`GOURD_EXIT_TURN_PWM(41)`、`GOURD_EXIT_TURN_DEG(85.0f)`、`GOURD_EXIT_SETTLE_FRAMES(30)`、`GOURD_EXIT_TURN_DIST(750)`、`GOURD_EXIT_TURN_FRAMES(60)`、`GOURD_EXIT_COOLDOWN(40)`、`GOURD_EXIT_USE_YAW(1)`、其余出圈触发开关保持 0（PATTERN/ON_LOST/ODOM/GW）。

### A2. 新静态变量（照 `gourd_wrap_evt` 的模式声明/复位/消费同进同出）
```c
static uint8_t s_cand_lost = 0;        /* 本次候选期内出现过全灭 */
static uint8_t gourd_exit_noflip_evt = 0;  /* 出口事件位（NOFLIP 判据） */
static uint8_t ge_exit_done = 0;       /* 出口转向已完成（Path①' 的前置） */
```

### A3. 相切点候选块改动（≈356-437 行那一段）
1. 候选起（`else if (wide_one && !gourd_latch)` 处）：同时 `s_cand_lost = 0u;`
2. 候选期内全灭处（现有 `if (r.active == 0u) { cand_win = 0u; cand_ok = 0u; }`）：加 `s_cand_lost = 1u;`
3. **TANGENT 确认条件的紧后面**加"出口候选干净退出"分支（与 TANGENT 互斥——`cand_ok` 相反）：
```c
else if ((cand_win == 0u) && !cand_ok && !s_cand_lost
         && ge_done && !ge_exit_done)
{
    gourd_exit_noflip_evt = 1u;   /* 出圈触发块消费 */
}
```
注意 C 里没有这种 else-if 挂法就直接在 `if ((cand_win == 0u) && cand_ok && !gourd_latch)` 的完整 else 里写（结构：期满时若 `cand_ok` 走 TANGENT，否则若 `!s_cand_lost && ge_done && !ge_exit_done` 置出口事件位）。**不许动 TANGENT 原判据一个字**。

### A4. 出圈触发块消费（≈1063-1070 行，`GOURD_EXIT_USE_GW` 旁）
```c
#if GOURD_EXIT_USE_NOFLIP
if ((trig_now == 0u) && gourd_exit_noflip_evt)
{
    trig_now = 5u;                    /* 第 5 种来源：NOFLIP-WIDE */
    gourd_exit_noflip_evt = 0u;
}
#endif
```
事件行（现有 `GOURD-EXIT: trig=%u ...` 打印处）加来源串：`trig==5 ? "by NOFLIP-WIDE (exit pattern, no flip)"`。转向动作/角度闭环/settle 全部复用现有代码，一字不改。

### A5. 出口转向完成 → `ge_exit_done = 1`
现有出圈转向块"done"处理处（打印 `GOURD-EXIT done: deg=...` 那一段，≈1146 行）加 `ge_exit_done = 1u;`

### A6. ge_done 后减速（速度自适应段，sp 定型处 ≈569-584 行之后）
```c
#if GOURD_SLOW_AFTER_DONE
if (ge_done && !ge_exit_done)
    sp = (int16_t)((sp * GOURD_SLOW_PCT) / 100);
#endif
```

### A7. Path① 改挂"出口转完"（CARD-003 的 A3 段）
现行：`if (ge_done) { ge_straight 累计; >=100 清 IG }` → 改成：
```c
if (ge_done && ge_exit_done) { /* 同原逻辑 */ }
```
即：出口没转完，ge_straight 不累计（实测"GW 归零后圈内有连续 100 拍普通线"会提前清）。清 IG 时同时清 `ge_exit_done`；Path③ 兜底清与 `line_follow_init()` 复位处同步清三个新变量。**直角弯不用识别**：弯道图案非普通线 → ge_straight 天然清零，100 拍直行只可能在弯后凑齐（用户定义）。

---

## Part B · 验收

### B1. 回归跑（TEST 2：直道 + 弯道各一）
- ✅ 循迹无变化；IG 全程 0（圈外误点由 Path② 释放）
- ✅ **零** `GOURD-EXIT` 行（出口判据被 ge_done 挡住）；直角弯照常过
- ✅ GZ 峰值无削顶

### B2. 葫芦全程（练习发车点 `R`，`_tmp/card002_run.ps1` 长录）
- 目击：过 4 圆 → **明显减速** → 出口处右转出圈 → 上直道 → 直角弯 → 继续跑 → 人急停
- ✅ `GOURD-EXIT ... by NOFLIP-WIDE ... ODE=xxx` 恰好 1 行，OD 落在最后圆切点前 ~60-200mm 量级
- ✅ `GOURD-EXIT done` 1 行（deg≈85 或 dist/frames 兜底）
- ✅ 出口转完后：直角弯照常过、弯后出现 `mark cleared`（Path①'，OD 应在直角弯之后）；此后 `mark set` 不再出现
- ✅ GW 首过 3 TANGENT 归零（对照 CARD-003）

### B3. 失败分支（不许改参数，原样回报）
- 出口处不转（继续绕圈）→ 回报：有无 EXIT 候选相关行、出口段（切点前 60mm）的 IR/GZ 短行序列
- 转错方向/转进圆里/转完丢线不回 → 回报丢线前最后 10 拍 IR+GZ（急停回放里有）
- 提前触发（在圈外/直角弯转）→ 回报当时 ge_done/GW/IG 状态行

---

## 注意
- 动手前 main 干净先 commit；FW_TAG 真实时间；编译 0E0W。
- 【别动】：出圈触发其余开关（全 0）、`GOURD_TOTAL(3)`、`GOURD_WAVE_MIN_MM(200)`、`GOURD_NO_CROSS(1)`、`WIDE_ONE_GO_STRAIGHT(0)`、别动清单全套。
- 事件位/新变量的 `#if` 范围照 `gourd_wrap_evt` 抄（声明/复位/使用同进同出）。
- 字符串字面量不写中文；烧录/串口照分工文档第七节。
