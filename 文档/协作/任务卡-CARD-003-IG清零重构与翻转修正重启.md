# 任务卡-CARD-003-IG清零重构与翻转修正重启（平稳过葫芦）
- 日期：2026-09-25　写卡人：军师
- 背景：进圈判据已实测可靠（每次进葫芦 IG 准时跳 1，用户亲测），但旧清零条件（"连续 20 拍普通线 或 TTL 到期"）太灵——圈内绕圈时 |GZ| 瞬跌 + 普通线图案会把 IG 打到 0（用户单圆绕圈亲见）。IG 乱跳正是翻转修正门控不可靠的最后一个根因（`GOURD_FLIP_ON_NORMAL` 注释里写明的重试前提）。同时相切点目前【没有任何转向动作】（`WIDE_ONE_GO_STRAIGHT=0`、`GOURD_USE_FLIP=0`）→ 车跨圆全靠运气（run1 目击：每个圆绕两圈才过去）。
- 目标：① IG 在葫芦内稳定为 1，只在"GW 数满 + 出圈后直行"才清；② 相切点恢复翻转修正，车在每个切点主动拐进下一个圆。

---

## Part A · IG 清零逻辑重构（line_follow.c + app_config.h）

### A1. app_config.h 葫芦配置块新增两个宏（值不许改）
```c
#define GOURD_EXIT_STRAIGHT_FRAMES 100   /* GW数满后 连续"普通线"100拍(=1秒) → 确认出圈 */
#define GOURD_IG_FAILSAFE_FRAMES   3000  /* IG=1 持续3000拍(=30秒) 强制清 —— 兜底, 正常过葫芦~10秒用不到 */
```
`GOURD_ENTRY_CLEAR_FRAMES(20)` / `GOURD_ENTRY_MARK_TTL(600)` 保留原值（Path② 继续用）。

### A2. line_follow.c 新增状态变量（与 `ge_in_gourd` 同一个 `#if` 范围声明、同进同出）
```c
static uint8_t  ge_confirmed = 0;  /* 本次进圈后已数到过>=1个相切点(=葫芦坐实) */
static uint8_t  ge_done      = 0;  /* GW 已归零(相切点全部过完) */
static uint16_t ge_straight  = 0;  /* ge_done 后的连续普通线拍数 */
static uint16_t ge_stay      = 0;  /* IG=1 持续拍数(兜底用) */
```
置位/复位点（全部跟着现有同类动作走，别新开分支）：
- `gourd_waves++` 同处（≈419 行）：`ge_confirmed = 1u;`
- GW 归零同处（≈422-423 行，`gourd_wrap_evt = 1u` 旁）：`ge_done = 1u;`
- `s_ig_leave` 清 GW 同处（≈885-889 行）：同时清 `ge_confirmed / ge_done / ge_straight`
- 每一处 IG 清 0（含下面三条新路径与旧 Path②）：同时清 `ge_confirmed / ge_done / ge_straight / ge_stay`
- `line_follow_reset()`：四个全清（与 `ge_in_gourd` 同处）

### A3. IG 块（≈789-857 行）清零判定改成三条路径
伪代码（照此翻成 C；打印沿用现有风格，字符串字面量【不许写中文】）：
```
if (ig_now) {
    /* 现有动作全部保留：TTL 续命、0→1 边沿置 IG、记 ge_odom_entry、
       GOURD_EXIT_ONE_SHOT 解锁、"GOURD-ENTRY mark set" 打印 */
    在 0→1 边沿处: ge_stay = 0;
} else if (!ge_confirmed) {
    /* 【Path②】GW 未坐实：旧快清【原样保留】——
       TTL-- / ge_plain_cnt 普通线计数 / (plain>=20 || ttl==0) 则清 IG，
       含原有解锁动作与 "mark cleared" 打印，一字不改。
       作用：圈外误点 IG(普通弯道/直角弯) 200ms 内释放，不黏住。 */
} 
/* ge_confirmed==1 且 ig_now==0：什么都不做 —— IG 保持 1（这就是修"圈内乱跳0"） */

if (ge_in_gourd) {
    if (ge_stay < 60000u) ge_stay++;
    /* 【Path①】正常出圈：GW 数满后，连续普通线 100 拍 */
    if (ge_done) {
        if (ge_is_plain_line(r.raw)) ge_straight++;  else ge_straight = 0u;
        if (ge_straight >= GOURD_EXIT_STRAIGHT_FRAMES)
            → 清 IG（清零动作与现有清零块一致）+ 打印 "GOURD-ENTRY mark cleared (done+straight)";
    }
    /* 【Path③】兜底：IG=1 持续 3000 拍强清（防 GW 数不满死在圈里） */
    if (ge_stay >= GOURD_IG_FAILSAFE_FRAMES)
        → 清 IG + 打印 "GOURD-IG FAILSAFE clear";
}
```
要点（为什么这么改，回报时不用复述，但别改错）：
- 删掉的就是 `ge_confirmed==1` 时"ig_now==0 + 普通线 20 拍"这条旧路 —— 它是圈内 IG 乱跳 0 的根因。
- 直角弯图案不是普通线（`ge_is_plain_line` 返回 0）→ `ge_straight` 天然清零，"直角弯后直行 100 拍"自动成立，【不需要】识别直角弯（那是问题二）。
- 若出口→直角弯之间的直道超过 1 秒，IG 会在弯前清掉 —— 无害，那时葫芦已跑完。
- 已知风险（接受，不处理）：圈外 IG 误点【且】圈外相切点判据误报（历史上直角弯误报过 1 次）同时发生 → IG 由 Path③ 兜底最长黏 30 秒。验证跑若真出现再开卡治。

### A4. `s_ig_leave` 那段（IG=0 连续 30 拍清 GW）一字不动。

---

## Part B · 翻转修正重启（app_config.h，只改这两个值）
- `GOURD_USE_FLIP`：0 → **1**
- `GOURD_FLIP_ON_NORMAL`：0 → **1**
- 【不动】`GOURD_FLIP_CYCLES(15)`、`GOURD_FLIP_ERR(3)`、`GOURD_TOTAL(3)`、`GOURD_WAVE_MIN_MM(200)`、`WIDE_ONE_GO_STRAIGHT(保持0)`。
- 为什么：车锁同一个圆的病根 = 下一个圆的曲率方向相反、PD 只顺最强线绕原圆；翻转（切点确认后 15 拍反向修正）是对症药。0924 晚试过失败的根因是 IG 误点火（量纲/清零两个病），现已被新基线 + Part A 修掉。直行窗口这次【不开】——变量隔离，翻转不行下张卡再试它。

---

## Part C · 验收（两趟跑）

### C1. 回归跑（圈外，防新清零逻辑伤普通路段）
`TEST 2` 在直道 + 弯道段各来一次（或葫芦前的普通赛道跑一小段）。
- ✅ 直道/弯道循迹肉眼无变化；
- ✅ IG 全程 0；若弯道期间 IG 变 1，回到直道后 30 拍内必须出现 "mark cleared"（Path② 在干活）；
- ✅ 无 `GOURD-TANGENT` 打印。

### C2. 葫芦穿越跑
练习发车点发车（`R`），`_tmp/card002_run.ps1` 长录。
- 车若在某个圆里绕圈出不去 → 人喊 `XX` 急停（自动回放）；
- 车若自己顺着出口线出了葫芦 → 让它跑到直角弯、过了直角弯再停（能顺带验证 Path① 全程）；
- 用户目击记录：在哪个圆卡住 / 每个切点是否干净利落地换圆。

验收线：
- ✅ 目击：每个相切点换到下一个圆，**同一个圆不绕第二圈**，按顺序过圆；
- ✅ 日志：`GOURD-TANGENT` 恰好 3 行、相邻 OD 差 ≥200mm；GW 走 1→2→3(归零)；
- ✅ IG 全程 1，停车前无 "mark cleared"。

### C3. 判定与上报
- 仍锁同一个圆 → 【不许改参数】，回报写清卡在哪个圆 + 附日志，军师下张卡定"加直行窗口"还是"加大翻转力度"；
- 一趟跑出现 6 行 `GOURD-TANGENT`（每切点 2 次）→ 原样回报，下张卡把 `GOURD_TOTAL` 改 6（清零判据按归零、形式不变）；
- 数据别加工：录制文件原样 + 用户目击描述写进回报。

---

## 注意
- 动手前 main 必须干净（先 commit）；`FW_TAG` 用真实时间（`date +%m%d-%H%M`）。
- 编译线：`powershell -File _tmp/build.ps1` → 0 Error 0 Warning。
- 【别动】`USE_GOURD_EXIT_ODOM / GOURD_EXIT_USE_GW / GOURD_EXIT_USE_PATTERN / GOURD_EXIT_ON_LOST`（全保持 0）、`USE_GOURD_SLOWDOWN(0)`、`GOURD_ARC_FLIP(0)`、`GOURD_NO_CROSS(1)`。
- 新增四个静态变量与 `ge_is_plain_line` 的可见性都在 `USE_GOURD_EXIT=1` 下，编译没问题；`#if` 范围声明/复位/使用同进同出。
- 烧录与串口照分工文档第七节（一条 DAPLink USB；烧录和读串口别同时）。
