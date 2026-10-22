# FW:1022-1805（分支 `exp/igc-1023`）= **IG 判据改成"亮 ≥5 且连续 2 拍"** + 两个 LED 版本

## 1) IG 判据（他："改成 5 个红外传感器被触发才 IG+1"）

| 宏 | 值 | 含义 |
|---|---|---|
| `IGC_LIT_MIN` | **5** | 亮路数 **≥5** 才算一次（"大于 4 个" ≡ "≥5"，数值本来就等于 5）|
| `IGC_CONFIRM_FR` | **2（新）** | ★"**才**"的严格性：必须**连续 2 拍**都 ≥5 才算数 —— 单拍毛刺不再计数 |
| `IGC_LIT_CLEAR` | 3 | 亮 ≤3 才算退出一片白（迟滞）|
| `IGC_MIN_MM` | 150 | 两次计数最少隔 150 里程 |

想更严 ⇒ `IGC_LIT_MIN` 改 6；想更松 ⇒ `IGC_CONFIRM_FR` 改 1。

## 2) ★LED 亮灯源全清单（他要求"查一下，我排除"）

### 跑车中（`CAR_RUN`）会亮灯的地方

| # | 位置 | 条件 | 现象 |
|---|---|---|---|
| ① | `fsm/car_fsm.c:100` | 进入运行态 | **LED3 常亮**（基线，躲不掉）|
| ② | `fsm/car_fsm.c:292` | `line_follow_is_lost()` | **LED2 100ms 闪**（丢线）|
| ③ | `fsm/car_fsm.c:315`（出口 LED）| 出口右转触发 | **三灯同闪 600ms**（`GSP_EXIT_LED 1` / `GSP_EXIT_LED_MS 600`）|
| ④ | `fsm/car_fsm.c:346`（三灯同闪）| 切点窗口 ∩ `GSP_BLINK_MASK`（**0x00 已关**）或 槽③ 速度窗口 ∩ `SPD_WIN_BLINK_MASK`（**0x04，IGC4 里是开的**）| **三灯 10Hz 连续闪**（槽③ = 最后直角弯 17200~17655 那 455mm）|
| ⑤ | `fsm/car_fsm.c:408`（单灯窗口指示）| `line_follow_gsp_dir_mask() & GSP_LED_MASK(0x07)` | **LED1/LED2/LED3 单灯常亮**：哪个切点窗口开着就点哪盏 —— **这个最容易和 IG 混** |
| ⑥ | `control/line_follow.c:2005`（直道提速）| `boost_active` 上膛（`USE_STRAIGHT_BOOST 1`）| **三灯常亮**！注意：`STRAIGHT_BOOST_SPEED 57` = 静态档 ⇒ **提速没效果、但灯照亮**（很可能是你看到的"进去之后常亮"）|
| ⑦ | 新增 `IGC` | IG+1 | **三灯闪 50ms** |

### 其它状态 / 手动

| # | 位置 | 条件 | 现象 |
|---|---|---|---|
| ⑧ | `fsm/car_fsm.c:256` | `CAR_COUNTDOWN` 读秒 | LED1→2→3 **逐个点亮** |
| ⑨ | `fsm/car_fsm.c:424` | `CAR_STOPPED` 到站 | 三灯 **2Hz 闪** |
| ⑩ | `fsm/car_fsm.c:433` | `CAR_EMERGENCY` 急停 | 三灯 **快闪** |
| ⑪ | `telemetry.c:809` | 串口 `L <0~7>` | 手动点灯 |

## 3) 两个 hex，按需要选

| | 内容 | hex | built |
|---|---|---|---|
| **IGC4** | 只改 IG 判据（≥5 且连续 2 拍）。**灯仍然热闹**（④⑤⑥ 都在）| `firmware/1022-1805-IGC4/test16.hex`（`0beab65c…2d97d`）| **18:05:38** |
| **IGC5** | IGC4 + **把 ④⑤⑥ 全关**：`SPD_WIN_BLINK_MASK 0x04→0`、`GSP_LED_MASK 0x07→0`、`USE_STRAIGHT_BOOST 1→0` | `firmware/1022-1805-IGC5/test16.hex`（`4b405c8b…d886e02`）| **18:05:56** |

**IGC5 跑车时只剩这几种灯**：

| 看到 | 含义 |
|---|---|
| LED3 常亮 | 正常跑（基线）|
| LED2 闪 | 丢线 |
| **三灯闪一下（50ms）** | **IG +1** ← 你要看的 |
| 三灯连续闪 600ms | 出口触发 |
| （单灯窗口 / 槽③ 闪 / 提速三灯）| **全部已关** |

⚠️ `USE_STRAIGHT_BOOST` 改成 0 是**行为无害**的（原来目标速 57 = 静态档，提速本来就没生效），只删掉了那三盏常亮灯。
以后想恢复任何一条：`GSP_LED_MASK 0x07` / `USE_STRAIGHT_BOOST 1` / `SPD_WIN_BLINK_MASK 0x04`，各自独立。

## 版本信息

- 编译 **0E0W**，`Code=32588`（IGC5 与 IGC4 同尺寸）；横幅 `1021-R183` ⇒ 认版看 `built`。
- tag：`fw-1022-1805-igc4` / `fw-1022-1805-igc5`。
