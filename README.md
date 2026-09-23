# SMARTCAR · 智能循迹小车

西南石油大学校赛智能车（2026 年 10 月）—— 循迹 + 差速竞速车。
**STM32F103C8T6 + HAL 库 + Keil MDK 工程**；赛道为黑底 25mm 白线，含直角弯 / 十字 / Ω 回头弯 / 相切圆环。

---

## 硬件

| 部分 | 说明 |
|---|---|
| 主控 | STM32F103C8T6 核心板（插在自制 PCB 上） |
| 驱动 | TB6612 ×1 → 2× JGA25-370 减速电机（620rpm，差速转向） |
| 循迹 | 8 路灰度/红外模块（数字量，装在车头前伸的**可调扩展板**上，前瞻 124mm） |
| 测速 | 编码器正交解码（左 TIM3 = PA6/PA7，右 TIM4 = PB6/PB7） |
| 姿态 | MPU6050（I2C2 = PB10/PB11） |
| 底盘 | 三点式：后两轮驱动 + 前球脚轮；3D 打印底板（149.84 × 122.69 × 2.5mm） |
| 人机 | LED×3（PC13/PB14/PB15，**高电平点亮**）+ 启动键（PB9）+ 蜂鸣器（PA11，当前硬件故障） |

## 目录

```
Core/App/             ← 我们自己写的分层框架（唯一需要改的地方）
  app_config.h        ★★ 调参入口：参数 / 功能开关 / 引脚表 / 版本号
  app.c/.h            顶层调度（10ms 控制节拍 + 10ms 人机节拍 + 遥测节拍）
  telemetry.c/.h      串口遥测 + 在线调参（寄存器级中断，防 HAL 卡死）
  drivers/            motor / line_sensor / speed / encoder / led / buzzer / key / imu / odom(里程计)
  control/            pid / filter / line_follow（PD + 速度自适应 + 丢线 + 十字 + 葫芦出圈）
  fsm/car_fsm.c       状态机 IDLE→读秒→RUN→STOPPED/EMERGENCY
Core/Src  Core/Inc    CubeMX 生成的（main/gpio/tim/usart/it/msp/freertos）+ 手写 i2c.c（I2C2）
                      + FreeRTOSConfig.h（RTOS 配置）
Drivers/              ST 官方 HAL + CMSIS
Middlewares/          FreeRTOS + CMSIS-RTOS2（2026-09-23 队友引入）
MDK-ARM/              Keil 工程 test16.uvprojx
文档/                  设计文档 / 比赛规则 / 调研记录 / 代码交接说明 / **实车复盘**
机械结构/               SolidWorks 模型 + STL（底板、扩展板）
硬件设计/               PCB 工程包 + BOM
脚本/                  tune.ps1（串口调参辅助）
```

## 编译 & 烧录

1. Keil uVision5 打开 `MDK-ARM/test16.uvprojx` → `F7` 编译 → `F8` 下载
2. 命令行编译：`UV4.exe -b "MDK-ARM\test16.uvprojx" -o "MDK-ARM\build.log"`
3. 或用 OpenOCD（ST-Link，SWD：PA13/PA14）：
   ```
   openocd -f interface/stlink.cfg -c "transport select swd" -f target/stm32f1x.cfg \
           -c "adapter speed 500" -c "program MDK-ARM/test16/test16.hex verify reset exit"
   ```

## 串口命令（115200，每条必须以 `\r\n` 结尾）

| 命令 | 作用 |
|---|---|
| `P 3.0` / `D 0.5` | 设 KP / KD |
| `S 70` | 设**直道**速度（弯道自动 ×0.7） |
| `T 1 40` / `T 2 40` | 单电机测试（正数 = 前进）；`T 0 0` 停 |
| `M 40 40` | 直接给左右轮 PWM（测试模式） |
| `L 0~7` | 直接点灯（bit0=LED1） |
| `B [ms]` | 蜂鸣器响 |
| `R` | 串口启动（= 按启动键：3 秒读秒 → 跑） |
| `V` | 打印版本横幅 |
| `H` | 诊断：8 路位图 + 按键 + 编码器 + IMU（`WHO=` 应为 68） |
| `O` | 读里程：`ODOM NOW: OD=xxxxmm PATH=yyyymm L=.. R=..`（**手推标定用**） |
| `Z` | 里程清零：`OK ODOM RESET`（不用断电就能反复测） |
| `X` | **急停**（连发两个 `XX` 才生效，防噪声误触发） |

**遥测行**（待机 500ms / 运行中 50ms 一行；字段很多，常用的加粗）：
```
FW:0923-5400 IR:10110000 RPM1=210 RPM2=205 KP=3.0 SP=52 ST=2 DT=10 OD=1234 PATH=1250
ODE=0 OS=0 GZ=0 GW=0 SB=0 G7=3 GX=0 GR=0 CX=4 BR=1 GE=0 GD=0 IG=0 EH=1
```
| 字段 | 含义 |
|---|---|
| **`ST=`** | 0 待机 / 1 读秒 / 2 运行 / 3 到站 / 4 急停 |
| **`IR=`** | 8 路位图，**第 1 个字符 = 最左那一路**（权重 -7…+7） |
| `RPM1/2` | 左右轮实测转速（编码器） |
| **`DT=`** | **实测控制周期**（应稳定在 10；明显 >10 = 主循环被拖慢） |
| **`OD=` / `PATH=`** | 里程：净位移 / 走过总路程（mm） |
| **`ODE=`** | 从"进葫芦圈那一刻"起算的位移（mm）—— 出圈里程判据就看它 |
| `OS=` | 本窗口内"同一拍多次子采样位图不一致"的拍数（>0 = 过采样在起作用） |
| `GZ=` | MPU6050 偏航率（°/s） |
| `GW=` | 葫芦相切点计数（`USE_GOURD_SM`，现在主要当诊断用） |
| `SB=` | 直线提速生效中 |
| `G7/GX/GR` | 最右一路触发计数 / 已触发过出圈标志 / 硬转剩余拍数（`USE_G7_TURN=0` 时后两个恒 0） |
| **`CX=` / `BR=`** | 判成十字的次数 / "只贴一端→拒绝当十字"的次数 |
| `GE/GD/IG/EH` | 出圈状态机：进圈印记 / 印记倒计时 / 是否在圈内 / 边缘保持生效 |

## 调参入口（只在 `Core/App/app_config.h`）

> ⚠️ 这个文件现在有 780+ 行、分节带注释，**改之前先读该节顶部的说明**——里面写了每个数的来历
> （哪个是实测的、哪个是推算的、改电压要重算哪些）。以下只列分类，不重复数值。

- 开关：`USE_ENCODER` / `USE_IMU` / `USE_SPEED_LOOP` / `USE_ADAPTIVE_SPEED` / `USE_D_FILTER` / `USE_STRAIGHT_BOOST` / `AUTO_START_ENABLE`
- 十字：`CROSS_ACTIVE_MIN` / `CROSS_CONFIRM_CNT` / `CROSS_MAX_FRAMES` / `CROSS_NEED_BOTH_ENDS` / `USE_CROSS_BUDGET` + `CROSS_BUDGET_MAX` / `CROSS_REARM_FRAMES`
- **葫芦出圈**：`USE_GOURD_EXIT`（总开关）/ `USE_GOURD_EXIT_ODOM` + `GOURD_EXIT_ODOM_MM` + `GOURD_EXIT_ODOM_FROM_START`（里程判据）/ `GOURD_EXIT_USE_YAW`（偏航角闭环）/ `GOURD_EXIT_TURN_FRAMES`（拍数保险）/ `USE_GOURD_IG_GYRO`（进圈判据）
- 旧的"最右一路计数硬转"：`USE_G7_COUNT`（计数，诊断用，**保持开**）/ `USE_G7_TURN`（**当前 0 = 关**，见下方复盘）
- 红外过采样：`LINE_OS_ENABLE` / `LINE_OS_SAMPLES` / `LINE_OS_SPACING_US` / `LINE_OS_VOTE_MAJORITY`
- 边缘保持：`USE_EDGE_HOLD`；提速：`STRAIGHT_BOOST_*`
- 循迹：`KP` / `KD` / `SPEED_STRAIGHT` / `SPEED_CURVE` / `CURVE_ABS_ERR_THRESH`
- 丢线：`LOST_SPIN_DELAY` / `LOST_SPIN_SPEED` / `LOST_MAX_CYCLES`
- 里程计：`ODOM_UM_PER_COUNT`（um/计数）、`ODOM_SIGN_L/R`
- 速度闭环：`SPD_PID_KP/KI/KD/IMAX`、`RPM_PER_PWM_X100`（**改电池电压要按比例改它**）
- 极性：`LINE_ACTIVE_LEVEL`（本车 8 路模块 = **白线输出高** → `GPIO_PIN_SET`）
- 电机方向：`MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT`
- 版本：`FW_TAG`（改完代码更新，**取真实时间** → 串口每行都带，一眼确认烧没烧）

## ⚠️ 踩过的坑（改代码前先看）

1. **CubeMX 重新生成后必查三处**：`usart.c` 的 USER CODE 区要保留 WordLength/StopBits/Parity/**Mode** 四行（6.17 生成时会吞掉 → 串口收发全废）；`main.c` 的 `#include "app.h"` 必须在 USER CODE 区；`.ioc` 里编码器枚举要写 `TIM_ENCODERMODE_TI12`
2. **PB10/PB11 上是 I2C2，不是 I2C1** ← 写错过。I2C 驱动是**手写**的（`Core/Src/i2c.c`），不走 CubeMX；换新工程要手动放开 `stm32f1xx_hal_conf.h` 的 `HAL_I2C_MODULE_ENABLED`
3. **`motor.c` 必须 include `app_config.h`**（否则 `#if MOTOR_LEFT_INVERT` 被当 0，方向反转静默失效；已加 `#error` 守卫）
4. **8 路模块若供 5V，DO 可能是 5V 电平** → PA0/PA1/PB0/PB1 **不耐 5V**，接线前先量
5. **烧录和串口别同时插两条 USB**（两个地回路打架 → 串口收不到）。烧录用 ST-Link、调参用串口，一次一条
6. **电池没电 = 电机嗡嗡转不动**（不是代码问题，先量 VM 电压）
7. **别往字符串字面量里写中文** → 会触发 ARMCC `#870-D`（源文件是 UTF-8，ARMCC 按 GBK 双字节规则扫）。**字面量一律用英文，注释里中文没事**
8. **CubeMX 重新生成有把工程搞坏的历史**（9/22 出过一次事故：多文件被冲）→ 生成前先 commit，生成后逐项核对，见 `文档/事故-CubeMX重新生成破坏工程-20260922.md`
9. **`speed_update()` 全工程只被 `telemetry_report()` 调用** → 以后做"静默/关遥测"的改动，**判断必须放在 `speed_update()` 之后**，否则测速冻结，"车是否真的在跑"的判据会失效
10. **`LED1 = PC13` 只在急停/停车时闪**，而那两处无条件调了 `print_odom()` → 所以 **"PC13 在闪" 等价于 "`print_odom` 执行过"**，是个现成的诊断等式

## 当前状态（2026-09-23 晚）

- 固件 **FW:0923-5400** —— **新基线 = 队友实车验证过的那条线**
  （`main` 在 `96b2940` 以它为准合并；队友自己的完整版仍在分支 `team/0923-5400` / 标签 `fw-0923-5400`）
- **电池已换 12V 锂电池**（3S：标称 11.1V / 满电 12.6V）。队友**故意取"适度提速 +20%"**（PWM ×0.74）：
  `BASE_SPEED 44` / 直道 52 / 弯道 36 / 提速 70，**`KP` 仍 3.0**（注释写"先跑一次看弯道摆不摆头，摆了再降"）。
  ⚠️ 改电压要重算的清单写在 `app_config.h` §二 顶部；注意 `RPM_PER_PWM_X100` 要 **×1.622**
- **已实现**：串口在线调参 · 编码器测速 · **里程计**（`OD`/`PATH`/`ODE`，含 `O`/`Z` 标定命令）·
  速度自适应 · 丢线兜底 · 十字形状判据（两端都贴）· **葫芦出圈 = 里程 + 偏航角闭环** ·
  **红外过采样 8 次 + 多数表决** · 边缘保持 · 直线提速 · MPU6050 偏航率 · FreeRTOS
- **★实车已验证的三条结论（别推翻）**：
  1. **图案判据驱动动作 = 死路** —— `11000000` 在左直角弯出口同样出现，`REQUIRE_ENTRY` 闸门也挡不住
  2. **开环盲走有害** —— 22 拍日志证明它会把"本来在收敛的 PD"强行关掉
  3. **出圈只能靠里程** —— 详见 `文档/真相-3800为什么过不了葫芦圈-20260923.md` 与
     `文档/失败复盘-出圈判据打坏直角弯-20260923.md`
- **待办**：终点停车（到 G 停住，规则里不到 G 停 -5 分）· 速度闭环（已实现但**暂停中**，需先把 PID 整定好）·
  蜂鸣器硬件 · 串口 RX 待实测 · `USE_CROSS_BUDGET` 的 `4` 待实测标定
- 更多细节见 `文档/给接手AI的复查提醒-20260923.md`、`文档/文档索引与状态.md`、`文档/对话记忆与交接-20260910.md`
