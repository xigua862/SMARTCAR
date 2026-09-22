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
  drivers/            motor / line_sensor / speed / encoder / led / buzzer / key / imu
  control/            pid / filter / line_follow（PD + 速度自适应 + 丢线 + 十字 + 葫芦）
  fsm/car_fsm.c       状态机 IDLE→读秒→RUN→STOPPED/EMERGENCY
Core/Src  Core/Inc    CubeMX 生成的（main/gpio/tim/usart/it/msp）+ 手写 i2c.c（I2C2）
Drivers/              ST 官方 HAL + CMSIS
MDK-ARM/              Keil 工程 test16.uvprojx
文档/                  设计文档 / 比赛规则 / 调研记录 / 代码交接说明
机械结构/               SolidWorks 模型 + STL（底板、扩展板）
硬件设计/               PCB 工程包 + BOM
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
| `X` | **急停** |

**遥测行**（待机 500ms / 运行中 50ms 一行）：
```
FW:0922-0940 IR:10110000 RPM1=210 RPM2=205 KP=3.0 SP=70 ST=2 GZ=0 GW=0 SB=0
ST: 0待机 1读秒 2运行 3到站 4急停 | GZ=偏航率°/s | GW=葫芦相切点计数 | SB=直线提速中
```

## 调参入口（只在 `Core/App/app_config.h`）

- 开关：`USE_ENCODER` / `USE_IMU` / `USE_SPEED_LOOP` / `USE_ADAPTIVE_SPEED` / `USE_D_FILTER` / `USE_STRAIGHT_BOOST` / `USE_GOURD_SM` / `AUTO_START_ENABLE`
- 循迹：`KP` / `KD` / `SPEED_STRAIGHT` / `SPEED_CURVE` / `CURVE_ABS_ERR_THRESH`
- 丢线：`LOST_SPIN_DELAY` / `LOST_SPIN_SPEED` / `LOST_MAX_CYCLES`
- 十字：`CROSS_ACTIVE_MIN` / `CROSS_CONFIRM_CNT` / `CROSS_MAX_FRAMES`（宽图案持续时间上限）
- 直线提速：`STRAIGHT_BOOST_DELAY` / `STRAIGHT_BOOST_SPEED`
- 葫芦弯：`GOURD_EDGE_WINDOW` / `GOURD_TOTAL` / `GOURD_FLIP_CYCLES`
- 极性：`LINE_ACTIVE_LEVEL`（本车 8 路模块 = **白线输出高** → `GPIO_PIN_SET`）
- 电机方向：`MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT`
- 版本：`FW_TAG`（改完代码更新 → 串口每行都带，一眼确认烧没烧）

## ⚠️ 踩过的坑（改代码前先看）

1. **CubeMX 重新生成后必查三处**：`usart.c` 的 USER CODE 区要保留 WordLength/StopBits/Parity/**Mode** 四行（6.17 生成时会吞掉 → 串口收发全废）；`main.c` 的 `#include "app.h"` 必须在 USER CODE 区；`.ioc` 里编码器枚举要写 `TIM_ENCODERMODE_TI12`
2. **PB10/PB11 上是 I2C2，不是 I2C1** ← 写错过。I2C 驱动是**手写**的（`Core/Src/i2c.c`），不走 CubeMX；换新工程要手动放开 `stm32f1xx_hal_conf.h` 的 `HAL_I2C_MODULE_ENABLED`
3. **`motor.c` 必须 include `app_config.h`**（否则 `#if MOTOR_LEFT_INVERT` 被当 0，方向反转静默失效；已加 `#error` 守卫）
4. **8 路模块若供 5V，DO 可能是 5V 电平** → PA0/PA1/PB0/PB1 **不耐 5V**，接线前先量
5. **烧录和串口别同时插两条 USB**（两个地回路打架 → 串口收不到）。烧录用 ST-Link、调参用串口，一次一条
6. **电池没电 = 电机嗡嗡转不动**（不是代码问题，先量 VM 电压）

## 当前状态（2026-09-22）

- 固件 **FW:0922-0940**：新车 v2（前瞻 124mm）已装车跑通；KP=3.0、直道 70 / 弯道 49
- 已实现：串口在线调参 / 编码器测速（RPM ≈ 3.59 × PWM，满速 ≈ 1.23 m/s）/ 速度自适应 / 丢线兜底 / 十字强制直行 / **直线提速到 99** / **葫芦弯相切点状态机** / MPU6050 偏航率
- 待办：终点停车（图案 + 里程双条件）、速度闭环、蜂鸣器硬件修复、串口 RX 修复
- 更多细节见 `文档/代码交接说明-20260921.md` 与 `文档/文档索引与状态.md`
