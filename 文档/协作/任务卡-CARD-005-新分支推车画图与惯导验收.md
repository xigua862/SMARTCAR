# 任务卡-CARD-005-新分支推车画图与惯导验收（回放路线第一卡）
- 日期：2026-09-26　写卡人：军师
- 背景：用户拍板双线并行——主线（GW 切点状态机，CARD-004）队友继续推；新开分支走"地图回放导航"（= 把赛道"背"下来预存轨迹表，正赛回放；马铃薯国赛光电管 v17 思路，决策日志 2026-09-25 已列入二期，当时前置"须先修 IMU"已由 CARD-001/002 完成）。
- **本卡目标：只回答一个问题——我们的 MPU6500+编码器做航位推算，绕赛道一圈（含葫芦）能画成什么样。这是整条新路的命脉，十分钟推车就能验证，不写任何控制代码。**

---

## 方案考古结论（为什么这么裁剪，工人不用读原库）

马铃薯 v17 完整库 400KB+（TC264 双核 200MHz×2、VQF 四元数滤波 80KB、16 路 ADC 光电点云匹配、负压、5m/s）。逐文件看过后**只搬三件 + 一套公式**：

| 马铃薯文件 | 大小 | 搬不搬 | 说明 |
|---|---|---|---|
| `drive_odometry.c/h` | 4.4KB | **搬**（重写） | 左右轮平均位移 + 陀螺航向中点积分位姿，~50 行 |
| `route_projection.c/h` | 3.8KB | **搬**（CARD-006） | 前向窗连续投影（进度 s / 横向误差 ey） |
| `route_reference.c/h` | 2.9KB | **搬**（CARD-006） | 航向/曲率插值（±180° 绕回处理） |
| 控制公式 | 注释公开 | **搬**（CARD-007） | `omega_ff=v·kappa(s+v·τ)`，`turn=-(b/2)·omega_cmd + K_omega·(ω测-ω令)`，Frenet 回正 |
| VQF（vqf.cpp 80KB） | — | 不搬 | 5m/s mm 级定位才需要；我们 1.2m/s 校赛，梯形 yaw 积分够 |
| 光电点云匹配 | 45KB+ | 二期再说 | 我们 8 路数字红外做不了点云；先纯推算，红外观测修正列二期 |
| 双核架构/EMEM/负压 | — | 不搬 | F103 单核 72MHz @100Hz 软浮点余量充足 |

**我们已有的零成本资产**：`imu_get_gyro_z()`（°/s 已减零偏，正=左转）、`imu_get_yaw()/imu_yaw_reset()`（队友固件 100Hz yaw 积分，带实测 dt）、`odom_left_mm()/odom_right_mm()`（左右轮净位移 mm）、遥测/TRACE 全套。

---

## Part A · 建立双线并行环境（git worktree，主目录一根手指都不碰）

### A1. git worktree（主目录工人正在跑 CARD-004，切分支会踩他）
```powershell
& "C:\Program Files\Git\cmd\git.exe" -C "c:\Users\asus\Desktop\test17" worktree add "c:\Users\asus\Desktop\test17-nav" -b nav-replay
```
- 之后所有新分支工作在 `c:\Users\asus\Desktop\test17-nav` 目录做（MDK 工程在那边独立打开编译，输出互不影响）。
- 主目录 `test17` = 主线（CARD-004），谁都不许在两边混着改。
- 物理约束：车和 DAPLink 只有一套 → **烧录/串口使用前后跟用户说一声错峰**。

### A2. 新文件 `Core/App/drivers/pose.c/h`（nav-replay 分支内）
接口：
```c
void    pose_reset(void);                      /* x=y=ψ=0 */
void    pose_step(void);                       /* 每控制拍(10ms)调一次 */
float   pose_get_x_mm(void);
float   pose_get_y_mm(void);
float   pose_get_yaw_deg(void);                /* 顺时针为正(右转增)，与GZ左正相反 */
```
实现规则（钉死符号，写错整张图镜像）：
- 坐标系：发车点为原点，**x 车头方向、y 车身右侧**（右系顺时针，与马铃薯一致）。
- 每拍取增量：`ΔL = odom_left_mm() - 上拍值`、`ΔR = odom_right_mm() - 上拍值`（存 static 上拍值；`odom_reset()` 后上拍值同步清）。
- 航向：`ψ_rad += (-imu_get_gyro_z()) × dt`（GZ 左正 → ψ 右正，所以取负）。
- **中点航向法**（照抄马铃薯 drive_odometry，二阶精度）：
  ```
  ψ_mid = ψ_old + (ψ_new - ψ_old)/2
  d = (ΔL + ΔR)/2
  x += d × cos(ψ_mid);   y += d × sin(ψ_mid)
  ```
- dt 用 `CTRL_PERIOD_MS`（与 imu.c 同源即可）。
- `pose_step()` 挂进主循环/控制拍调用点（和 `odom_update()` 同一处，保证推车时不发车也在跑——run2/3 静止数据证明传感器环常跑）。

### A3. 遥测加一行 `P`（10Hz，别碰现有短行格式）
现有遥测状态机里加（照现有节流模式）：每 10 拍输出
`P x=<int> y=<int> yaw=<%+.1f>`（x/y 取整 mm，yaw 度）。
FW_TAG 换成带分支标识，如 `NAV-0926-HHMM`（真实时间）。

### A4. PC 收图脚本 `_tmp/pose_plot.py`（新分支目录的 _tmp）
- 串口收 `P` 行（参数照 `_tmp/card002_run.ps1` 的端口/波特率抄），存 CSV（`_tmp/pose_runN.csv`）。
- matplotlib 画轨迹存 PNG（`_tmp/pose_runN.png`），无 matplotlib 则 `pip install matplotlib`（还不行就只出 CSV + 终端 ASCII 网格简图）。
- 图上标注：发车原点 ★、终点 ●、闭合差 mm、路径总长 mm、点数。

---

## Part B · 验收（推车，不烧电机）

### B1. 空转验证
- 车静止上电 → `P` 行 10Hz 稳定输出，x/y/yaw 全 0 附近（|yaw| 10 秒漂 <1°）。
- 单独前推 1m（直尺比对）：x≈1000±30，y 漂 <50。
- 原地左转 90°（目视对齐地板缝）：yaw≈-90±5。

### B2. 推车画图（核心验收）
手推车沿赛道**正常跑线**一圈回发车点（含葫芦四圆、直角弯；推稳一点，别打滑）。跑 2 次存 2 份 CSV。
- ✅ 图上 4 个圆清晰可辨、直角弯处轨迹方正；
- ✅ 记录闭合差（起终点直线距离）与路径总长。

### B3. Go/No-Go 判据（军师已定，回报数据即可，不用自己下结论）
| 闭合差/全程 | 判定 | 下一步 |
|---|---|---|
| < 1% | **GO** | CARD-006：轨迹平滑重采样成路径表 |
| 1%~2% | GO（带条件） | 可用，红外观测中途修正提为 CARD-007 必做项 |
| 2%~5% | 边缘 | 先治 IMU（静止零偏复测/采样率 100→200Hz）再推一次 |
| > 5% | **NO-GO** | 此路不通的证据，回主线，数据留档 |

---

## 后续路线图（预告，不展开；CARD-005 GO 后逐卡细化）
- **CARD-006** 路径表：推车 CSV → PC 平滑+固定间距重采样 → 生成 `nav_path.h`（x/y/yaw/曲率/速度档）编译进固件（照马铃薯模式，不依赖他的生成器）。
- **CARD-007** 回放控制：投影(ey/epsi/进度s) + 曲率前馈 + Frenet 回正 + 陀螺闭环；红外循迹 PD 降级为 fallback（投影失败/丢图时切回，保命）。
- **CARD-008** 速度表按曲率分档 + 葫芦专项验收（4 圆 1 圈出）。
- 二期：红外观测做低频位姿修正（对应马铃薯点云定位的粗化版）。

## 注意
- 本卡【只做感知+记录】，一行控制代码都不写；不动主线 `test17` 目录任何文件。
- worktree 编译：在 `test17-nav` 目录用同一 MDK 工程，输出独立；`FW_TAG` 带分支名。
- 编译 0 Error 0 Warning；`P` 行字符串字面量不写中文。
- 若 worktree 命令报错（锁/权限），原样回报别硬来。
- 推车数据是这条路线的第一份资产：CSV 原样保存，别加工。
