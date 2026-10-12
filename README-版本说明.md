# BACKUP — 1008-R35（冻结基准）

- **版本标签**: 1008-R35      **git 提交**: 85920fa      **分支**: fw/1008-R35      **git tag**: backup-1008-R35
- **冻结时间**: 2026-10-12 17:34
- **用途**: 用户要求"以后从这个版本开始改" ⇒ 本目录是只读基准，**不要在它里面改代码**；
  要改就复制出去（或从 git 的 backup-1008-R35 开新分支）。

## 这一版包含什么（都是实测验证过的行为）

| 项 | 值 |
|---|---|
| 里程闸门 | 里程 >= 10700 才启用葫芦圈逻辑（挡直角弯干扰）|
| 减速 | **已关闭**（GOURD_EXIT_SLOW_EN=0；R29 那版"过 10679 之后一直慢"有缺陷，待重做）|
| 圈内 KP | 2.0（GOURD_KP），闸门外 2.3 |
| 分支口判据 | 两坨 或 一坨且亮路数 >= 5 |
| 相位翻转 | 每拍更新"两坨"历史，上升沿翻一次（R34 修掉"L,R,R,R"的 bug）|
| PWM 下限 | WHEEL_FLOOR_MIN_CMD = 2（补"1~8 空洞"，治一顿一顿）|
| 出圈 | 判据⑤就位但 CALIB_ONLY=1（物理上不可能误转向），缺"停 T 上"标定 |
| 版本号 | 串口横幅 = 1008-R35 |

## 构建 / 烧录

    C:\keil5\UV4\UV4.exe -r MDK-ARM\test16.uvprojx -j0 -o MDK-ARM\build.log   # 编译（要求 0 错 0 警）
    C:\keil5\UV4\UV4.exe -f MDK-ARM\test16.uvprojx -j0 -o MDK-ARM\flash.log   # 烧录
    # 报 Internal DLL Error / Target DLL has been cancelled = 目标板没上电（或 DAPLink 卡死）

## 读黑匣子（RAM 台账 + 轨迹）

    _work\read_trace.ps1        # 需 PowerShell -ExecutionPolicy Bypass；用 DAPLink 直读，不复位
    # ⚠️ 车不要断电、不要复位，否则 RAM 里的台账/轨迹会丢
