# FW:1021-1327 = **回退到 1016-R182 原样**（源码 = tag `fw-1021-1327`，提交 `d564ebe`）

这一版就是把 `1021-1318` 那三件事**全部撤销**后的状态，内容与 `5c85504`（1016-R182）**逐字节一致**
（校验：`git diff 5c85504 HEAD -- Core/` 为空；`Code=31916` 与 R182 相同）。

横幅 `FW_TAG` 仍是 **`1016-R182`**（没升版本 tag），所以认版本只能靠 hex 的 SHA-256：

```
cc5358c64219b525f951deee67bf931f2edc81cb966fce59794d7e673d5edd7a  test16.hex
```

## 两版对照（别搞混）

| | `1021-1327`（本目录）= R182 原样 | `1021-1318` = 退掉的那版 |
|---|---|---|
| `USE_BRANCH_EDGE`（分支口接管） | **1 = 开着** | 0 = 关 |
| `GSP_CUT1/2/3_MM`（三个切点里程） | **683 / 1204 / 1706** | 883 / 1404 / 1906（+200）|
| `GSP_ONEWAY_MASK`（单向阀开哪几槽） | **0x06**（只切点2/3） | 0x07（切点1 也屏蔽右转）|
| `Code` | 31916 | 30892 |
| hex SHA-256 前 8 位 | `cc5358c6` | `adc07d42` |

## 想烧哪个

```powershell
cd C:\Users\asus\Desktop\test17
# 本版的 hex（R182 原样）
& "C:/Users/asus/AppData/Local/Programs/xpack-openocd-0.12.0-7/bin/openocd.exe" -f interface/cmsis-dap.cfg -c "transport select swd" -f target/stm32f1x.cfg -c "adapter speed 100" -c "program firmware/1021-1327/test16.hex verify reset exit"

# 1021-1318 那版（切点+200 / 切点1屏蔽右转 / 关接管）—— 文件还在，直接烧就行
#   ... -c "program firmware/1021-1318/test16.hex verify reset exit"
```

源码想切回去看/改：`git checkout fw-1021-1318`（回来：`git checkout main`）。

## 编译信息

Keil ARMCC（`D:/Keil_v5`），`0 Error(s), 0 Warning(s)`，
`Program Size: Code=31916 RO-data=1256 RW-data=604 ZI-data=15076`。
日志：`MDK-ARM/build-0929-R1.log`。

## 仍然没解决（回退不会修好这些）

- **右编码器坏的**（手推 ΔENC2=-1/0/0、跑车 R=0、架空手转 ENC2 不动）⇒ 它的抖动被
  `(|d_l|+|d_r|)/2` 当路程累加（实测虚高：手推 11.4% / 跑车 16.9%）⇒ **里程忽早忽晚的头号原因**。
- **`ENCODER_PPR=422` 是推算值，实测 512** ⇒ RPM 读数偏大 21%（改它会让同档位真车速 +21%，需连档位一起重标）。
- 实测左轮 1 计数 = 531 µm ⇒ 轮径约 86.6mm（设计书 66mm）⇒ 待卡尺确认。
