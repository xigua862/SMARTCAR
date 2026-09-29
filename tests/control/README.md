# 主控制宿主回归

运行 `powershell -ExecutionPolicy Bypass -File tests/control/run.ps1`。

测试直接包含实际 `line_follow.c`，链接实际 `filter.c` 和 `pid.c`，仅模拟红外、陀螺、测速、编码器、串口和电机硬件接口。覆盖单拍丢线恢复、旋转找线及恢复时的历史清理、十字强制直行、葫芦圈连续离开判据和停车遥测一致性。

脚本分别编译默认对称转向记忆及 `USE_SYMMETRIC_CORR_TRIM=0` 回退分支，两种配置各运行五个用例。回退配置只在测试翻译单元内覆盖，不改写工程的 `app_config.h`。

这些测试验证软件状态与输出，不模拟车辆动力学，不代表实车已经通过葫芦圈。编译器固定为本机 RedPanda-Cpp 的 MinGW64 GCC；换电脑可调整脚本中的编译器目录。
