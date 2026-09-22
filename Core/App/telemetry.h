#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/* 串口遥测 + 在线调参(115200)。
   打印: FW:版本号 IR:位图 RPM1 RPM2 KP KD SP ST
   命令: P <KP> / S <speed> / D <KD> / T <1|2> <speed> / H / B(蜂鸣) / V(版本) / X(急停) */

void telemetry_init(void);
void telemetry_msg(const char *msg);   /* 状态机事件打印(自带换行) */              /* 使能 USART1 RXNE 中断(寄存器级) */
void telemetry_banner(void);            /* 上电横幅: 版本/编译时间/参数(确认烧没烧) */
void telemetry_report(void);            /* 待机 500ms / 运行中 50ms 打印一行状态 */
void telemetry_process_command(void);   /* 主循环处理攒好的串口命令 */

#endif /* TELEMETRY_H */
