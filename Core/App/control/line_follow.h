#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* 顶层循迹控制: P/D 差速 + 速度自适应 + 丢线旋转 + 十字强制直行。
   由 car_fsm 在 CAR_RUN 状态每 CTRL_PERIOD_MS 调用一次。 */
void line_follow_init(void);
void line_follow_control(int16_t base);
void line_follow_stop(void);

/* 状态查询(给状态机做指示灯/终点判定用) */
uint8_t line_follow_is_lost(void);    /* 1 = 持续丢线(正在原地找线) */
uint8_t line_follow_on_cross(void);   /* 1 = 当前判定为十字(强制直行中) */

uint8_t line_follow_gourd_waves(void);   /* 葫芦弯已识别的相切点个数(0~2, 到3清零) */
uint8_t line_follow_boost_active(void);  /* 1 = 直线提速档生效中 */

/* ★弯道计数 → 强制右转（2026-09-22 他提的方案） */
uint16_t line_follow_g7_count(void);     /* 最右一路(bit7)已触发次数(遥测 G7=, 边沿计数) */
uint8_t  line_follow_g7_flag(void);      /* 1 = 已触发过"出葫芦弯道"(锁存, 遥测 GX=) */
uint8_t  line_follow_turn_left(void);    /* 硬转剩余拍数(>0 = 正在硬转, 遥测 GR=) */

/* ★十字事件计数（2026-09-22 15:31 修"葫芦出口被误当十字"时加的） */
uint16_t line_follow_cross_events(void);  /* 一趟里"判成十字"的次数(遥测 CX=) */
uint16_t line_follow_branch_events(void); /* 一趟里"宽图案只贴一端→拒绝当十字"的次数(遥测 BR=) */
#endif /* LINE_FOLLOW_H */
