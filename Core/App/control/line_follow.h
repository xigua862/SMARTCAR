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
uint16_t line_follow_1k_missed(void);    /* ★本窗口 100Hz 漏看了几拍 1kHz 采样(遥测 K1=) */
/* ★本窗口 1kHz 采样的完整统计（遥测 K1=missed/total 与 MX=maxact/maxraw）。
   ★必须读这个、不要再直接调 line_1k_take —— 那个会清零窗口，
     相切点识别已经取走了 maxact/maxraw，两边各取一次谁都看不全。 */
void line_follow_1k_stats(uint16_t* missed, uint16_t* total, uint8_t* maxact, uint16_t* maxraw);
uint8_t line_follow_boost_active(void);  /* 1 = 直线提速档生效中 */

/* ★弯道计数 → 强制右转（2026-09-22 他提的方案） */
uint16_t line_follow_g7_count(void);     /* 最右一路(bit7)已触发次数(遥测 G7=, 边沿计数) */
uint8_t  line_follow_g7_flag(void);      /* 1 = 已触发过"出葫芦弯道"(锁存, 遥测 GX=) */
uint8_t  line_follow_turn_left(void);    /* 硬转剩余拍数(>0 = 正在硬转, 遥测 GR=) */

/* ★十字事件计数（2026-09-22 15:31 修"葫芦出口被误当十字"时加的） */
uint16_t line_follow_cross_events(void);  /* 一趟里"判成十字"的次数(遥测 CX=) */
uint16_t line_follow_branch_events(void); /* 一趟里"宽图案只贴一端→拒绝当十字"的次数(遥测 BR=) */

/* ★葫芦圈出口（2026-09-23）：'只有最左 N 路亮'→右转出圈；这个是触发次数(遥测 GE=) */
uint16_t line_follow_gourd_exit_events(void);
/* ★上次出口右转实际转了多少度(遥测 GD=) —— 用来标定 GOURD_EXIT_TURN_DEG。
   IMU 没通时恒为 0。 */
float    line_follow_gourd_exit_last_deg(void);
/* ★当前是否在葫芦圈里(遥测 GD 旁边用来核对) —— 出圈闸门与丢线屏蔽都用它 */
uint8_t  line_follow_in_gourd(void);
/* ★★ 新的一趟开始：给"出圈右转"重新上膛（一次性锁存复位）。
   car_fsm.c 在 CAR_RUN 入口、紧跟 odom_reset() 之后调用。
   ★必须在每趟起步时调 —— 否则第二趟就不会再右转了。 */
void     line_follow_gourd_rearm(void);
/* ★"从进圈那一刻起算"的净位移(mm)（遥测 ODE=）—— 标定 GOURD_EXIT_ODOM_MM 用。
   不在圈里时返回 0。 */
int32_t  line_follow_gourd_odom_mm(void);
uint16_t line_follow_edge_hold_cnt(void);  /* 最边两路掉路容忍补过几次(遥测 EH=) */
#endif /* LINE_FOLLOW_H */
