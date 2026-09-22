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
uint8_t line_follow_turn_commit(void);   /* >0 = 正在分支口硬转 */  /* 1 = 直线提速档生效中 */
#endif /* LINE_FOLLOW_H */
