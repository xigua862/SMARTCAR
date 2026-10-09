#ifndef V2_TUNE_H
#define V2_TUNE_H

/* v2 在线调参（冻结@M0）：命令白名单一行一条
   M0 命令：V(版本) / T <1|2> <pwm>(点动) / Z(里程清零) / X(急停=停电机)
   W/E/$CSV 等留给 M4（参数落盘/全参数读出/流式遥测） */
void v2_tune_poll(void);

#endif /* V2_TUNE_H */
