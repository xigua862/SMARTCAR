#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

/* 无源蜂鸣器：PA11 = TIM1_CH4 PWM → 板上 SS8050 低边驱动 + 1N4148W 续流 + 10k 下拉(防上电乱响)
   ★ 无源 = 必须给方波才响；这里用 PWM 直接给方波（频率 = 音调）
   全部接口非阻塞：响多少毫秒由 buzzer_update() 在 UI 节拍里收尾 */
void    buzzer_init(void);
void    buzzer_on(uint32_t freq_hz);   /* 开声（指定频率） */
void    buzzer_off(void);              /* 静音（占空比归 0，不留直流） */
void    buzzer_beep(uint16_t ms);      /* 响 ms 毫秒后自动停（非阻塞） */
void    buzzer_update(void);           /* 每个 UI 节拍(10ms)调用 */
uint8_t buzzer_is_on(void);

#endif /* BUZZER_H */
