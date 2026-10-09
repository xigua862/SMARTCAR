#ifndef V2_BUZZER_H
#define V2_BUZZER_H

#include <stdint.h>

/* ============================================================================
 * v2 蜂鸣器 + 1kHz 节拍源（冻结@M0）
 *   PA11 = TIM1_CH4（无源蜂鸣器，1kHz 方波；旧板 2.7kHz 是另一个时基）
 *   ★TIM1 在 v2 里的主职是【1kHz 更新中断】= 感知/控制节拍；蜂鸣只是搭车
 *   M0 只验证"不冲突"，音调质量 M4 再调
 * ==========================================================================*/

void    v2_buzzer_init(void);          /* TIM1 时基 1kHz + CH4 PWM + NVIC TIM1_UP */
void    v2_buzzer_set(uint8_t on);     /* 1 = 响(1kHz)，0 = 静音 */
void    v2_buzzer_beep(uint16_t ms);   /* 非阻塞：响 ms 毫秒（由 1kHz tick 收尾） */
void    v2_buzzer_tick(void);          /* 1kHz ISR 里调 */
uint8_t v2_buzzer_is_on(void);

#endif /* V2_BUZZER_H */
