#ifndef LED_H
#define LED_H

#include <stdint.h>

/* 三颗状态灯：LED1=PC13(蓝/心跳)  LED2=PB14  LED3=PB15
   ★ 实物接法：端口 → LED → 1k → GND = 高电平点亮（LED_ON_LEVEL 在 app_config.h）
   用途：启动读秒 1→2→3、运行指示、丢线闪烁、到站闪光、急停全亮 */
#define LED_COUNT   3
#define LED_ID_1    0
#define LED_ID_2    1
#define LED_ID_3    2

void    led_init(void);
void    led_set(uint8_t id, uint8_t on);   /* 单灯亮/灭 */
void    led_all(uint8_t on);               /* 三灯一起 */
void    led_mask(uint8_t mask);            /* bit0..bit2 对应三灯 */
uint8_t led_get_mask(void);

#endif /* LED_H */
