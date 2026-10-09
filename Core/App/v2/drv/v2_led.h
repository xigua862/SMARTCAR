#ifndef V2_LED_H
#define V2_LED_H

#include <stdint.h>

/* v2 三灯（冻结@M0）：LED1=PC13 / LED2=PB14 / LED3=PB15
   实物接法 = 端口 -> LED -> 1k -> GND：高电平点亮 */
#define V2_LED_COUNT 3

void    v2_led_init(void);
void    v2_led_set(uint8_t id, uint8_t on);
void    v2_led_all(uint8_t on);
void    v2_led_mask(uint8_t mask);
uint8_t v2_led_get_mask(void);

#endif /* V2_LED_H */
