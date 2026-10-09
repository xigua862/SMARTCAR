#ifndef V2_KEY_H
#define V2_KEY_H

#include <stdint.h>

/* v2 启动键（冻结@M0）：PB9 上拉，按下 = 低
   消抖 3 × UI 节拍(10ms) = 30ms；长按 1000ms = 急停语义（M0 只产生事件） */
void    v2_key_init(void);
void    v2_key_update(void);        /* 10ms 调一次 */
uint8_t v2_key_take_press(void);
uint8_t v2_key_take_long(void);
uint8_t v2_key_is_down(void);
void    v2_key_clear_events(void);

#endif /* V2_KEY_H */
