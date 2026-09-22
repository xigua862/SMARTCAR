#ifndef KEY_H
#define KEY_H

#include <stdint.h>

/* 启动按键：PB9，按下 = 接地（gpio.c 已开内部上拉）
   软件消抖：UI_PERIOD_MS(10ms) × KEY_DEBOUNCE_TICKS(3) = 30ms
   事件：短按(松开时), 长按(按住 >= KEY_LONG_PRESS_MS) */
void    key_init(void);
void    key_update(void);          /* 每个 UI 节拍(10ms)调用一次 */
uint8_t key_take_press(void);      /* 取走"短按"事件(取走即清零) */
uint8_t key_take_long(void);       /* 取走"长按"事件(取走即清零) */
uint8_t key_is_down(void);         /* 当前是否按住(已消抖) */
void    key_clear_events(void);    /* 丢弃未处理的事件(进单电机测试时调, 防误启动) */

#endif /* KEY_H */
