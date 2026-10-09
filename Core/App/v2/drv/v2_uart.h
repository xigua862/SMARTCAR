#ifndef V2_UART_H
#define V2_UART_H

#include <stdint.h>

/* ============================================================================
 * v2 串口驱动（冻结@M0）
 *   USART1 PA9/PA10 @115200
 *   ★M0 用【主循环轮询】收发（收发都非阻塞、各有环形缓冲）：
 *     原因：USART1 中断向量被旧 telemetry.c 占着，而 M0 不许改旧树编译/代码；
 *           轮询版反而把"TX 阻塞几毫秒"的隐患一起消掉了（详见回报）。
 *   调用约定：v2_app_loop() 每圈调 v2_uart_poll()，且主循环里不许有长阻塞。
 * ==========================================================================*/

#define V2_UART_RX_SIZE 256
#define V2_UART_TX_SIZE 512

void     v2_uart_init(void);
void     v2_uart_poll(void);            /* 高频调用：RX 收进环、TX 环吐字节 */
int      v2_uart_getc(void);            /* -1 = 无数据 */
uint16_t v2_uart_rx_avail(void);
void     v2_uart_putc(char c);
void     v2_uart_puts(const char* s);
void     v2_uart_printf(const char* fmt, ...);   /* 只允许主循环调用（非 ISR） */
uint16_t v2_uart_rx_total(void);        /* 收到的总字节数（链路体检） */
uint16_t v2_uart_rx_over(void);         /* 溢出丢字节数 */

#endif /* V2_UART_H */
