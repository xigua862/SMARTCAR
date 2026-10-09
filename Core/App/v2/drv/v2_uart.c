#include "v2_uart.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>

static volatile uint8_t  s_rx_buf[V2_UART_RX_SIZE];
static volatile uint16_t s_rx_head = 0u, s_rx_tail = 0u;
static volatile uint8_t  s_tx_buf[V2_UART_TX_SIZE];
static volatile uint16_t s_tx_head = 0u, s_tx_tail = 0u;
static volatile uint16_t s_rx_total = 0u, s_rx_over = 0u;

void v2_uart_init(void)
{
  /* PA10(RX) 重配为"输入 + 上拉"（旧教训：悬空被电机 EMI 灌入 -> 假命令 -> 锁车） */
  GPIO_InitTypeDef gi = {0};
  gi.Pin  = GPIO_PIN_10;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gi);

  /* 清掉上电残留标志 */
  (void)USART1->SR;
  (void)USART1->DR;

  s_rx_head = s_rx_tail = 0u;
  s_tx_head = s_tx_tail = 0u;
  s_rx_total = s_rx_over = 0u;

  /* 确保收发使能（HAL 初始化后本应已开；这里显式兜底） */
  USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);
}

void v2_uart_poll(void)
{
  /* ---- RX：把硬件里所有已到的字节收进环形缓冲 ---- */
  while ((USART1->SR & USART_SR_RXNE) != 0u)
  {
    uint8_t b = (uint8_t)(USART1->DR & 0xFFu);   /* 读 DR 同时清 RXNE/ORE/NE/FE */
    uint16_t nxt;

    if (s_rx_total < 60000u) s_rx_total++;

    nxt = (uint16_t)((s_rx_head + 1u) & (uint16_t)(V2_UART_RX_SIZE - 1u));
    if (nxt == s_rx_tail)
    {
      if (s_rx_over < 60000u) s_rx_over++;        /* 满 -> 丢这个字节 */
    }
    else
    {
      s_rx_buf[s_rx_head] = b;
      s_rx_head = nxt;
    }
  }

  /* ---- TX：有空闲就吐一个字节（非阻塞） ---- */
  while ((s_tx_tail != s_tx_head) && ((USART1->SR & USART_SR_TXE) != 0u))
  {
    USART1->DR = s_tx_buf[s_tx_tail];
    s_tx_tail = (uint16_t)((s_tx_tail + 1u) & (uint16_t)(V2_UART_TX_SIZE - 1u));
  }
}

int v2_uart_getc(void)
{
  int c;
  if (s_rx_tail == s_rx_head) return -1;
  c = (int)s_rx_buf[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1u) & (uint16_t)(V2_UART_RX_SIZE - 1u));
  return c;
}

uint16_t v2_uart_rx_avail(void)
{
  return (uint16_t)((s_rx_head - s_rx_tail) & (uint16_t)(V2_UART_RX_SIZE - 1u));
}

void v2_uart_putc(char c)
{
  uint16_t nxt = (uint16_t)((s_tx_head + 1u) & (uint16_t)(V2_UART_TX_SIZE - 1u));
  if (nxt == s_tx_tail) return;                 /* 满 -> 丢（不许在此阻塞等待） */
  s_tx_buf[s_tx_head] = (uint8_t)c;
  s_tx_head = nxt;
}

void v2_uart_puts(const char* s)
{
  while ((s != 0) && (*s != '\0')) { v2_uart_putc(*s); s++; }
}

void v2_uart_printf(const char* fmt, ...)
{
  char buf[160];
  va_list ap;
  int n, i;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n < 0) return;
  if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
  for (i = 0; i < n; i++) v2_uart_putc(buf[i]);
}

uint16_t v2_uart_rx_total(void) { return s_rx_total; }
uint16_t v2_uart_rx_over(void)  { return s_rx_over; }
