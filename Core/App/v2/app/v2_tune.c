#include "v2_tune.h"
#include "v2_uart.h"
#include "v2_motor.h"
#include "v2_odom.h"
#include "v2_pose.h"
#include "v2_app_task.h"
#include "main.h"

#define V2_TUNE_LINE_MAX 32

static char    s_line[V2_TUNE_LINE_MAX];
static uint8_t s_len = 0u;

static const char* v2_tune_skip_sp(const char* p)
{
  while (*p == ' ') p++;
  return p;
}

static int32_t v2_tune_parse_int(const char** pp)
{
  int32_t v = 0;
  uint8_t neg = 0u;
  const char* p = *pp;

  if (*p == '-') { neg = 1u; p++; }
  while ((*p >= '0') && (*p <= '9'))
  {
    v = v * 10 + (int32_t)(*p - '0');
    p++;
  }
  *pp = p;
  return neg ? -v : v;
}

static void v2_tune_exec(char* s)
{
  if (s[0] == 'V')
  {
    uint32_t us_avg = 0u, ol = 0u, ct = 0u;
    v2_app_isr_take(&us_avg, &ol, &ct);
    v2_uart_printf("OK V2 M0  ISRus(last)=%lu OL=%lu RX=%u/%u\r\n",
                   (unsigned long)v2_app_isr_us_last(),
                   (unsigned long)ol,
                   (unsigned)v2_uart_rx_total(),
                   (unsigned)v2_uart_rx_over());
    return;
  }

  if (s[0] == 'Z')
  {
    v2_odom_reset();
    v2_uart_puts("OK Z odom=0\r\n");
    return;
  }

  if (s[0] == 'X')
  {
    v2_motor_stop();
    v2_uart_puts("OK X motor stop\r\n");
    return;
  }

  if (s[0] == 'T')
  {
    const char* p = v2_tune_skip_sp(s + 1);
    int32_t ch, pwm;
    if ((*p < '1') || (*p > '2'))
    {
      v2_uart_puts("ERR T: ch 1|2\r\n");
      return;
    }
    ch  = v2_tune_parse_int(&p);
    p   = v2_tune_skip_sp(p);
    pwm = v2_tune_parse_int(&p);

    if (pwm > V2_MOTOR_MAX)  pwm = V2_MOTOR_MAX;
    if (pwm < -V2_MOTOR_MAX) pwm = -V2_MOTOR_MAX;

    v2_motor_set((int)(ch - 1), (int16_t)pwm);
    v2_uart_printf("OK T %ld %ld\r\n", (long)ch, (long)pwm);
    return;
  }

  v2_uart_puts("ERR unknown cmd (V/T/Z/X)\r\n");
}

void v2_tune_poll(void)
{
  int c;

  while ((c = v2_uart_getc()) >= 0)
  {
    if ((c == '\r') || (c == '\n'))
    {
      if (s_len != 0u)
      {
        s_line[s_len] = '\0';
        v2_tune_exec(s_line);
        s_len = 0u;
      }
      continue;
    }
    if (s_len < (uint8_t)(V2_TUNE_LINE_MAX - 1u))
    {
      s_line[s_len++] = (char)c;
    }
  }
}
