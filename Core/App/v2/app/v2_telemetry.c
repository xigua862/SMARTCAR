#include "v2_telemetry.h"
#include "v2_uart.h"
#include "v2_app_task.h"
#include "main.h"

/* ★FW_TAG 取真实时间（写代码时跑 date +%m%d-%H%M 抄进来） */
#define V2_FW_TAG  "M0-1009-1733"

void v2_telemetry_banner(void)
{
  v2_uart_puts("\r\n===== CAR FW v2 (Suika Lab) =====");
  v2_uart_printf("\r\n tag : %s  built: %s %s", V2_FW_TAG, __DATE__, __TIME__);
  v2_uart_puts("\r\n arch: 4-layer, TIM1 1kHz ISR, 100Hz loop, UART poll");
  v2_uart_puts("\r\n cmd : V | T <1|2> <pwm> | Z | X");
  v2_uart_puts("\r\n$V2,FW,ms,IR,act,err,OD,YAW,GZ,ISRus,OL,CT\r\n");
}

void v2_telemetry_csv(void)
{
  line_reading_t lr;
  pose_t         p;
  uint32_t       us_avg = 0u, ol = 0u, ct = 0u;

  v2_app_line_take(&lr);
  v2_app_pose_take(&p);
  v2_app_isr_take(&us_avg, &ol, &ct);

  v2_uart_printf("$V2,%s,%lu,%02X,%u,%d,%ld,%ld,%d,%lu,%lu,%lu\r\n",
                 V2_FW_TAG,
                 (unsigned long)HAL_GetTick(),
                 (unsigned)lr.bitmap,
                 (unsigned)lr.active,
                 (int)lr.err_x100,
                 (long)p.od_mm,
                 (long)p.yaw_mdeg,
                 (int)p.dps_x100,
                 (unsigned long)us_avg,
                 (unsigned long)ol,
                 (unsigned long)ct);
}
