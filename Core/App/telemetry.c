#include "telemetry.h"
#include "app.h"
#include "app_config.h"
#include "usart.h"             /* huart1 */
#include "drivers/motor.h"
#include "drivers/line_sensor.h"
#include "drivers/speed.h"
#include "drivers/encoder.h"
#include "drivers/key.h"
#include "drivers/buzzer.h"
#include "drivers/led.h"
#include "drivers/imu.h"
#include "main.h"
#include "control/line_follow.h"
#include "fsm/car_fsm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- 串口逐字节攒命令(寄存器级, 中断里攒, 主循环里取) ---- */
static          char     cmd_line[32];
static          uint8_t  cmd_len   = 0;
static volatile uint8_t  cmd_ready = 0;

/* ★2026-09-22：给状态机用的"事件打印"（急停原因/解除等），自带换行 */
void telemetry_msg(const char *msg)
{
  uint16_t n = 0;
  while (msg[n] != 0 && n < 100) n++;
  if (n) HAL_UART_Transmit(&huart1, (uint8_t*)msg, n, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
}

void telemetry_init(void)
{
  HAL_NVIC_EnableIRQ(USART1_IRQn);          /* 开串口接收中断(在线调参) */
  USART1->CR1 |= USART_CR1_RXNEIE;          /* 直接使能RXNE中断(绕开HAL状态机, 防卡死) */

  /* ★2026-09-22：PA10(RX) 悬空时会被电机 EMI 灌入 → 凑出假命令(尤其 X → 锁车)。
     重新配成"输入 + 上拉"，空闲电平稳稳在高；线没接也不会误触发。 */
  {
    GPIO_InitTypeDef gi = {0};
    gi.Pin  = GPIO_PIN_10;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gi);
  }
}

/* 上电横幅：一眼确认"烧没烧、烧的是哪版"（编译时间由编译器自动填，不用手工维护） */
void telemetry_banner(void)
{
  char buf[192];
  int n = snprintf(buf, sizeof(buf),
    "\r\n===== CAR FW (Suika Lab) =====\r\n"
    " tag : %s   built: %s %s\r\n"
    " KP=%d.%d KD=%d.%d SP=%d(curve %d)\r\n"
    " ST=%d  LINE_ACTIVE=%d  AUTO_START=%d\r\n"
    "===============================\r\n",
    FW_TAG, __DATE__, __TIME__,
    kp_x10 / 10, kp_x10 % 10, kd_x10 / 10, kd_x10 % 10, sp_straight, sp_curve,
    (int)car_fsm_state(), (int)LINE_ACTIVE_LEVEL, (int)AUTO_START_ENABLE);
  if (n > 0)
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, 200);
}

/* 打印当前状态: 版本标签 + IR 8 路位图 + RPM + KP/KD/SP + 状态机状态 */
void telemetry_report(void)
{
  speed_update();                            /* 计算本窗口 RPM 并清零窗口计数 */
  line_reading_t r = line_read();

  char ir[LINE_CHANNELS + 1];
  for (uint8_t i = 0; i < LINE_CHANNELS; i++)
    ir[i] = (r.raw >> i) & 1u ? '1' : '0';
  ir[LINE_CHANNELS] = '\0';

  char buf[224];
  int n = snprintf(buf, sizeof(buf),
    "FW:%s IR:%s RPM1=%d RPM2=%d KP=%d.%d SP=%d ST=%d GZ=%d GW=%d SB=%d G7=%d GX=%d GR=%d CX=%d BR=%d\r\n",
    FW_TAG, ir,
    speed_get_rpm(MOTOR_LEFT), speed_get_rpm(MOTOR_RIGHT),
    kp_x10 / 10, kp_x10 % 10,
    sp_straight, (int)car_fsm_state(),
    (int)imu_get_gyro_z(), (int)line_follow_gourd_waves(), (int)line_follow_boost_active(),
    (int)line_follow_g7_count(),      /* ★最右一路(bit7)已触发次数 —— 用它定 G7_TURN_TRIG */
    (int)line_follow_g7_flag(),       /* 1 = 已触发过"出葫芦弯道" */
    (int)line_follow_turn_left(),     /* >0 = 正在强制右转（剩余拍数） */
    (int)line_follow_cross_events(),  /* ★判成十字的次数 —— 一趟应 = 赛道上的真十字数(2) */
    (int)line_follow_branch_events());/* ★"只贴一端→拒绝当十字"的次数 = 被救回来的出口数 */
  if (n > 0)
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, 100);
}

/* 处理一条完整命令(主循环调用, 非阻塞) */
void telemetry_process_command(void)
{
  if (!cmd_ready) return;
  cmd_ready = 0;

  char  c = (char)(cmd_line[0] & 0xDF);   /* 首字母转大写 */
  float v = atof(cmd_line + 1);           /* 兼容 "P 15" 和 "P15" */

  /* ★2026-09-22：白名单 —— 只认已知命令；噪声凑出来的字符串直接丢掉，不回话 */
  if (strchr("PSDTMLBRVHX", c) == NULL) return;
  char buf[112];
  int  n = 0;

  if (c == 'P')
  {
    if (v >= 0.5f && v <= 50.0f) kp_x10 = (int16_t)(v * 10.0f + 0.5f);
    n = snprintf(buf, sizeof(buf), "OK KP=%d.%d\r\n", kp_x10 / 10, kp_x10 % 10);
  }
  else if (c == 'S')                        /* 直道速度; 弯道速度按比例跟着走 */
  {
    test_mode = 0;                          /* S 同时退出单电机测试模式 */
    if (v >= 0.0f && v <= 99.0f)
    {
      base_speed  = (int16_t)(v + 0.5f);
      sp_straight = base_speed;
      sp_curve    = (int16_t)(v * CURVE_SPEED_RATIO + 0.5f);
      if (sp_curve < 10) sp_curve = 10;
    }
    n = snprintf(buf, sizeof(buf), "OK SP=%d CURVE=%d\r\n", sp_straight, sp_curve);
  }
  else if (c == 'D')
  {
    if (v >= 0.0f && v <= 20.0f) kd_x10 = (int16_t)(v * 10.0f + 0.5f);
    n = snprintf(buf, sizeof(buf), "OK KD=%d.%d\r\n", kd_x10 / 10, kd_x10 % 10);
  }
  else if (c == 'H')                        /* 诊断: 8 路原始位图 + 按键 + 编码器 + IMU */
  {
    line_reading_t r = line_read();
    char ir[LINE_CHANNELS + 1];
    for (uint8_t i = 0; i < LINE_CHANNELS; i++)
      ir[i] = (r.raw >> i) & 1u ? '1' : '0';
    ir[LINE_CHANNELS] = '\0';

    n = snprintf(buf, sizeof(buf),
      "RAW=%s ERR=%d BTN=%d ENC1=%ld ENC2=%ld WHO=%02X IMU=%d GZ=%d\r\n",
      ir, (int)r.error, (int)key_is_down(),
      (long)encoder_get_count(MOTOR_LEFT), (long)encoder_get_count(MOTOR_RIGHT),
      imu_who_am_i(), (int)imu_is_present(), (int)imu_get_gyro_z());
  }
  else if (c == 'T')                        /* T <1|2> <速度>: 单电机测试(架空用) */
  {
    int mot = 0, spd = 0;
    if (sscanf(cmd_line + 1, "%d %d", &mot, &spd) == 2)
    {
      if (spd >  99) spd =  99;
      if (spd < -99) spd = -99;
      if (mot == 1)      { motor_set_speed(MOTOR_LEFT,  (int16_t)spd); test_mode = 1; }
      else if (mot == 2) { motor_set_speed(MOTOR_RIGHT, (int16_t)spd); test_mode = 1; }
      else               { motor_stop(); test_mode = 0; }   /* T 0 0 = 停止测试 */
      if (test_mode) key_clear_events();   /* 防测试期间攒下的按键事件回来误启动 */
      n = snprintf(buf, sizeof(buf), "OK T%d=%d\r\n", mot, spd);
    }
    else
    {
      n = snprintf(buf, sizeof(buf), "USE: T <1|2> <speed -99~99>\r\n");
    }
  }
  else if (c == 'V')                        /* 版本: 重新打印开机横幅 */
  {
    telemetry_banner();
    n = 0;
  }
  else if (c == 'R')                        /* 串口启动: 等效按一下启动键(读秒→开跑) */
  {
    test_mode = 0;
    car_fsm_request_start();
    n = snprintf(buf, sizeof(buf), "OK START ST=%d\r\n", (int)car_fsm_state());
  }
  else if (c == 'L')                        /* L <0~7>: 直接点灯 bit0=LED1 bit1=LED2 bit2=LED3 */
  {
    int m = atoi(cmd_line + 1);
    if (m < 0) m = 0;
    if (m > 7) m = 7;
    led_mask((uint8_t)m);
    n = snprintf(buf, sizeof(buf), "OK LED mask=%d\r\n", m);
  }
  else if (c == 'M')                        /* M <左> <右>: 直接给两轮 PWM(-99~99), 进测试模式 */
  {
    int l = 0, r = 0;
    if (sscanf(cmd_line + 1, "%d %d", &l, &r) == 2)
    {
      if (l >  99) l =  99;  if (l < -99) l = -99;
      if (r >  99) r =  99;  if (r < -99) r = -99;
      motor_set_differential((int16_t)l, (int16_t)r);
      test_mode = 1;
      key_clear_events();
      n = snprintf(buf, sizeof(buf), "OK M L=%d R=%d\r\n", l, r);
    }
    else
    {
      n = snprintf(buf, sizeof(buf), "USE: M <left -99~99> <right -99~99>\r\n");
    }
  }
  else if (c == 'B')                        /* B [毫秒]: 蜂鸣器响(默认500ms; 给长一点方便量电路) */
  {
    int ms = (int)v;                        /* "B" 单独发时 atof 得 0 → 用默认 500 */
    if (ms <= 0)     ms = 500;
    if (ms > 10000)  ms = 10000;
    buzzer_beep((uint16_t)ms);
    n = snprintf(buf, sizeof(buf), "OK BEEP %dms\r\n", ms);
  }
  else if (c == 'X')                        /* ★急停：必须连发两个 X（XX），防噪声误触发锁车 */
  {
    if ((cmd_line[1] & 0xDF) == 'X')
    {
      test_mode = 0;
      car_fsm_emergency_stop_cause(2u);     /* 2 = 串口触发 */
      n = snprintf(buf, sizeof(buf), "OK EMERGENCY-STOP by SERIAL (ST=4; hold START 2s to clear)\r\n");
    }
    else
    {
      n = snprintf(buf, sizeof(buf), "USE: XX (two letters) to emergency-stop\r\n");
    }
  }
  else
  {
    n = snprintf(buf, sizeof(buf),
      "CMD: P<KP> / S<SPD> / D<KD> / T<1|2><spd> / M<L><R> / L<0~7> / B[ms] / R(启动) / V(版本) / H(diag) / XX(emergency stop)\r\n");
  }
  if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, 100);
}

/* USART1 中断入口(寄存器级接收)。从旧 main.c 迁移。
   即使 ORE 溢出也只清标志继续收, 不会像 HAL Receive_IT 那样卡死 */
void USART1_IRQHandler(void)
{
  uint32_t sr = USART1->SR;
  if (sr & USART_SR_RXNE)                       /* 收到一字节 */
  {
    uint8_t b = (uint8_t)(USART1->DR & 0xFF);   /* 读 DR 自动清 RXNE */
    if (b == '\n' || b == '\r')                 /* 回车/换行 = 一条命令结束 */
    {
      if (cmd_len > 0)
      {
        cmd_line[cmd_len] = '\0';
        cmd_len = 0;
        cmd_ready = 1;
      }
    }
    else if (cmd_len < (uint8_t)(sizeof(cmd_line) - 1))
    {
      cmd_line[cmd_len++] = (char)b;
    }
  }
  else if (sr & USART_SR_ORE)                   /* 溢出: 读 DR 清标志, 防止 RXNE 中断失效 */
  {
    (void)USART1->DR;
  }
}
