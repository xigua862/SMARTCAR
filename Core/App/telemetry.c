#include "telemetry.h"
#include "app.h"
#include "app_config.h"
#include "usart.h"             /* huart1 */
#include "drivers/motor.h"
#include "drivers/line_sensor.h"
#include "drivers/speed.h"
#include "drivers/encoder.h"
#include "drivers/odom.h"        /* ★里程计: 遥测 OD= / PATH= */
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

  char buf[240];
  int n = snprintf(buf, sizeof(buf),
    "FW:%s IR:%s RPM1=%d RPM2=%d KP=%d.%d SP=%d ST=%d DT=%d OD=%ld PATH=%ld ODE=%ld OS=%d GZ=%d GW=%d SB=%d G7=%d GX=%d GR=%d CX=%d BR=%d GE=%d GD=%d IG=%d EH=%d\r\n",
    FW_TAG, ir,
    speed_get_rpm(MOTOR_LEFT), speed_get_rpm(MOTOR_RIGHT),
    kp_x10 / 10, kp_x10 % 10,
    sp_straight, (int)car_fsm_state(),
    (int)loop_dt_ms,                  /* 实测控制周期(ms) */
    (long)odom_distance_mm(),         /* ★OD: 净位移(mm), 从起步算起, 前进为正 —— 做"到点强制右转"就用它 */
    (long)odom_path_mm(),             /* ★PATH: 累计路程(mm), 只加不减 */                  /* ★DT: 实测控制周期(ms)。应稳定在 10 附近;
                                         明显 >10 = 主循环被拖慢(过采样过多 或 IMU 的 I2C 阻塞) */
    (long)line_follow_gourd_odom_mm(),/* ★ODE: 从【进圈那一刻】起算的净位移(mm)。
                                         标定 GOURD_EXIT_ODOM_MM 就抄这个数：
                                         出圈事件行里也会打印一份。
                                         不在圈里(IG=0)时恒为 0 —— 别拿它当里程表用 */
    (int)line_os_disagree_take(),     /* ★OS: 本窗口内"同一拍子采样位图不一致"的拍数。
                                         >0 = 快采样抓到了单次采样会漏掉的东西(过采样生效)
                                         =0 = 这几个µs内红外没变(模块跟不上/间距太小) */
    (int)imu_get_gyro_z(), (int)line_follow_gourd_waves(), (int)line_follow_boost_active(),
    (int)line_follow_g7_count(),      /* ★最右一路(bit7)已触发次数 —— 用它定 G7_TURN_TRIG */
    (int)line_follow_g7_flag(),       /* 1 = 已触发过"出葫芦弯道" */
    (int)line_follow_turn_left(),     /* >0 = 正在强制右转（剩余拍数） */
    (int)line_follow_cross_events(),  /* ★判成十字的次数 —— 一趟应 = 赛道上的真十字数(2) */
    (int)line_follow_branch_events(), /* ★"只贴一端→拒绝当十字"的次数 = 被救回来的出口数 */
    (int)line_follow_gourd_exit_events(), /* ★葫芦圈出口触发次数。正常=出圈次数;
                                              在右直角弯处乱涨 = 该开闸门 GOURD_EXIT_GATE_WAVES=2 */
    (int)line_follow_gourd_exit_last_deg(), /* ★上次出圈实际转了多少度 —— 标定用:
                                               应≈90; 小了=没转够, 大了=转过头
                                               (IMU 没通时恒为 0) */
    (int)line_follow_in_gourd(),            /* ★IG: 1=当前在葫芦圈里（此时丢线兜底被屏蔽） */
    (int)line_follow_edge_hold_cnt());      /* ★EH: 最边两路"掉路容忍"补过几次（累计）
                                                涨得多 = 相切处最边传感器熄灭确实在发生 */
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

  /* ★2026-09-22：白名单 —— 只认已知命令；噪声凑出来的字符串直接丢掉，不回话
     ★2026-09-23 加 O / Z：里程计读数与清零（手推标定葫芦圈出口里程用） */
  if (strchr("PSDTMLBRVHXOZ", c) == NULL) return;
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
  else if (c == 'O')                        /* ★O: 立刻打印里程（手推标定用，不改变状态） */
  {
    n = snprintf(buf, sizeof(buf),
      "ODOM NOW: OD=%ldmm PATH=%ldmm L=%ld R=%ld\r\n",
      (long)odom_distance_mm(), (long)odom_path_mm(),
      (long)odom_left_mm(), (long)odom_right_mm());
  }
  else if (c == 'Z')                        /* ★Z: 里程清零（把当前位置当作 0 点） */
  {
    odom_reset();
    n = snprintf(buf, sizeof(buf), "OK ODOM RESET (OD=0 PATH=0)\r\n");
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
      /* ★2026-09-23：这段【必须纯 ASCII】。
         原来里面写了中文（启动/版本），加上我新加的"读里程/里程清零"后
         ARMCC 报 `#870-D: invalid multibyte character sequence` ——
         它按 GBK 的双字节规则去扫这个字符串，而源文件是 UTF-8，
         三字节中文被错配成非法双字节序列。
         而且就算编过去，UTF-8 中文发到 GBK 串口终端上也是乱码。
         ★以后往【字符串字面量】里加内容一律用英文；注释里中文没问题。 */
      "CMD: P<KP> / S<SPD> / D<KD> / T<1|2> <spd> / M<L> <R> / L<0~7> / B[ms] / R(start) / V(version) / H(diag) / O(read odom) / Z(reset odom) / XX(emergency stop)\r\n");
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
