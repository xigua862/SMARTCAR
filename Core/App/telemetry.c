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
/* ★轨迹黑匣子（2026-09-24）：控制拍里记录最近 TRACE_N 拍的 IR/GZ/OD，`Q` 回放。
   为什么要它：跑车时 USB 线会拖，而"过直角弯/出口那几秒"正是最需要看的数据。
   用法：跑完停下 → 串口发 `Q` → 按时间顺序回放 600 拍（6 秒 @100Hz）。
   ⚠️ 回放是阻塞发送（600 行 ≈ 1.6 秒），所以请【停下来再发】。 */
#define TRACE_N 600
typedef struct { uint16_t od; int16_t gz; uint8_t ir; } trace_t;
static trace_t  s_tr[TRACE_N];
static uint16_t s_tr_i = 0u;   /* 写指针 */
static uint16_t s_tr_n = 0u;   /* 已写条数 */

void telemetry_trace_tick(void)
{
  line_reading_t r = line_read();
  s_tr[s_tr_i].od = (uint16_t)odom_distance_mm();
  s_tr[s_tr_i].gz = (int16_t)imu_get_gyro_z();
  s_tr[s_tr_i].ir = (uint8_t)(r.raw & 0xFFu);
  s_tr_i = (uint16_t)((s_tr_i + 1u) % (uint16_t)TRACE_N);
  if (s_tr_n < (uint16_t)TRACE_N) s_tr_n++;
}

void telemetry_trace_reset(void)
{
  s_tr_i = 0u;
  s_tr_n = 0u;
}

void telemetry_trace_dump(void)
{
  char b[64];
  uint16_t i = (s_tr_n < (uint16_t)TRACE_N) ? 0u : s_tr_i;   /* 环满则从最旧的开始 */
  telemetry_msg("--- TRACE begin (oldest first) ---");
  for (uint16_t k = 0u; k < s_tr_n; k++)
  {
    char ir[LINE_CHANNELS + 1];
    for (uint8_t j = 0u; j < LINE_CHANNELS; j++)
      ir[j] = (s_tr[i].ir >> j) & 1u ? '1' : '0';
    ir[LINE_CHANNELS] = '\0';
    int n = snprintf(b, sizeof(b), "T%03u IR:%s GZ=%5d OD=%u\r\n",
                     (unsigned)k, ir, (int)s_tr[i].gz, (unsigned)s_tr[i].od);
    if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)b, (uint16_t)n, 100);
    i = (uint16_t)((i + 1u) % (uint16_t)TRACE_N);
  }
  telemetry_msg("--- TRACE end ---");
}

/* ★2026-09-24：默认遥测改成【短核心行】（≈67 字符 → 80 列终端不再折行）。
   理由：他反馈"输出又多又杂、看不懂"。细节全部挪到 `H` 指令（下面那个长行函数）。 */
void telemetry_report(void)
{
  line_reading_t r = line_read();
  char ir[LINE_CHANNELS + 1];
  for (uint8_t i = 0; i < LINE_CHANNELS; i++)
    ir[i] = (r.raw >> i) & 1u ? '1' : '0';
  ir[LINE_CHANNELS] = '\0';

  uint16_t k1m = line_1k_take(NULL, NULL, NULL);   /* 只留"100Hz 漏看数"当信号灯 */

  char buf[112];
  int n = snprintf(buf, sizeof(buf),
    /* ★2026-09-24 加 GW= 与 IG= —— 必须加，否则出圈判据没法验证：
       带 GW/IG 的完整长行要靠发 `H` 命令取，而本车【PC→车 RX 不通】，
       发不了命令 → 队友方案第 4 步"葫芦里 GW 应 1→2→0、直角弯一直是 0"
       根本无从检查。把这两个塞进默认短行，插上串口就能直接看。
       GW = 相切点计数（0..GOURD_TOTAL）：葫芦里应该 1→2→0 然后右转；
            直角弯必须一直是 0（这正是判据成败的验收线）。
       IG = 是否在葫芦圈里：只在 IG=1 时 GW 才累计，IG=0 会把 GW 强制清零。
       行长从 ~67 增到 ~78 字符，仍在 80 列内。 */
    "FW:%s ST=%d IR:%s RPM1=%d RPM2=%d OD=%ld GZ=%d GW=%d IG=%d K1=%d\r\n",
    FW_TAG, (int)car_fsm_state(), ir,
    speed_get_rpm(MOTOR_LEFT), speed_get_rpm(MOTOR_RIGHT),
    (long)odom_distance_mm(), (int)imu_get_gyro_z(),
    (int)line_follow_gourd_waves(), (int)line_follow_in_gourd(), (int)k1m);
  if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, 100);
}

/* ★完整长行（所有细节）—— 由 `H` 指令按需打印, 不再每拍刷屏。 */
void telemetry_print_full(void)
{
  /* ★2026-09-23 晚：speed_update() 不再在这里调用 —— 已挪到 app_loop 的控制拍里
     （10ms 一次）。原来只在这里算 → 速度闭环拿到的是 50ms 前的旧值。
     这里只读走最新值打印。 */
  line_reading_t r = line_read();

  char ir[LINE_CHANNELS + 1];
  for (uint8_t i = 0; i < LINE_CHANNELS; i++)
    ir[i] = (r.raw >> i) & 1u ? '1' : '0';
  ir[LINE_CHANNELS] = '\0';

  /* ★1kHz 采样统计（本窗口，读一次清一次）—— 量化"100Hz 漏掉了多少瞬间"
     K1 = missed/total（与主循环读数不同的 1kHz 样本 / 总样本）
     MX = 本窗口出现过的最宽图案（路数/位图）—— 一闪而过的宽图案会在这里现形 */
  uint16_t k1_total = 0u, k1_maxraw = 0u;
  uint8_t  k1_maxact = 0u;
  uint16_t k1_missed = line_1k_take(&k1_total, &k1_maxact, &k1_maxraw);
  char mx[LINE_CHANNELS + 1];
  for (uint8_t i = 0; i < LINE_CHANNELS; i++)
    mx[i] = (k1_maxraw >> i) & 1u ? '1' : '0';
  mx[LINE_CHANNELS] = '\0';

  char buf[288];
  int n = snprintf(buf, sizeof(buf),
    "FW:%s IR:%s RPM1=%d RPM2=%d KP=%d.%d SP=%d ST=%d DT=%d OD=%ld PATH=%ld ODE=%ld OS=%d GZ=%d GW=%d SB=%d G7=%d GX=%d GR=%d CX=%d BR=%d GE=%d GD=%d IG=%d EH=%d K1=%d/%d MX=%d/%s\r\n",
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
    (int)line_follow_edge_hold_cnt(),       /* ★EH: 最边两路"掉路容忍"补过几次（累计）
                                                涨得多 = 相切处最边传感器熄灭确实在发生 */
    (int)k1_missed, (int)k1_total,          /* ★K1=missed/total：1kHz 采样里"与主循环读数
                                                不同"的次数 / 总次数 = 100Hz 漏掉的瞬间占比。
                                                占比高 = 图案变化比 100Hz 采样还快 —— 出口那种
                                                "一闪而过"就是被这里量出来的 */
    (int)k1_maxact, mx);                    /* ★MX=路数/位图：本窗口出现过的最宽图案。
                                                若某次 MX 显示"6/00111111"而同一行的 IR: 从没
                                                出现过它 → 说明 100Hz 确实漏掉了一个宽图案 */
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
  /* ★2026-09-24 新增【测试态】`TEST <秒>`（他要求）：
     把车放在葫芦入口前 → 发 `TEST 1` → 车只循迹 1 秒 → **自动回放这 1 秒的 100Hz 轨迹**。
     用途：**用实测数据来定"葫芦入口的标志"**（他明说"我自己说的不准"）。
     ⚠️ 只在待机/到站时允许（不劫持正在跑的正式一趟）。 */
  if ((cmd_line[0] == 'T') && (cmd_line[1] == 'E'))
  {
    int sec = atoi(cmd_line + 4);
    if (sec < 1) sec = 1;
    if (sec > 5) sec = 5;
    {
      car_state_t st = car_fsm_state();
      if ((st != CAR_RUN) && (st != CAR_COUNTDOWN))   /* 急停也允许: 台架上方便 */
      {
        test_mode = 1;                        /* 让状态机让位, 由 app_loop 的测试态驱动 */
        telemetry_trace_reset();              /* 轨迹清零 → 回放就只有这 n 秒 */
        test_run_ms = (uint16_t)(sec * 1000);
        HAL_UART_Transmit(&huart1, (uint8_t*)"OK TEST run\r\n", 14, 100);
      }
      else
      {
        HAL_UART_Transmit(&huart1, (uint8_t*)"TEST only in IDLE/STOPPED\r\n", 27, 100);
      }
    }
    return;
  }

  if (strchr("PSDTMLBRVHXOZQ", c) == NULL) return;
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
  else if (c == 'Q')                        /* ★轨迹回放（黑匣子）—— 请停下再发 */
  {
    telemetry_trace_dump();
    n = 0;                                  /* 内容已经打完, 不再另回一行 */
  }
  else if (c == 'H')                        /* 诊断: 完整长行 + 8 路原始位图 + 按键 + 编码器 + IMU */
  {
    telemetry_print_full();                 /* ★2026-09-24: 细节全在这一行（默认不再每拍刷屏） */
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
