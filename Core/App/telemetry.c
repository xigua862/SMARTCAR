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

/* ============================================================================
 * ★★★ 2026-09-26 晚【串口改成"环形缓冲 + 中断发送" —— 修"经常卡着发不出去"】★★★
 *
 * 【病根】原来全部用 `HAL_UART_Transmit(..., 100)` —— 它是【阻塞】的，
 *   要等整行发完才返回，超时才放弃。实测两处致命：
 *     ① 黑匣子回放：600 行 × 每行阻塞 ≈4ms（超时上限 100ms/行）
 *        ⇒ **回放期间控制循环被冻住约 2.6 秒**（车速、循迹、丢线全停摆）
 *     ② 正常遥测：每 50ms 发约 100 字符 = 阻塞 ≈8.7ms
 *        ⇒ 占掉控制周期的 17%，而且一旦 UART 慢一点就 [卡着发不出去]
 *   这解释了用户反复遇到的"串口经常卡着"和"长日志丢字符"。
 *
 * 【修法】所有输出先写进环形缓冲，再由 TXE 中断在后台逐字节送出：
 *     · 控制循环只做一次 memcpy（微秒级）→ **永不等待串口**
 *     · 回放 600 行只占主循环约 20ms（原 2600ms，改善 130 倍）
 *     · 缓冲满时【丢弃】而不是阻塞 —— 丢字符比冻住控制循环好得多
 *
 * 【为什么不用 DMA】USART1 的 DMA 通道要动 CubeMX 配置（本项目有"CubeMX 重新
 *   生成破坏工程"的前车之鉴），而 TXE 中断只改这一个文件的寄存器操作，风险小得多。
 *   CPU 开销：115200 下每字符一次中断 ≈ 87µs 一次，可忽略。
 * ==========================================================================*/
#define TXBUF_SIZE   8192u              /* 要能装下黑匣子一次回放的一批行
                                           ★2026-09-26 晚 4096 → 8192：
                                             实测一次回放 ≈27KB，4KB 只能装 15% →
                                             输出被截断成 "T072 PWT073 PWT074 PW0T075…"
                                             （只有前 72 行完整）＝黑匣子等于没用。
                                             配合【回放抽稀 1/3】后一次约 9KB，8192 装得下。 */
static volatile uint8_t  txbuf[TXBUF_SIZE];
static volatile uint16_t tx_head = 0u;  /* 写指针（主循环）*/
static volatile uint16_t tx_tail = 0u;  /* 读指针（中断）*/
static volatile uint32_t tx_drop = 0u;  /* 因缓冲满而丢弃的字节数（只观测）*/

/* 送一段数据（非阻塞）。满了就丢，绝不等待。 */
static void tx_send(const uint8_t *p, uint16_t n)
{
  for (uint16_t i = 0; i < n; i++)
  {
    uint16_t nx = (uint16_t)((tx_head + 1u) % TXBUF_SIZE);
    if (nx == tx_tail) { tx_drop++; continue; }   /* 满 → 丢这一个字节 */
    txbuf[tx_head] = p[i];
    tx_head = nx;
  }
  USART1->CR1 |= USART_CR1_TXEIE;                 /* 催促中断去发 */
}

/* 送一个以 0 结尾的字符串（非阻塞） */
static void tx_send_str(const char *s)
{
  uint16_t n = 0;
  while (s[n] != '\0' && n < 250u) n++;
  tx_send((const uint8_t*)s, n);
}

/* ★2026-09-22：给状态机用的"事件打印"（急停原因/解除等），自带换行 */
void telemetry_msg(const char *msg)
{
  tx_send_str(msg);
  tx_send((const uint8_t*)"\r\n", 2u);
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
    tx_send((const uint8_t*)buf, (uint16_t)n);
}

/* 打印当前状态: 版本标签 + IR 8 路位图 + RPM + KP/KD/SP + 状态机状态 */
/* ★轨迹黑匣子（2026-09-24）：控制拍里记录最近 TRACE_N 拍的 IR/GZ/OD，`Q` 回放。
   为什么要它：跑车时 USB 线会拖，而"过直角弯/出口那几秒"正是最需要看的数据。
   用法：跑完停下 → 串口发 `Q` → 按时间顺序回放 600 拍（6 秒 @100Hz）。
   ⚠️ 回放是阻塞发送（600 行 ≈ 1.6 秒），所以请【停下来再发】。 */
/* ★2026-09-26 加 oL/oR（左右轮各自里程）：
   排查"弯道还抽"时发现两轮严重不对称（左轮均值 117、右轮 155 RPM，41% 的样本差 >100），
   但原来的合成里程 OD 看不出单轮 —— 无法区分
     ① 左轮机械偏弱（真的转得慢）
     ② 转向时内侧轮本来就该慢（正常差速）
   加了单轮里程就能一刀切开。 */
/* ★2026-09-26 晚 600 → 220：加了决策量后每个点 13→19 字节，
   600 点要 11.4KB + 串口缓冲 8KB → 超出 STM32F103C8 的 20KB（链接报 L6406E）。
   既然回放已抽稀 3 倍（TRACE_DECIM=3），点数同比例减少【覆盖时长完全不变】：
     220 点 × 3 拍 × 10ms = 6.6 秒（原来 600 点 × 1 拍 = 6 秒，还更长一点）。
   占用：220 × 19 = 4.2KB（比原来 600 × 13 = 7.8KB 还省）。 */
#define TRACE_N 130
/* ★2026-09-26 晚【回放抽稀】：每 TRACE_DECIM 拍才记一次。
   为什么：600 拍全量回放 ≈27KB，而环形缓冲 8KB、115200 下也要 2.3 秒才送完 ——
   实测 4KB 缓冲时只有前 72 行完整，后面全被截断（黑匣子等于没用）。
   抽稀 3 倍 → 200 点、约 9KB → 装得下且有富余。
   ★代价：时间分辨率从 100Hz 降到 33Hz。对本病不影响 ——
     进葫芦圈后的振荡周期约 80ms，33Hz（30ms/点）仍能画出 3 个点/周期。 */
#define TRACE_DECIM  3
typedef struct {
  uint16_t od; uint16_t oL; uint16_t oR;
  int16_t  gz; int16_t pwl; int16_t pwr;
  uint8_t  ir;
  /* ★2026-09-26 晚【加决策量】：光有结果（IR/RPM/PWM）判断不了
     "跟不住线" 还是 "在丢线原地旋转" —— 这两种的修法完全不同。 */
  int8_t   de;    /* 交给 PD 的误差 */
  int16_t  dcor;  /* 算出的差速修正 */
  int16_t  dsp;   /* 速度基准 */
  uint16_t ddcy;  /* 丢线计数（> LOST_SPIN_DELAY = 正在原地旋转）*/
  uint8_t  dflg;  /* bit0 丢线 / bit1 在葫芦圈 / bit2 相切点直行 */
} trace_t;
static trace_t  s_tr[TRACE_N];
static uint16_t s_tr_i = 0u;   /* 写指针 */
static uint16_t s_tr_n = 0u;   /* 已写条数 */
static uint8_t  s_tr_dec = 0u; /* 抽稀计数 */
static uint8_t  s_dump_quiet = 0u; /* ★回放期间暂停周期遥测（>0 则跳过打印）*/
static uint16_t s_dump_cool  = 0u; /* ★回放冷却：刚回放过就不再回放（防两次连发）*/

void telemetry_trace_tick(void)
{
  /* ★回放冷却递减（放在最前面：先于任何提前 return）*/
  if (s_dump_cool > 0u) s_dump_cool--;

  /* ★2026-09-26 晚【抽稀】：每 TRACE_DECIM 拍才记一次（见 TRACE_DECIM 的说明）。
     必须在最前面做，这样"葫芦圈外不记"和"抽稀"的顺序都不影响计数节奏。 */
  s_tr_dec = (uint8_t)(s_tr_dec + 1u);
  if (s_tr_dec < (uint8_t)TRACE_DECIM) return;
  s_tr_dec = 0u;

  /* ★★★ 2026-09-26 晚【只在葫芦圈里记录】★★★
     为什么：600 拍 = 6 秒。而车卡住往往发生在整趟的【中段】——
     等它停下来再回放时，"最后 6 秒"早就不是出问题的那段了。
     实测：A150 那趟卡在 OD ~2500，而全程跑到 7800 → 靠"停车后回放最后 6 秒"
     根本看不到卡住那一刻。
     ⇒ 改成【只在 IG=1（葫芦圈内）时记录】：
        600 拍全部是葫芦里的画面 = 6 秒的葫芦行为，正好覆盖卡住的周期。
     ★代价：葫芦外面的数据不进黑匣子（本阶段不需要）。
     要恢复全程记录：把下面这个 if 去掉即可。 */
  if (!line_follow_in_gourd()) return;

  line_reading_t r = line_read();
  s_tr[s_tr_i].od = (uint16_t)odom_distance_mm();
  s_tr[s_tr_i].gz = (int16_t)imu_get_gyro_z();
  s_tr[s_tr_i].ir = (uint8_t)(r.raw & 0xFFu);
  /* ★2026-09-26 【关键补充】把"控制器实际输出的 PWM"也记进轨迹。
     为什么必须加：诊断"一顿一顿往前窜"时，光有位移看不出是
       ① 控制器在乱输出（PWM 大幅摆动） 还是
       ② 控制器输出平稳但机械/电机在抖
     而本车【PC→车 RX 不通】（见 telemetry_report 的注释），发不了 `H` 去取长行，
     默认短行又太挤塞不下 PWM —— 轨迹是唯一还能用的观测通道，
     所以把 PWM 记在这里。600 拍 = 6 秒，足够看清振荡周期。 */
  line_follow_last_pwm(&s_tr[s_tr_i].pwl, &s_tr[s_tr_i].pwr);
  /* ★2026-09-26 单轮里程：区分"左轮机械偏弱"vs"转向正常差速" */
  s_tr[s_tr_i].oL = (uint16_t)odom_left_mm();
  s_tr[s_tr_i].oR = (uint16_t)odom_right_mm();
  /* ★决策量（见 trace_t 里的说明）：由 line_follow 每拍写入 */
  line_follow_dbg_get(&s_tr[s_tr_i].de, &s_tr[s_tr_i].dcor, &s_tr[s_tr_i].dsp,
                      &s_tr[s_tr_i].ddcy, &s_tr[s_tr_i].dflg);
  s_tr_i = (uint16_t)((s_tr_i + 1u) % (uint16_t)TRACE_N);
  if (s_tr_n < (uint16_t)TRACE_N) s_tr_n++;
}

void telemetry_trace_reset(void)
{
  s_tr_i = 0u;
  s_tr_n = 0u;
  s_tr_dec = 0u;
}

void telemetry_trace_dump(void)
{
  /* ★★2026-09-26 晚【回放防重入】★★
     实车证据（FW:0926-F20）：一次停车触发了两遍回放 ——
       "按停" 一条路径 + 停车 3 秒后的自动回放，两次共 15.6KB 挤进 8KB 缓冲，
       结果第二次把第一次的尾巴冲掉，T112 之后全是乱码（黑匣子又白记了）。
     修法：回放设 5 秒冷却；冷却期内再调用直接返回。 */
  if (s_dump_cool > 0u) return;
  s_dump_cool = 500u;              /* 500 拍 × 10ms = 5 秒 */

  char b[96];
  uint16_t i = (s_tr_n < (uint16_t)TRACE_N) ? 0u : s_tr_i;   /* 环满则从最旧的开始 */
  /* ★2026-09-26 晚：回放期间【暂停周期遥测】，见 s_dump_quiet 的说明。 */
  s_dump_quiet = 60u;
  telemetry_msg("--- TRACE begin (oldest first) ---");
  /* ★2026-09-26 晚【改格式】：加了决策量，并【删掉 L=/R=】腾地方。
     为什么删单轮里程：它只在"验证左右轮机械差异"时有用（那件事已做完，
     结论是 WHEEL_GAIN_L=1.11）；而现在最缺的是"控制器每一拍在做什么决定"。
     ★新字段（见 trace_t）：
        e   = 交给 PD 的误差      cor = 算出的差速修正
        sp  = 速度基准            dc  = 丢线计数（>LOST_SPIN_DELAY = 正在原地旋转）
        f   = 标志位（bit0 丢线 / bit1 在葫芦圈 / bit2 相切点直行）
     ⇒ **dc 是本次最关键的**：它能一眼区分"跟不住线"和"在丢线原地旋转"。
     行长约 60 字符，比原格式还短，不易被截断。 */
  for (uint16_t k = 0u; k < s_tr_n; k++)
  {
    char ir[LINE_CHANNELS + 1];
    for (uint8_t j = 0u; j < LINE_CHANNELS; j++)
      ir[j] = (s_tr[i].ir >> j) & 1u ? '1' : '0';
    ir[LINE_CHANNELS] = '\0';
    int n = snprintf(b, sizeof(b),
                     "T%03u PWM=%3d/%3d IR:%s e=%3d cor=%3d sp=%2d dc=%3u f=%u GZ=%5d OD=%u\r\n",
                     (unsigned)k, (int)s_tr[i].pwl, (int)s_tr[i].pwr,
                     ir, (int)s_tr[i].de, (int)s_tr[i].dcor, (int)s_tr[i].dsp,
                     (unsigned)s_tr[i].ddcy, (unsigned)s_tr[i].dflg,
                     (int)s_tr[i].gz, (unsigned)s_tr[i].od);
    if (n > 0) tx_send((const uint8_t*)b, (uint16_t)n);
    i = (uint16_t)((i + 1u) % (uint16_t)TRACE_N);
  }
  telemetry_msg("--- TRACE end ---");
}

/* ★2026-09-24：默认遥测改成【短核心行】（≈67 字符 → 80 列终端不再折行）。
   理由：他反馈"输出又多又杂、看不懂"。细节全部挪到 `H` 指令（下面那个长行函数）。 */
void telemetry_report(void)
{
  /* ★回放静默期：跳过周期遥测，把串口带宽留给黑匣子回放 */
  if (s_dump_quiet > 0u) { s_dump_quiet--; return; }

  line_reading_t r = line_read();
  char ir[LINE_CHANNELS + 1];
  for (uint8_t i = 0; i < LINE_CHANNELS; i++)
    ir[i] = (r.raw >> i) & 1u ? '1' : '0';
  ir[LINE_CHANNELS] = '\0';

  /* ★2026-09-24 晚：改读 line_follow 的缓存，不再直接 line_1k_take ——
     那个会清零窗口，而相切点识别（100Hz）已经取走了 maxact/maxraw，
     遥测只有 20Hz，直接取几乎总是拿到空窗口。 */
  uint16_t k1m = 0u;
  line_follow_1k_stats(&k1m, NULL, NULL, NULL);   /* 只留"100Hz 漏看数"当信号灯 */
  /* ★2026-09-26：K1 已从默认短行移除（换成 SAT/SATN），但这次调用【保留】——
     它有副作用（取走 1kHz 采样窗口），去掉会改变相切点识别的行为。
     变量本身不再打印，显式 (void) 掉以保持 0 警告。要恢复 K1= 时直接用它。 */
  (void)k1m;

  char buf[112];
  int n = snprintf(buf, sizeof(buf),
    /* ★2026-09-24 加 GW= 与 IG= —— 必须加，否则出圈判据没法验证：
       带 GW/IG 的完整长行要靠发 `H` 命令取，而本车【PC→车 RX 不通】，
       发不了命令 → 队友方案第 4 步"葫芦里 GW 应 1→2→0、直角弯一直是 0"
       根本无从检查。把这两个塞进默认短行，插上串口就能直接看。
       GW = 相切点计数（0..GOURD_TOTAL）：葫芦里应该 1→2→0 然后右转；
            直角弯必须一直是 0（这正是判据成败的验收线）。
       IG = 是否在葫芦圈里：只在 IG=1 时 GW 才累计，IG=0 会把 GW 强制清零。
       行长从 ~67 增到 ~78 字符，仍在 80 列内。
       ★2026-09-26 把 K1（1kHz 漏采统计）换成 SAT/SATN（陀螺削顶）：
         K1 是"1kHz 采样漏了多少"的统计，当前诊断用不到；
         而 SA 是"陀螺有没有削顶"——葫芦圈问题的关键未知数（见 imu.h 的长注释）。
         SAT= 本拍是否削顶(0/1)，SATN= 开机以来累计削顶拍数。
         ★要查相切点/出口识别时，把 K1= 换回来即可（line_follow_1k_stats）。 */
    /* ★★★ 2026-09-26 晚【终于找到"加了字段却看不到"的真正原因】★★★
       前两版把 AR=/AN= 加在 `telemetry_print_full()`（长行）里 ——
       而那个函数【只有发 `H` 指令才会打印】，本车 PC→车 RX 不通 ⇒ **永远不会执行**。
       真正每 50ms 打印的是本函数（默认短行）。
       ⇒ 教训（比字段本身重要）：**加诊断量之前，先确认它所在的打印路径真的会被执行**。
         "编译通过 + 烧录成功" 完全不能保证"这行代码会跑"。
       本行原来约 105 字符，早就超过该串口能稳定送出的长度（实测尾部 SAT= 常被吃掉），
       所以这里按 ≤95 字符重排，并保留本次诊断必需的：
         IR / RPM / OD / GZ / AR / AN / IG
       删掉：ST=（跑车中恒为 2）、GW=/SAT=/SATN=（本次不查，要查时从 git 历史恢复）。
       AR/AN 定义见 line_follow.h；AR 阈值 = GOURD_ARC_FLIP_MDEG。 */
    "FW:%s IR:%s RPM %d/%d OD %ld GZ %d IG %d AR %ld AN %u\r\n",
    FW_TAG, ir,
    speed_get_rpm(MOTOR_LEFT), speed_get_rpm(MOTOR_RIGHT),
    (long)odom_distance_mm(), (int)imu_get_gyro_z(),
    (int)line_follow_in_gourd(),
    (long)line_follow_arc_mdeg(), (unsigned)line_follow_arc_trigs());
  if (n > 0) tx_send((const uint8_t*)buf, (uint16_t)n);
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

  /* ★1kHz 采样统计（本窗口）—— 量化"100Hz 漏掉了多少瞬间"
     K1 = missed/total（与主循环读数不同的 1kHz 样本 / 总样本）
     MX = 本窗口出现过的最宽图案（路数/位图）—— 一闪而过的宽图案会在这里现形
     ★2026-09-24 晚：改读 line_follow 的缓存，不再直接 line_1k_take
       （那个会清零窗口，而相切点识别 100Hz 已经取走了 maxact/maxraw）。 */
  uint16_t k1_total = 0u, k1_maxraw = 0u;
  uint8_t  k1_maxact = 0u;
  uint16_t k1_missed = 0u;
  line_follow_1k_stats(&k1_missed, &k1_total, &k1_maxact, &k1_maxraw);
  /* ★2026-09-26 晚：这四个统计量【本次不打印】（要腾出行长度给 AR=/AN=），
     但 line_follow_1k_stats() 的调用【必须保留】—— 它有清零窗口的副作用。
     输出变量显式标注为"故意不用"，避免编译告警（原来那个把 k1_maxraw 展开成
     点阵字符串的 mx[] 已经删除，因为没人再读它）。 */
  (void)k1_missed; (void)k1_total; (void)k1_maxact; (void)k1_maxraw;

  /* ★2026-09-26 诊断：控制器【实际输出】的 PWM（左右轮）。
     排查"一抽一抽"时，只有 RPM 无法区分"电机不响应"和"控制器没出力"；
     有了 PWM= 就能直接分岔（判据见 line_follow_last_pwm 的说明）。
     ⚠️ 本次用 PWM= 顶掉了原来的 K1=missed/total（1kHz 漏采统计）——
        直线抖动诊断用不到它；要查相切点/出口识别时再换回来。 */
  int16_t pwm_l = 0, pwm_r = 0;
  line_follow_last_pwm(&pwm_l, &pwm_r);

  char buf[288];
  /* ★★★ 2026-09-26 晚【行太长 → 诊断字段收不到：同一个错犯了两次】★★★
     上一版把 AR=/AN= 加在【行尾】，实车日志里它们从未出现过 ——
     用户贴回的每行都在 `IG 0` 附近就断了，连本来靠后的 SAT= 也常丢。
     整行算下来约 125 字符，早就超过这套串口能稳定送出的长度。
     ⇒ 教训：加字段前必须先【算清整行长度】，不能"删几个旧的"就以为腾出了地方。
     这一版按 ≤80 字符重排，并把新诊断量【挪到前面】——再被截断也先保住它们。

     保留（本次诊断必需）：IR / RPM / PWM / OD / GZ / IG / AR / AN
     删掉（本次用不到；要查时从 git 历史取回）：
       ST=、DT=、GW=、SB=、G7=、GX=、GR=、CX=、BR=、GE=、GD=、EH=、MX=
     AR/AN 定义见 line_follow.h；AR 阈值 = GOURD_ARC_FLIP_MDEG = 320000。 */
  int n = snprintf(buf, sizeof(buf),
    "FW:%s IR:%s RPM %d/%d PWM %d/%d OD %ld GZ %d IG %d AR %ld AN %u\r\n",
    FW_TAG, ir,
    speed_get_rpm(MOTOR_LEFT), speed_get_rpm(MOTOR_RIGHT),
    (int)pwm_l, (int)pwm_r,          /* ★实际下发的 PWM：和 RPM 对照就知道是谁的问题 */
    (long)odom_distance_mm(),         /* ★OD: 净位移(mm), 从起步算起, 前进为正 */
    (int)imu_get_gyro_z(),            /* ★GZ: 偏航角速度(dps), 正=左转 */
    (int)line_follow_in_gourd(),      /* ★IG: 1=当前在葫芦圈里 */
    (long)line_follow_arc_mdeg(),     /* ★AR: 绕圈脱困的同号偏航累计（毫度）*/
    (unsigned)line_follow_arc_trigs() /* ★AN: 脱困触发次数（只增不减）。
                                         恒为 0 = 机制从未跑起来（"开了等于没开"的铁证）*/
    );
  if (n > 0)
    tx_send((const uint8_t*)buf, (uint16_t)n);
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
        tx_send((const uint8_t*)"OK TEST run\r\n", 14u);
      }
      else
      {
        tx_send((const uint8_t*)"TEST only in IDLE/STOPPED\r\n", 27u);
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
  if (n > 0) tx_send((const uint8_t*)buf, (uint16_t)n);
}

/* USART1 中断入口(寄存器级, 收 + 发都在这里)。
   ★2026-09-26 晚：加了【发送】分支 —— 配合上面的环形缓冲做非阻塞输出。
     为什么用 TXE（发送数据寄存器空）而不是 TC（发送完成）：
       TXE 在 DR 被搬进移位寄存器时置位 → 可以立刻填下一个字节；
       TC 要等整个字节移完（含停止位）→ 会白白空等一个字节的时间。
       115200 下每字节 ≈87µs，用 TXE 能连续填满、不产生间隙。
     收部分保持不变：即使 ORE 溢出也只清标志继续收，不会像 HAL Receive_IT 那样卡死。 */
void USART1_IRQHandler(void)
{
  uint32_t sr = USART1->SR;

  /* ---- 发送：缓冲里还有就继续送 ---- */
  if ((sr & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))
  {
    if (tx_tail != tx_head)
    {
      USART1->DR = txbuf[tx_tail];
      tx_tail = (uint16_t)((tx_tail + 1u) % TXBUF_SIZE);
    }
    else
    {
      USART1->CR1 &= ~USART_CR1_TXEIE;   /* 发空 → 关 TXE 中断，别再进来 */
    }
  }

  /* ---- 接收 ---- */
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
