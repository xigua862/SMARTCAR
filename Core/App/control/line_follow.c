#include "line_follow.h"
#include "app.h"
#include "app_config.h"
#include "drivers/motor.h"
#include "drivers/line_sensor.h"
#include "control/filter.h"
#include "telemetry.h"        /* ★事件打印用 telemetry_msg("出葫芦弯道" 等) */
#include <stdio.h>            /* snprintf: 拼"十字/分支口"事件行 */
#if USE_IMU
#include "drivers/imu.h"
#endif
#if USE_SPEED_LOOP
#include "control/pid.h"
#endif
/* ★speed.h 移出条件编译：即使 USE_SPEED_LOOP=0（闭环关掉），
   "进圈判据"也要用 speed_get_rpm() 判断"车确实在跑"（见 GOURD_IG_MIN_RPM）。 */
#include "drivers/speed.h"
#if USE_IMU
#include "drivers/imu.h"       /* ★出口右转用积分航向角判"转够 90° 没有" */
#endif
#if USE_GOURD_EXIT
#include "drivers/encoder.h"   /* ★出口右转的距离兜底 */
#include "drivers/odom.h"      /* ★出口判据③：按里程强制右转（进圈起算的净位移） */
#endif

/* 丢线/上次误差状态 */
static int8_t   last_error  = 0;
static int8_t   last_e      = 0;   /* 上一次误差(PD 微分用) */
static uint16_t lost_cycles = 0;   /* ★uint8 会在 256 拍回绕, 把"停车防跑飞"抵消 */
static uint8_t  line_lost   = 0;

/* 宽图案判定状态（十字 / 圆出口 / 交叉） */
static uint8_t  wide_cnt    = 0;   /* 宽图案持续拍数 */
static uint8_t  on_cross    = 0;   /* 1 = 判定为十字（直行通过） */
static uint8_t  wide_long   = 0;   /* 1 = 宽图案持续过久 → 不是十字（圆出口/直角入口） */
static uint8_t  s_wo_hold   = 0;   /* ★相切点强制直行剩余拍数（WIDE_ONE_GO_STRAIGHT）*/
#if GOURD_ARC_FLIP
static int32_t  s_arc_mdeg  = 0;   /* ★同号偏航累计（毫度）—— 绕圈/相切点检测 */
static uint8_t  s_arc_push  = 0;   /* ★反向打舵剩余拍数 */
static int8_t   s_arc_dir   = 0;   /* ★反向打舵方向（±1）*/
#endif
/* ★2026-09-22 15:31 十字判据修正 + 事件计数/打印 */
static uint16_t cross_events = 0;  /* 一趟里"判定为十字"的次数（遥测 CX=） */
static uint16_t branch_events = 0; /* 一趟里"宽图案只贴一端 → 拒绝当十字"的次数（遥测 BR=） */
static uint8_t  one_end_prev = 0;  /* 上一拍是否"只贴一端的宽图案"（边沿，防刷屏） */
static uint8_t  det_prev     = 0;  /* 上一拍是否"落在十字时间窗"（边沿，给预算计数用） */
static uint8_t  wide_gap     = 0;  /* 宽图案已连续消失多少拍（★事件计数防抖，见 app_config） */
static uint8_t  wide_armed   = 1;  /* 本段宽图案"可否计数"：★在段首那一拍判定并锁存整段 */

/* ★直线提速（2026-09-22）—— ★2026-09-24 关掉（USE_STRAIGHT_BOOST=0）：
   关掉时 straight_cycles 只在 init 里被写、没人读 → 会报 "set but never used"。
   所以声明和复位都跟着开关走（#if 范围必须一致，这是本项目踩过的老坑）。 */
#if USE_STRAIGHT_BOOST
static uint16_t straight_cycles = 0;   /* 连续"直线"拍数 */
#endif
static uint8_t  boost_active    = 0;   /* 1 = 已进入提速档（accessor 会读, 所以不受开关影响） */

/* ★2026-09-24 晚：本窗口的 1kHz 采样统计（遥测 K1= / MX=）。
   ★为什么放在这里而不是让遥测直接调 line_1k_take：
     line_1k_take 会【清零】窗口统计（missed/total/maxact/maxraw 全清），
     相切点识别那边已经取走了 maxact/maxraw —— 如果遥测再取一次，
     两边就各拿到半个窗口，谁都看不全（而且相切点识别是 100Hz、遥测只有 20Hz，
     晚取的那一方几乎总是拿到空窗口）。
     所以统一"由相切点识别处取走、缓存到这里"，遥测读缓存。 */
static uint16_t s_1k_missed = 0;
/* ★2026-09-24 晚：IG=0 的"离开确认"计数（GW 清零用）。
   见 IG 块末尾的说明 —— 实车日志证明 IG 会乱跳，立刻清零会把 GW 抹掉。 */
static uint16_t s_ig_leave = 0;
/* ★2026-09-24 晚：上一个相切点被记下时的里程读数。
   用来做"两个相切点之间至少要隔 GOURD_WAVE_MIN_MM 毫米"的门槛 ——
   实车日志里误报是成簇的（两个"相切点"只隔 47mm），会把 GW 提前推满。 */
static int32_t  s_wave_odom = 0;
static uint16_t s_1k_total  = 0;
static uint8_t  s_1k_maxact = 0;
static uint16_t s_1k_maxraw = 0;

/* ★葫芦弯相切点状态机（2026-09-22 他提的方案） */
static uint8_t  gourd_latch  = 0;      /* 分离锁存：一段分离只记 1 次 */
static uint8_t  gourd_waves  = 0;      /* 已识别的相切点个数（到 3 清零 = 绕完一圈） */
#if USE_GOURD_SM
/* ★★★ 2026-09-24 新增：出圈触发事件（队友方案第 3 步）★★★
   GW 归 0（= 第 3 个相切点）时置 1，被出圈触发块消费后清 0。
   ★为什么用独立事件位、而不是直接判 `gourd_waves == 0`：
     发车时 GW 本来就是 0 —— 直接判的话车一启动就会立刻右转。 */
static uint8_t  gourd_wrap_evt = 0;
#endif
/* ★2026-09-24 相切点新判据(三条)的状态：GZ 符号 / 翻号窗口 / 候选窗口+候选是否通过 */
static int8_t   gz_sign   = 0;
static uint8_t  flip_win  = 0;
static uint8_t  cand_win  = 0;
static uint8_t  cand_ok   = 0;
#if GOURD_USE_FLIP
static uint8_t  gourd_flip   = 0;      /* "翻转修正"剩余拍数（仅 GOURD_USE_FLIP=1 时用） */
static int8_t   gourd_dir    = 1;      /* 进相切点前的误差方向 */
#endif

/* ★弯道计数 → 强制右转（2026-09-22 他提的方案）
   ★这几个变量【无条件声明】（不放进 #if）—— 2026-09-22 那次"宏开关漏罩 init"的编译事故
   就是 #if 范围不一致造成的，这里一律全声明、只在真正使用的地方 #if。 */
static uint16_t g7_count   = 0;   /* 最右一路(bit7)累计触发次数（上升沿计数） */
static uint8_t  g7_prev    = 0;   /* 上一拍 bit7 电平（边沿检测用） */
static uint8_t  g7_lock    = 0;   /* 计数锁：>0 时不许再计（防抖 / 防一次点亮算多次） */
static uint8_t  g7_waitmsg = 0;   /* "数够但位置不对"只打印一次 */
static uint8_t  g7_flag    = 0;   /* 1 = 已触发过"出葫芦弯道"（锁存，给遥测显示） */
static uint8_t  g7_turn    = 0;   /* 硬转剩余拍数（>0 = 正在硬转，期间忽略传感器） */

#if USE_D_FILTER
static lpf_t d_lpf;
#endif

#if USE_SPEED_LOOP
static pid_t spd_pid[2];   /* 左右轮速度环(编码器闭环) */
static float  tgt_rpm[2] = {0, 0};
#endif

/* ===========================================================================
 * ★★★ 葫芦圈【出口】检测（2026-09-23 用户实测特征）
 *
 *  特征：**只有最左边 GOURD_EXIT_LEFT_N 路亮**（默认 2 路 → 位图 `00000011`）
 *        → 直接右转就能出葫芦圈。
 *
 *  为什么它的方向是对的（几何自洽）：
 *     误差 = (-7)+(-5) = -12 → 强烈偏左 → 本来就该猛右转；
 *     而且**最右那一路(L8)是灭的** → 右侧完全没有线。
 *
 *  ⚠️ 已知冲突：右直角弯【入口】也是这个图案。
 *     动作上它俩都该右转（所以不会走错方向），区别只在"该不该在这儿转"。
 *     `GOURD_EXIT_GATE_WAVES` 打开后要求"已数到 >=N 个相切点"，即可区分。
 * ===========================================================================*/
/* ★最边两路掉路容忍：统计补过多少次。
   ⚠️ 它属于 USE_EDGE_HOLD，【不能】放在葫芦圈那块的 #if 里面 ——
     否则一关葫芦圈开关它就"未定义"（2026-09-23 又踩了一次这个坑）。 */
static uint16_t edge_hold_cnt = 0;
#if (EDGE_HOLD_CONFIRM > 0)
static uint8_t  ge_dwell_l = 0;
static uint8_t  ge_dwell_r = 0;
static uint16_t edge_rej_cnt = 0;   /* 因"没站稳"被抹掉的次数 */
#endif

#if USE_GOURD_EXIT
/* ★这两个只服务【图案判据①】，所以跟着 GOURD_EXIT_USE_PATTERN 走。
   不罩起来的话，关掉图案判据时 ARMCC 会报
   `#550-D: variable was set but never used`（构建返回 1 而不是 0）。 */
#if GOURD_EXIT_USE_PATTERN
static uint8_t  ge_exit_cnt  = 0;   /* 窗口内累计命中出口特征的拍数（不再要求连续！） */
static uint16_t ge_win       = 0;   /* 滑动窗口剩余拍数（>0 表示还在累加） */
#endif
static uint16_t ge_cool      = 0;   /* 起转后的冷却拍数（靠它防同一次出口重复起转） */
static uint16_t ge_events    = 0;   /* 触发过几次（遥测 GE=，排查误触发用） */

/* ★出口右转的闭环状态（三级判据：角度 / 距离 / 拍数） */
static uint8_t  ge_turning     = 0;   /* 1 = 正在闭眼右转 */
static uint16_t ge_settle      = 0;   /* ★2026-09-23：转完之后的"停一停"剩余拍数。
                                         转动是闭眼给固定差速做的，停那一刻车还在转；
                                         立刻交回 PD 会在车没停稳时就开始修正 → 更容易转过头。
                                         所以转完先两轮停住 GOURD_EXIT_SETTLE_FRAMES 拍。 */
static uint32_t ge_turn_dist   = 0;   /* 本次转动已走的编码器计数 */
static uint16_t ge_turn_frames = 0;   /* 本次转动已转的拍数 */
static uint32_t ge_moved       = 0;   /* 本拍两轮合计前进的计数 */
static int32_t  ge_encL        = 0;   /* 上一拍编码器原始值 */
static int32_t  ge_encR        = 0;
static float    ge_turn_deg    = 0.0f;/* 本次转动当前的航向角（负=右转） */
static float    ge_last_deg    = 0.0f;/* 上次转动【结束时】的角度（标定用，事件行打印） */
static int32_t  ge_odom_entry  = 0;   /* ★进圈那一刻的里程读数(odom_distance_mm)，
                                         用它把"进圈起算的净位移"算成
                                         odom_distance_mm() - ge_odom_entry */

/* 出口特征判定：亮的那些路【恰好就是】最左边 N 路，一路不多一路不少
   ★2026-09-23：用 GOURD_EXIT_USE_PATTERN 罩起来 ——
     关掉图案判据时这个函数就没人用了, 不罩会出现
     ARMCC 警告 #177-D "declared but never referenced"（构建返回 1 而不是 0）。 */
#if GOURD_EXIT_USE_PATTERN
static uint8_t ge_is_exit_pattern(uint16_t raw)
{
  uint16_t mask = (uint16_t)((1u << (uint8_t)GOURD_EXIT_LEFT_N) - 1u);   /* 最左 N 路的掩码 */
  return (uint8_t)((raw == mask) ? 1u : 0u);
}
#endif

/* ★★ 进圈印记（2026-09-23）：只有"确实进过葫芦圈"才允许出圈转向。
 *
 *  为什么必须要它：出圈特征"只闪一下"→ 只能把确认次数放宽到 1 次 →
 *  误触发风险上升。而右直角弯入口的图案【也是】"只有最左两路亮"。
 *  所以加一道闸门：没进过圈就不许出圈。
 *
 *  进圈判据用的是【已验证过的 4 个实车图案】（用户实拍给的）：
 *      11100110  10110010  11110010  11001110
 *  这 4 个的亮路数都是 4~5、都有"被亮通道夹住的灭区"、且最右那路是灭的。
 *  和直角弯/十字/直道都不冲突（详见 _work/exit_check.js 的穷举核对）。
 *
 *  做法：只要某一拍命中这 4 个图案之一 → 打上"进过圈"印记。
 *        印记在两个"进圈特征再没出现"后的 GOURD_ENTRY_MARK_TTL 拍内保持，
 *        然后自动清除（防止一次进圈后整圈都放开）。
 *        ★注意：这只是【放宽的近似】——它不知道车有没有真的转出去，
 *          但对"该不该允许出圈转向"这个是非判断足够了。 */
static const uint16_t kGourdEntryPat[4] = { 0x0067u, 0x004Du, 0x004Fu, 0x0073u };
/* 说明：0x67=11100110  0x4D=10110010  0x4F=11110010  0x73=11001110（bit0=最左） */

static uint8_t  ge_in_gourd = 0;    /* 1 = 已打上"进过圈"印记 */
static uint16_t ge_mark_ttl = 0;    /* 印记剩余存活拍数（只当保险） */
static uint8_t  ge_fired    = 0;    /* ★1 = 本次进圈【已经】出圈转过一次了（一次性锁存） */
static uint16_t ge_plain_cnt = 0;   /* ★连续多少拍是"普通线"（用于判定离开葫芦圈） */
/* ★2026-09-25 CARD-003：IG 清零重构的状态（与 ge_in_gourd 同一个 #if 范围，同进同出） */
static uint8_t  ge_confirmed = 0;  /* 本次进圈后已数到过>=1个相切点(=葫芦坐实) */
static uint8_t  ge_done      = 0;  /* GW 已归零(相切点全部过完) */
static uint16_t ge_straight  = 0;  /* ge_done 后的连续普通线拍数 */
static uint16_t ge_stay      = 0;  /* IG=1 持续拍数(兜底用) */
#if USE_GOURD_SLOWDOWN
static uint16_t ge_frames   = 0;    /* ★进圈后经过的拍数（后半段减速用） */
#endif

/* ★★★ 角速度判据的滑动窗口（2026-09-23，实测数据驱动）★★★
   维护"最近 GOURD_IG_WINDOW 拍内 sum(|GZ|)"：
   走进一拍就加上它的 |GZ|、并减掉滑出窗口那一拍的 |GZ| → O(1) 更新。
   实测分离度：直线段窗口和最大 448，葫芦圈最小 12542（28 倍余量）。
   ★注意：GOURD_IG_* 这几个宏定义在 app_config.h 的【葫芦圈配置块】里 —
     那一块被 #if (USE_GOURD_EXIT || USE_GOURD_LOST_GUARD) 罩着。
     所以这里【不能】只判 USE_GOURD_IG_GYRO，否则关掉葫芦圈时宏不可见 → 编译错。
     （2026-09-23 踩过：24 个"未定义"。） */
#if (USE_GOURD_EXIT || USE_GOURD_LOST_GUARD) && USE_GOURD_IG_GYRO
static uint16_t ge_gz_ring[GOURD_IG_WINDOW];   /* 环形缓冲：每拍的 |GZ| */
static uint8_t  ge_gz_idx  = 0;
static uint8_t  ge_gz_fill = 0;                /* 已填了几拍（不足窗口时不算） */
static uint32_t ge_gz_sum  = 0;                /* 当前窗口内的 sum(|GZ|) */

static void ge_gz_push(uint16_t gz_abs)
{
  uint16_t old = ge_gz_ring[ge_gz_idx];
  ge_gz_ring[ge_gz_idx] = gz_abs;
  ge_gz_idx = (uint8_t)((ge_gz_idx + 1u) % (uint8_t)GOURD_IG_WINDOW);
  ge_gz_sum += (uint32_t)gz_abs;
  ge_gz_sum -= (uint32_t)old;                  /* 减掉被顶出去的那一拍 */
  if (ge_gz_fill < (uint8_t)GOURD_IG_WINDOW) ge_gz_fill++;
}
#define GE_GZ_READY 1
#else
#define GE_GZ_READY 0
#endif

/* ★最边两路的"驻留计数"（只在 EDGE_HOLD_CONFIRM>0 时用到；当前是 0 = 停用）
   （edge_hold_cnt 已经在上面第 91 行声明过了，这里不要重复声明） */
#if (EDGE_HOLD_CONFIRM > 0)
static uint8_t  ge_dwell_l = 0;
static uint8_t  ge_dwell_r = 0;
static uint16_t edge_rej_cnt = 0;   /* 因"没站稳"被抹掉的次数 */
#endif

/* ★"普通线"判据：连续一小片亮、没有暗缝。
   为什么用它判"离开葫芦圈"：
     光靠计时不行 —— TTL 太小会在圆圈之间被清掉（锁存失效），
     TTL 太大出了圈还清不掉。
     葫芦圈里传感器【一直看到有暗缝的分离图案】；出圈后就是普通的一条线。
   实现：亮的路数 1~4，且"亮的最左~最右之间没有暗缝"（span == 亮路数）。 */
static uint8_t ge_is_plain_line(uint16_t raw)
{
  uint8_t L = 0xFFu, R = 0u, cnt = 0u, i;
  for (i = 0; i < 8u; i++)
  {
    if (raw & (uint16_t)(1u << i)) { if (L == 0xFFu) L = i; R = i; cnt++; }
  }
  if ((L == 0xFFu) || (cnt == 0u) || (cnt > 4u)) return 0u;   /* 全灭 / 太宽 → 不算普通线 */
  return (uint8_t)(((uint8_t)(R - L + 1u) == cnt) ? 1u : 0u);  /* 中间没有暗缝 */
}

static uint8_t ge_is_entry_pattern(uint16_t raw)
{
  uint8_t i;
  for (i = 0; i < 4u; i++)
  {
    if (raw == kGourdEntryPat[i]) return 1u;
  }
  return 0u;
}
#endif /* USE_GOURD_EXIT */

/* ★起步盲直行（2026-09-24 用户要求）：读秒结束后先闷头直行 START_BLIND_MS，
   完全不看传感器 —— 起点那两条斑马线会把 PD / 十字判据全部带跑。
   剩余拍数由 line_follow_gourd_rearm() 在【每趟起步瞬间】装载（与出圈上膛同一处）。 */
#if USE_START_BLIND
static uint16_t start_blind = 0;
#endif

void line_follow_control(int16_t base)
{
  line_reading_t r = line_read();
  int8_t e = r.error;

#if USE_START_BLIND
  /* ---- ★起步盲直行（最高优先级，先于一切判据）----
     读秒刚结束那一小段：两轮等速直行，不给任何差速。
     为什么不算"开环有害"：这是【已知直道 + 车头已摆正 + 时间固定】的起点，
     不是他们复盘里被否定的"在弯道口闭眼转弯"。1 秒 ≈ 0.82m，足够越过起点区。 */
  if (start_blind > 0u)
  {
    start_blind--;
    motor_set_differential(sp_straight, sp_straight);
    last_error  = 0;      /* 把起点那几条斑马线带来的乱七八糟清干净, 别带给后面的 PD */
    last_e      = 0;
    lost_cycles = 0;
    line_lost   = 0;
    return;
  }
#endif

  /* ---- ★葫芦弯相切点检测（判据按 2026-09-22 实测数据定）----
     实测：相切点处传感条看到"图案分裂成两组、往两边分离"
        00110011 (3,4 + 7,8) → 11000011 (1,2 + 7,8) → 00000011 (只剩右2)
     而普通循迹是"连续一小片"(00011000 / 00011100)，
     十字是"一大片连着亮"(11111111, >=5 路) —— 三种特征天然分得开。
     规则：最左活跃 L 与最右活跃 R 之间空 >=2 路 → 判"分离"；分离期间只记 1 次（锁存）。*/
#if USE_GOURD_SM
  {
    /* ★★★ 2026-09-24 晚：相切点识别改用【1kHz 过采样】的位图（用户方案）★★★
       用户实测观察："小车在过葫芦圈里的弯道的时候转速特别大，
                     实际上的识别窗口就变小了" —— 完全正确：
       车转得快，"宽 + 只贴一端"这个图案可能只存在 1~3ms，
       而相切点判据原来只看 100Hz 那一拍（每 10ms 才采一次）→ 直接漏掉 →
       相切点识别不到 → 翻转修正不触发 → 车就锁在同一个圆上绕圈。
       ★本工程早就有 1kHz 采样器（SysTick 里 line_sample_1khz）：
         os1k_maxact / os1k_maxraw = 【本窗口内亮路数最多的那一帧的完整位图】。
         现在把它取出来，和当前拍的位图【取或】—— 哪个满足判据就算命中。
       ★为什么在这里取（而不是留给遥测）：line_1k_take 会【清零】窗口，
         两边各取一次就把窗口切成两半、谁也看不全。
         所以统一由这里取走，遥测的 K1 改读 line_follow_1k_missed()。 */
    uint16_t bm1k   = 0u;
    uint8_t  act1k  = 0u;
    s_1k_missed = line_1k_take(&s_1k_total, &act1k, &bm1k);
    s_1k_maxact = act1k;
    s_1k_maxraw = bm1k;

    /* ★2026-09-24 相切点判据【实测定稿·三条】(数据: _tmp/test2.txt 葫芦 vs 直角弯)
       ① 宽图案(>=CROSS_ACTIVE_MIN 路) 且【只贴一端】—— 直角弯也会命中, 单靠它不够
       ② GZ 在 ±8 拍内【翻号】—— 葫芦实测 -192→+200 (8 拍内)；直角弯宽图案时同号
       ③ 事件后 6 拍内【不出现全灭】—— 直角弯必全灭(~330ms)，相切点一直有线
       实现：①成立先挂"候选"(6 拍), 期内验收 ②③, 出现全灭立即作废。
       ★① 改成对【当前拍】和【1kHz 窗口最宽那帧】各算一次，任一命中即算成立。 */
    {
      uint8_t wide_one = 0u;
      for (uint8_t pass = 0u; (pass < 2u) && (wide_one == 0u); pass++)
      {
        uint16_t bm = (pass == 0u) ? r.raw : bm1k;
        uint8_t L = 0xFFu, R = 0u, cnt = 0u;
        if (bm == 0u) continue;                     /* 1kHz 那帧本来就没有 → 跳过 */
        for (uint8_t i = 0; i < LINE_CHANNELS; i++)
        {
          if (bm & (uint16_t)(1u << i)) { if (L == 0xFFu) L = i; R = i; cnt++; }
        }
        if ((L != 0xFFu) && (cnt >= (uint8_t)CROSS_ACTIVE_MIN))
        {
          uint8_t tL = (L == 0u)                   ? 1u : 0u;
          uint8_t tR = (R == (LINE_CHANNELS - 1u)) ? 1u : 0u;
          wide_one = (tL != tR) ? 1u : 0u;          /* 只贴一端 = 分支口/相切点 */
        }
      }
      {
        int8_t s = (imu_get_gyro_z() >= 0.0f) ? 1 : -1;
        if ((gz_sign != 0) && (s != gz_sign)) flip_win = 8u;   /* ★② 翻号 → 开窗口 */
        else if (flip_win) flip_win--;
        gz_sign = s;
      }
      if (cand_win)                                /* 候选期内验收 */
      {
        cand_win--;
        if (r.active == 0u) { cand_win = 0u; cand_ok = 0u; }    /* ★③ 全灭 → 作废 */
        else
        {
          if (flip_win) cand_ok = 1u;
          if ((cand_win == 0u) && cand_ok && !gourd_latch)
          {
            gourd_latch = 1u;                      /* 一个相切点只记一次 */
            /* ★★★ 2026-09-24 只统计【圈内】的相切点（队友方案第 2 步）★★★
               为什么必须加这道门：直角弯在【圈外】也会满足这三条判据
               （实测误报过 1 次，且不稳定）—— 不管它的话 GW 会被圈外误报推着走。
               配合"IG=0 强制清零 GW"（见本函数后面的 IG 块）：
                 圈外误报 -> 进门即清，绕完一圈回到 0 也清 -> 不依赖"误报是否稳定"。
               ★这里用的 ge_in_gourd 是【上一拍】的值（IG 块在本函数后面才更新），
                 差 1 拍 = 10ms，相对一圈几秒可忽略。 */
            if (ge_in_gourd && (gourd_waves < 200u))
            {
              /* ★★★ 2026-09-24 晚：相切点必须【彼此隔开】才算数（实车日志驱动）★★★
                 日志证据：OD=864 记 GW=1、OD=913 记 GW=2 —— 只隔 47mm！
                 47mm 不可能是两个真相切点（葫芦圈那几个圆是几百毫米量级）
                 ⇒ 误报是成簇的：一次真实宽图案被判成 2~3 个相切点 →
                   GW 提前凑满 3 → 归 0 → 右转提前触发 →
                   车在没有出口线的位置拐 85°，转完就是 IR 全灭。
                 修法：距上一个相切点不足 GOURD_WAVE_MIN_MM 毫米 → 不计数。 */
              if ((odom_distance_mm() - s_wave_odom) >= (int32_t)GOURD_WAVE_MIN_MM)
              {
#if 1   /* ★事件打印始终保留（纯观察，不改转向）；直行窗口由下面的赋值单独开关 */
                /* ★★★ 2026-09-25 相切点【判定成立】→ 给转向发一个"直穿过去"窗口 ★★★
                   为什么必须在这里发：相切点图案是"只贴一端"的宽图案，
                   进不了 on_cross（要求两端都贴）也进不了 wide_long（要求持续>6拍），
                   于是 PD 按单边大误差继续转 → 顺着圆弧绕圈
                   （实车：第 3 个圈绕了一整圈才跨过去）。
                   几何：两圆外切 → 相切点处共用一条切线 → 航向连续 → 直行正确。
                   为什么用这个门：cand_ok(陀螺翻号②) + 无全灭③ 才是"真相切点"，
                   圈外直角弯同样满足 wide_one，但过不了②③ → 不会被误加直行。 */
#if WIDE_ONE_GO_STRAIGHT
                s_wo_hold = (uint8_t)WIDE_ONE_HOLD_FRAMES;
#endif
                {
                  /* ★一次性事件打印：下一次跑车时，日志里【每个真相切点应当出现一行】，
                     出现几次 = 有几个相切点被确认；一行都没有 = 判据没成立。 */
                  char _m[80];
                  (void)snprintf(_m, sizeof(_m),
                                 "GOURD-TANGENT: straight %d fr (OD=%ld GW=%d)",
                                 (int)WIDE_ONE_HOLD_FRAMES, (long)odom_distance_mm(), (int)gourd_waves);
                  telemetry_msg(_m);
                }
#endif
#if GOURD_USE_FLIP
              /* ★★★ 2026-09-24 翻转修正【也必须在这道门里】★★★
                 实车病根：车进了葫芦圈就【锁在第 1 个圆上绕两圈】——
                 因为相切点处走线要从圆 1 跨到圆 2，而圆 2 的曲率方向与圆 1 相反，
                 PD 只顺着最强的那条线走 → 永远绕同一个圆。
                 翻转修正就是治它的：命中相切点后强制把误差取反 15 拍，
                 让车头【拐向另一个圆】。
                 ⚠️ 但它一旦在圈外命中（直角弯），会让车在那儿突然反向修正 →
                    直接把直角弯走废。所以必须跟着 ge_in_gourd 走。 */
              gourd_dir  = (last_error >= 0) ? 1 : -1;   /* 进相切点前的误差方向 */
              gourd_flip = GOURD_FLIP_CYCLES;            /* 反向修正保持的拍数 */
#endif
                s_wave_odom = odom_distance_mm();        /* 记下本次相切点的位置 */
                ge_confirmed = 1u;                      /* ★CARD-003：葫芦坐实（数到过>=1个相切点） */
                gourd_waves++;
                if (gourd_waves >= GOURD_TOTAL)
                {
                  gourd_waves    = 0;   /* 归 0 = 第 3 个相切点 = 绕完 4 个圆 */
                  gourd_wrap_evt = 1u;  /* ★置出圈事件（由出圈触发块消费） */
                  ge_done        = 1u;  /* ★CARD-003：相切点全部过完 */
                }
              }
            }
          }
        }
      }
      else if (wide_one && !gourd_latch)           /* ★① 起候选 */
      {
        cand_win = 6u; cand_ok = 0u;
      }
      else if (!wide_one)
      {
        gourd_latch = 0u;
      }
    }
  }
#endif

  /* ---- ★弯道计数：数"最右一路"(bit7)的【上升沿】次数（2026-09-22 他提的方案）----
     边沿计数 + 锁存间隔：一次持续点亮只算 1 次，且不会被 10ms 拍/速度绑死。
     遥测 G7= 就是它（跑一圈看它在葫芦出口读到多少 → 用它改 G7_TURN_TRIG）。 */
#if USE_G7_COUNT
  {
    uint8_t b7 = (uint8_t)((r.raw >> (LINE_CHANNELS - 1u)) & 1u);   /* bit7 = 最右一路 */
    if (g7_lock) g7_lock--;
    if (b7 && !g7_prev && (g7_lock == 0u))          /* 暗→亮 = 一次触发 */
    {
      if (g7_count < 60000u) g7_count++;
      g7_lock = G7_LOCKOUT;
    }
    g7_prev = b7;
  }
#endif

  /* ---- ★数到阈值 → 强制右转（默认先等"分支口"确认，见 G7_TURN_GATE_BRANCH）---- */
#if USE_G7_COUNT && USE_G7_TURN
  if ((g7_turn == 0u) && (g7_count >= G7_TURN_TRIG))
  {
    uint8_t fire = 1u;
#if G7_TURN_GATE_BRANCH
    /* 分支口 = 宽图案(>=G7_GATE_WIDE_MIN 路) 且 只贴一端（另一端空）= 葫芦出口/直角入口 */
    uint8_t L = 0xFFu, R = 0u, cnt = 0u;
    for (uint8_t i = 0; i < LINE_CHANNELS; i++)
    {
      if (r.raw & (uint16_t)(1u << i)) { if (L == 0xFFu) L = i; R = i; cnt++; }
    }
    if ((L == 0xFFu) || (cnt < G7_GATE_WIDE_MIN))  fire = 0u;
    else
    {
      uint8_t tL = (L == 0u) ? 1u : 0u;
      uint8_t tR = (R == (LINE_CHANNELS - 1u)) ? 1u : 0u;
      fire = (tL != tR) ? 1u : 0u;                 /* 只贴一端才算分支口 */
    }
#endif
    if (fire)
    {
      g7_turn    = G7_TURN_FRAMES;
      g7_count   = 0u;                             /* 触发后清零，重新开始数下一段 */
      g7_waitmsg = 0u;
      g7_flag    = 1u;
      telemetry_msg("GOURD EXIT (G7 count) -> FORCE RIGHT");
    }
    else if (!g7_waitmsg)
    {
      /* 数够了、但这一刻不是分支口 → 不转（防直道上拐出去），等分支口出现那一拍再转。
         这条打印是给你看的：出现它说明"阈值 12 和出口的位置对不上"，该改 G7_TURN_TRIG。 */
      g7_waitmsg = 1u;
      telemetry_msg("G7 HIT but not BRANCH (wait)");
    }
  }
#endif

  /* ==========================================================================
   * ★★★ 最边两路【掉路容忍】（2026-09-23 用户实测提出）
   *
   *  用户现象："小车从两个圆相切的地方出来时，由于转向存在微小的抖动，
   *           导致最边边上的传感器熄灭，影响到了正常的转向。"
   *
   *  为什么影响这么大（算一下就清楚）：
   *      正常   11000000  → error = -7 + -5 = -12   → 强右转
   *      最左熄灭 01000000 → error =      -5 =  -5   → 转向力度掉一半多
   *      → 立刻少转 → 车身更歪 → 抖动更大 → 【恶性循环】
   *
   *  修法：最左/最右那一路【单路亮、且它的内侧邻居是灭的】时，
   *        认为它是"抖出来的孤立毛刺"→ 用它的权重顶上去（相当于"补回来一个"）。
   *      · 最左补 -7、最右补 +7（恢复成"两路亮"的量级）
   *      · 只处理最外那一路，中间的路不管（中间抖动影响小，而且容易被误判）
   *      · 只在【按 error 转向】时生效（转弯中不看红外，不需要）
   * ========================================================================== */
#if USE_EDGE_HOLD
  {
    /* ★★★ 2026-09-23 驻留滤波【已回退】★★★
       我原来加了一段"最左/最右那一路必须连续亮 N 拍才算数，否则从位图里抹掉"，
       用来压"最边传感器瞬间碰线导致猛地一偏"。
       **用户实车反馈："转向现在非常不正常，没有上个版本好"** → 已停用。

       为什么它会伤转向（复盘）：
         · 它在【边缘刚接触的那一刻】把那一路抹掉 → 正常压边缘跟线时，
           转向指令会晚一拍、且力度先弱后强 → 相当于给转向加了个"迟滞"。
         · 葫芦圈里线一直在边缘来回扫 → 这个迟滞全程在起作用 → 转向发木/发飘。
         · 而它想修的"单拍毛刺"是【偶发】的，用一个【一直都在生效】的滤波去治偶发问题，
           代价（常态转向变差）大于收益（偶发少偏一次）。

       ★保留：下面"单路亮时 error×2"（那一条实测是有效的，用户没反馈它有问题）。
       ★要再试驻留滤波：把 EDGE_HOLD_CONFIRM 从 0 改成 2 即可（代码都在）。 */
#if (EDGE_HOLD_CONFIRM > 0)
    {
      uint8_t b0 = (uint8_t)((r.raw >> 0) & 1u);                     /* 最左 L1 */
      uint8_t bb = (uint8_t)((r.raw >> (LINE_CHANNELS - 1u)) & 1u);  /* 最右 L8 */

      if (b0) { if (ge_dwell_l  < 250u) ge_dwell_l++;  } else if (ge_dwell_l  > 0u) ge_dwell_l--;
      if (bb) { if (ge_dwell_r  < 250u) ge_dwell_r++;  } else if (ge_dwell_r  > 0u) ge_dwell_r--;

      /* 边缘那一路"还没站稳" → 从位图里抹掉，再用剩下的算 error */
      if ((b0 != 0u) && (ge_dwell_l < (uint8_t)EDGE_HOLD_CONFIRM))
      {
        r.raw    = (uint16_t)(r.raw & (uint16_t)~(uint16_t)(1u << 0));
        r.active = (uint8_t)((r.active > 0u) ? (r.active - 1u) : 0u);
        e        = (int8_t)(e - EDGE_L_WEIGHT);
        edge_rej_cnt++;
      }
      if ((bb != 0u) && (ge_dwell_r < (uint8_t)EDGE_HOLD_CONFIRM))
      {
        r.raw    = (uint16_t)(r.raw & (uint16_t)~(uint16_t)(1u << (LINE_CHANNELS - 1u)));
        r.active = (uint8_t)((r.active > 0u) ? (r.active - 1u) : 0u);
        e        = (int8_t)(e - EDGE_R_WEIGHT);
        edge_rej_cnt++;
      }
    }
#endif /* EDGE_HOLD_CONFIRM > 0 */

    /* ★单路亮时把 error ×2 —— 只在【恰好一路亮】时补偿。
       这时 error 就等于那一路的权重，而"正常"是【两路同时亮】的（权重表为两路设计）。
       ★为什么必须处理"只剩第2路"（用户实测场景）：
          正常     11000000 → error = -7 + -5 = -12
          最左熄灭 01000000 → error =      -5 =  -5   ← 转向力度掉一半多
          ×2 后 = -10，与 -12 同量级。 */
    if (r.active == 1u)
    {
      e = (int8_t)(e * 2);
      edge_hold_cnt++;
    }
  }
#endif

  /* ---- 速度自适应: 直道快/弯道慢（目标速度是运行时变量, S 指令能同步改） ---- */
  int16_t sp = base;
#if USE_ADAPTIVE_SPEED
  if (r.active == 0)
  {
    sp = sp_curve;                    /* 丢线先减速 */
  }
  else if (e >= CURVE_ABS_ERR_THRESH || e <= -CURVE_ABS_ERR_THRESH)
  {
    sp = sp_curve;                    /* 误差大 = 弯道, 减速 */
  }
  else
  {
    sp = sp_straight;                 /* 直道 */
  }
#endif

  /* ---- ★宽图案判定 ---- 真十字: 25mm 横线扫过 → 只持续几拍
     葫芦弯出口/直角入口: "圆切线+垂直线" → 持续更久 → 不能直行
     ★2026-09-22 15:31 修根因：十字还必须【两端都贴住】(CROSS_NEED_BOTH_ENDS) */
  {
    uint8_t wide    = (r.active >= CROSS_ACTIVE_MIN) ? 1u : 0u;
    uint8_t one_end = 0u;
#if CROSS_NEED_BOTH_ENDS
    if (wide)
    {
      uint8_t b0 = (uint8_t)( r.raw                  & 1u);   /* 最左一路 */
      uint8_t b7 = (uint8_t)((r.raw >> (LINE_CHANNELS - 1u)) & 1u);   /* 最右一路 */
      if (!(b0 && b7))
      {
        wide    = 0u;      /* 只贴一端 = 分支口/圆出口，不是十字 → 不直行，交给 PD 转 */
        one_end = 1u;
      }
    }
#endif
    /* ★2026-09-22 16:07 事件计数防抖（实跑：第 4 次真十字被判成假十字 → 车转弯）
       问题的形状：一次过十字若图案中间抖掉一帧，wide_cnt 归零 → 又冒出一个"上升沿"
       → 同一次过十字被数成 2 次 → 预算提前用完，把第 4 次真十字也拦了。
       ⚠️ 判定时机很讲究：必须在"宽图案段的**第一拍**"判 gap 并**锁存整段**。
         因为真正的上升沿出现在宽图案的第 2 拍（wide_cnt 到 CROSS_CONFIRM_CNT 才成事件），
         而第 1 拍已把 wide_gap 清零 —— 若等上升沿那拍才去看 gap，读到的永远是 0，
         会把**所有**事件一起按掉。（我第一次就写错了，靠模拟跑出来才发现。） */
    if (wide)
    {
      if (wide_cnt == 0u)                                    /* 段首：判定 + 锁存整段 */
        wide_armed = (wide_gap >= CROSS_REARM_FRAMES) ? 1u : 0u;
      if (wide_cnt < 200u) wide_cnt++;
      wide_gap = 0;
    }
    else
    {
      wide_cnt = 0;
      if (wide_gap < 250u) wide_gap++;
    }

    uint8_t detected = ((wide_cnt >= CROSS_CONFIRM_CNT) && (wide_cnt <= CROSS_MAX_FRAMES)) ? 1u : 0u;
    wide_long = (wide_cnt > CROSS_MAX_FRAMES) ? 1u : 0u;

    /* 每个"宽图案落在十字时间窗"的事件记一次（上升沿），供预算层比较。
       raw_rising 必须在 `det_prev = detected;` **之前**取 —— 否则编译器会发现
       `detected && !det_prev` 恒假，把下面"dup"那条打印优化掉（已验证过）。 */
    uint8_t raw_rising = (detected && !det_prev) ? 1u : 0u;    /* 上升沿（还没过防抖） */
    uint8_t det_rising = (raw_rising && wide_armed) ? 1u : 0u; /* 过了防抖才算真事件 */
    if (det_rising && (cross_events < 60000u)) cross_events++;
    det_prev = detected;

    uint8_t cross_now = detected;
#if USE_CROSS_BUDGET
    /* ★预算层：真十字触发次数有限（本赛道 = 2 个十字 × 2 次 = 4）→ 第 5 次起一律判为假十字
       （葫芦的假触发），交回 PD 转，不再强制直行。见 app_config.h 的说明与前提。 */
    if (cross_now && (cross_events > CROSS_BUDGET_MAX)) cross_now = 0u;
#if GOURD_NO_CROSS
    /* ★★★ 2026-09-24 晚【用户判断】葫芦圈里没有十字路口 ★★★
       用户原话："葫芦圈里是没有十字路口的，所以当识别到葫芦圈后，
                 后续判断疑似十字路口的都当葫芦圈处理"
       圈内出现的"疑似十字"其实是【葫芦圈与外面赛道的连接口(T/Y 型)】，
       在那里【该顺着线转弯】；强制直行必然冲丢线
         （日志实证 0924-2323：CROSS #2 IR=11100011 → IR=00000000 ×2），
       而且很可能把车【送上圈的错方向】→ 之后才开始绕圈。
       ⇒ 圈内(ge_in_gourd)一律不按十字/宽图案过久处理，
         两个"直行"分支一起关掉 → 交回普通 PD 循迹（该转就转）。
       位置说明：放在预算裁剪之后、on_cross 赋值之前，所以两条路都断干净。 */
    if (ge_in_gourd)
    {
      cross_now = 0u;
      wide_long = 0u;
    }
#endif
#endif

#if CROSS_PRINT
    char msg_cross[112];
    if (det_rising)                              /* 真事件打一行（上升沿，不刷屏） */
    {
      char irs[LINE_CHANNELS + 1];
      for (uint8_t i = 0; i < LINE_CHANNELS; i++) irs[i] = (r.raw >> i) & 1u ? '1' : '0';
      irs[LINE_CHANNELS] = '\0';
      if (cross_now)
      {
        snprintf(msg_cross, sizeof(msg_cross), "CROSS #%d IR=%s e=%+d wide=%d (straight)",
                 (int)cross_events, irs, (int)r.error, (int)wide_cnt);
      }
      else
      {
        snprintf(msg_cross, sizeof(msg_cross), "CROSS #%d IR=%s e=%+d BLOCKED(budget %d) -> turn",
                 (int)cross_events, irs, (int)r.error, (int)CROSS_BUDGET_MAX);
      }
      telemetry_msg(msg_cross);
    }
    else if (raw_rising)                         /* ★被防抖按下的重复事件 → 也打一行 */
    {
      char irs[LINE_CHANNELS + 1];
      for (uint8_t i = 0; i < LINE_CHANNELS; i++) irs[i] = (r.raw >> i) & 1u ? '1' : '0';
      irs[LINE_CHANNELS] = '\0';
      snprintf(msg_cross, sizeof(msg_cross), "CROSS dup IR=%s gap=%d (<%d) -> ignored",
               irs, (int)wide_gap, (int)CROSS_REARM_FRAMES);
      telemetry_msg(msg_cross);
    }
    if (one_end && !one_end_prev)                /* 宽图案只贴一端 → 形状层拦下了 → 打一行 */
    {
      char irs[LINE_CHANNELS + 1];
      for (uint8_t i = 0; i < LINE_CHANNELS; i++) irs[i] = (r.raw >> i) & 1u ? '1' : '0';
      irs[LINE_CHANNELS] = '\0';
      if (branch_events < 60000u) branch_events++;
      snprintf(msg_cross, sizeof(msg_cross), "BRANCH #%d IR=%s e=%+d -> NOT cross (turn)",
               (int)branch_events, irs, (int)r.error);
      telemetry_msg(msg_cross);
    }
#endif
    one_end_prev = one_end;
    on_cross     = cross_now;
  }

  /* ---- ★直线提速：连续"直线"够久 → 直接冲到 99（慢起快落）---- */
#if USE_STRAIGHT_BOOST
  if ((r.active > 0u) && (e <= STRAIGHT_BOOST_ERR_MAX) && (e >= -STRAIGHT_BOOST_ERR_MAX)
      && (!on_cross) && (!wide_long) && (!line_lost))
  {
    if (straight_cycles < 60000u) straight_cycles++;
  }
  else
  {
    straight_cycles = 0;
    boost_active = 0;                 /* 误差一大立刻回落 */
  }
  if (straight_cycles >= STRAIGHT_BOOST_DELAY)
  {
    /* ★斜坡提速：每拍最多涨 STRAIGHT_BOOST_RAMP，避免瞬跳（原版直接跳 99）*/
    int16_t tgt = STRAIGHT_BOOST_SPEED;
    if (sp < tgt)
    {
      sp = (int16_t)(sp + STRAIGHT_BOOST_RAMP);
      if (sp > tgt) sp = tgt;
    }
    else
    {
      sp = tgt;
    }
    boost_active = 1;
  }
#endif

  /* ==========================================================================
   * ★★★ 葫芦圈【状态检测】—— 维护"是否在葫芦圈里"这个状态
   *
   *  这一段【独立于出口逻辑】：只要有任一功能需要它（出圈转向 / 丢线屏蔽），就跑。
   *  产出的 ge_in_gourd 被两处消费：
   *      ① 出圈转向的闸门（没进圈就不许转 → 挡右直角弯）
   *      ② 丢线屏蔽（圈内不做丢线兜底 → 防被拽走）
   * ========================================================================== */
#if (USE_GOURD_EXIT || USE_GOURD_LOST_GUARD)
  {
    /* ==========================================================================
     * ★★★ 进圈判据（2026-09-23 换成几何判据，实车数据驱动）★★★
     *
     *  旧判据（4 个硬编码图案 `11100110` 等）在实车日志里 **一次都没匹配上**
     *  → `IG` 全程 0 → 依赖它的三件事（出圈闸门/出圈锁存/圈内丢线屏蔽）全没生效。
     *
     *  新判据：葫芦圈是【蛇行波浪】= 车持续在转弯 → 偏航率 |GZ| 长期很大；
     *         直线段 |GZ| 只有噪声级。
     *
     *  ★★★ 2026-09-23 追加【"车确实在跑"前置条件】★★★
     *  实车证据（FW:0923-3200 起步日志）：
     *      拍1 ST=2 RPM1=22     ← 刚起步（PWM≈6，几乎没动）
     *      拍2 ST=2 RPM1=-17251 ← 编码器毛刺
     *           GOURD-ENTRY mark set (in gourd)   ← ★就在这一拍误点了火
     *  原因：起步瞬间陀螺仪有暂态(GZ=141/-78) + 编码器毛刺，把窗口和凑够了。
     *  修法：**只有"至少一个轮子在转(RPM >= 阈值)"时，才把 |GZ| 喂进窗口**。
     *      · 静止/刚起步 → RPM 不够 → 不喂 → 窗口全 0 → IG 保持 0  ✓
     *      · 真正在跑葫芦圈 → 车速 100+ RPM，远超阈值 → 正常喂 → IG=1  ✓
     * ========================================================================== */
    uint8_t ig_now = 0u;
#if USE_GOURD_IG_GYRO
    {
      /* 同时喂两种来源，取"或"：
         ① 陀螺仪偏航率（主力，实测 28 倍分离度）
         ② 旧的 4 个图案（保底：万一 IMU 没通，还能靠图案蒙一把） */
      int16_t  rpmL = speed_get_rpm(MOTOR_LEFT);
      int16_t  rpmR = speed_get_rpm(MOTOR_RIGHT);
      uint8_t  moving = 0u;
      if (rpmL < 0) rpmL = (int16_t)(-rpmL);
      if (rpmR < 0) rpmR = (int16_t)(-rpmR);
      if ((rpmL >= (int16_t)GOURD_IG_MIN_RPM) || (rpmR >= (int16_t)GOURD_IG_MIN_RPM)) moving = 1u;

      {
        int16_t  gz  = (int16_t)imu_get_gyro_z();
        uint16_t gza = (uint16_t)((gz < 0) ? -gz : gz);
        if (!moving) gza = 0u;          /* ★没在跑 → 喂 0（窗口自然衰减到 0） */
        ge_gz_push(gza);
      }
      if (moving
          && (ge_gz_fill >= (uint8_t)GOURD_IG_WINDOW)
          && (ge_gz_sum >= (uint32_t)GOURD_IG_SUM_TH))
      {
        ig_now = 1u;
      }
    }
#endif
    if (ge_is_entry_pattern(r.raw)) ig_now = 1u;   /* ② 图案保底（或作对照） */

    if (ig_now)
    {
      ge_mark_ttl = (uint16_t)GOURD_ENTRY_MARK_TTL;    /* 在圈里 → 续命 */
      if (!ge_in_gourd)
      {
        ge_in_gourd  = 1u;                             /* ★0→1 边沿：进圈 */
        ge_plain_cnt = 0u;                             /* 复位普通线计数（否则本圈印记会被立刻清） */
        ge_stay      = 0u;                             /* ★CARD-003：IG=1 持续拍数从进圈边沿起算 */
        /* ★记下"进圈那一刻"的里程读数 —— 出口判据③（按里程强制右转）以它为 0 点。
           这样标定出来的 GOURD_EXIT_ODOM_MM 与"车从哪儿发车"无关，换位置不用重标。 */
        ge_odom_entry = odom_distance_mm();
#if USE_GOURD_SLOWDOWN
        ge_frames    = 0u;                             /* 后半段减速的计时从这里开始 */
#endif
#if GOURD_EXIT_ONE_SHOT
        /* ★解锁时机（2026-09-23 修）★
           从【进圈起算】(FROM_START=0) 时才在这里解锁 —— 它天然是"一圈一次"。
           从【起步起算】(FROM_START=1) 时【绝对不能】在这里解锁：
             那种模式的判据是 `里程 >= GOURD_EXIT_ODOM_MM`，
             **一旦成立就永远成立**；在这里一解锁，车跑到普通线上
             就会被再触发一次 → 每隔一段猛地右转一次，无限重复。
             那种模式的解锁只在 line_follow_gourd_rearm()（每趟起步时调一次）。 */
#if !(USE_GOURD_EXIT_ODOM && GOURD_EXIT_ODOM_FROM_START)
        ge_fired = 0u;                                 /* ★只在这里解锁出圈锁存 */
#endif
#endif
        telemetry_msg("GOURD-ENTRY mark set (in gourd)");
      }
    }
    else if (!ge_confirmed)
    {
      /* 【Path②】GW 未坐实：旧快清【原样保留】——圈外误点 IG(普通弯道/直角弯) 200ms 内释放。
         ★离开葫芦圈的判据：连续 GOURD_ENTRY_CLEAR_FRAMES 拍是"普通线"才算出去。
         （葫芦圈里传感器一直看到"有暗缝的分离图案"；出圈后就是普通一条线。）
         ★TTL 只当保险：万一一直不出现普通线，最多 GOURD_ENTRY_MARK_TTL 拍后强制清。 */
      if (ge_mark_ttl > 0u) ge_mark_ttl--;

      if (ge_is_plain_line(r.raw))
      {
        if (ge_plain_cnt < 60000u) ge_plain_cnt++;
      }
      else
      {
        ge_plain_cnt = 0u;
      }

      /* ★★★ 2026-09-23 修一个"每分钟刷 200 次"的 bug ★★★
         原写法： if (plain >= N || ttl == 0) { 清印记; 打印; }
         问题： **ttl 到 0 之后 `ttl == 0` 这个条件【永远成立】** →
              车一停在普通线上，就每拍打印一次 "mark cleared"，串口被灌满；
              而且 ge_in_gourd 会在 0/1 之间反复跳，**丢线屏蔽跟着乱开乱关**。
         实车证据：架空测试日志里 `GOURD-ENTRY mark cleared` 每隔 2~3 拍刷一次。
         修法：加一道 `ge_in_gourd` 守卫 —— **只有在"印记还是 1"时才去清它**。
              清完 ge_in_gourd 变 0，下一拍这个分支根本不进来 → 彻底不刷。
              TTL 仍然起作用：它在"印记还是 1"的时候把 ttl 减到 0，
              下一拍由 `ttl == 0` 条件把印记清掉（只清一次）。 */
      if (ge_in_gourd
          && ((ge_plain_cnt >= (uint16_t)GOURD_ENTRY_CLEAR_FRAMES) || (ge_mark_ttl == 0u)))
      {
        ge_in_gourd  = 0u;                             /* 确认离开葫芦圈 */
        ge_plain_cnt = 0u;
        ge_mark_ttl  = 0u;
        ge_confirmed = 0u;                             /* ★CARD-003：IG 清 0 同清四个新状态 */
        ge_done      = 0u;
        ge_straight  = 0u;
        ge_stay      = 0u;
#if GOURD_EXIT_ONE_SHOT
        /* ★解锁时机：同上 —— 从起步起算时这里也不解锁（会无限重复右转）。 */
#if !(USE_GOURD_EXIT_ODOM && GOURD_EXIT_ODOM_FROM_START)
        ge_fired = 0u;                                 /* 解锁, 为下一圈准备 */
#endif
#endif
        telemetry_msg("GOURD-ENTRY mark cleared (left gourd, back on plain line)");
      }
    }
    /* ★CARD-003：ge_confirmed==1 且 ig_now==0 → 什么都不做，IG 保持 1（修"圈内乱跳 0"） */

    /* ★CARD-003：GW 坐实后，IG 只在下面两条路径清（正常出圈 / 兜底） */
    if (ge_in_gourd)
    {
      if (ge_stay < 60000u) ge_stay++;

      /* 【Path①】正常出圈：GW 数满后，连续普通线 GOURD_EXIT_STRAIGHT_FRAMES 拍。
         直角弯图案不是普通线（ge_is_plain_line 返回 0）→ ge_straight 天然清零。 */
      if (ge_done)
      {
        if (ge_is_plain_line(r.raw)) ge_straight++;
        else                         ge_straight = 0u;
        if (ge_straight >= (uint16_t)GOURD_EXIT_STRAIGHT_FRAMES)
        {
          ge_in_gourd  = 0u;
          ge_plain_cnt = 0u;
          ge_mark_ttl  = 0u;
          ge_confirmed = 0u;
          ge_done      = 0u;
          ge_straight  = 0u;
          ge_stay      = 0u;
#if GOURD_EXIT_ONE_SHOT
#if !(USE_GOURD_EXIT_ODOM && GOURD_EXIT_ODOM_FROM_START)
          ge_fired = 0u;
#endif
#endif
          telemetry_msg("GOURD-ENTRY mark cleared (done+straight)");
        }
      }

      /* 【Path③】兜底：IG=1 持续 GOURD_IG_FAILSAFE_FRAMES 拍强清（防 GW 数不满死在圈里） */
      if (ge_stay >= (uint16_t)GOURD_IG_FAILSAFE_FRAMES)
      {
        ge_in_gourd  = 0u;
        ge_plain_cnt = 0u;
        ge_mark_ttl  = 0u;
        ge_confirmed = 0u;
        ge_done      = 0u;
        ge_straight  = 0u;
        ge_stay      = 0u;
#if GOURD_EXIT_ONE_SHOT
#if !(USE_GOURD_EXIT_ODOM && GOURD_EXIT_ODOM_FROM_START)
        ge_fired = 0u;
#endif
#endif
        telemetry_msg("GOURD-IG FAILSAFE clear");
      }
    }
  }

  /* ★★★ 2026-09-24 IG=0 → 强制清零相切点计数（队友方案第 2 步的后半）★★★
     为什么：直角弯在圈外也会满足相切点三条判据（实测误报过，且不稳定）。
     "GW 只在 IG=1 时累计"挡住进门，"IG=0 强制清零"负责出门即清 ——
     两道一起，圈外误报无论稳不稳定都翻不出浪。
     ★放在 IG 块的末尾：这一拍的 ge_in_gourd 已经算完，用的是最新值。 */
#if USE_GOURD_SM
  /* ★★★ 2026-09-24 晚 修（实车日志证据）★★★
     原来：IG=0 就【立刻】清零 GW。
     日志证据：GW 反复走 0→1→2→(被清)→0→1→2，永远到不了 3（GOURD_TOTAL）→
              出圈永远不触发。原因：ge_in_gourd(IG) 本身在实车上【乱跳】
              （一秒内 0/1 反复多次，日志里 "GOURD-ENTRY mark set/cleared" 交替出现）。
     修法：改成"离开确认"—— 连续 GOURD_IG_LEAVE_FRAMES 拍 IG=0 才清。
           瞬时抖动不再抹掉已经数到的相切点。
     ★为什么还留着清零：直角弯在圈外的误报必须能被清掉（队友方案的本意），
       只是不该被"一次抖动"清掉。 */
  if (ge_in_gourd)
  {
    s_ig_leave = 0u;                       /* 还在圈里 → 确认计数复位 */
  }
  else if (s_ig_leave < (uint16_t)GOURD_IG_LEAVE_FRAMES)
  {
    s_ig_leave++;                          /* 刚出圈，先观察 */
  }
  else
  {
    gourd_waves    = 0u;                   /* 确认离开了 → 才清 GW */
    gourd_wrap_evt = 0u;
    ge_confirmed   = 0u;                   /* ★CARD-003：清 GW 同处同清（IG 早已清过，保险再清） */
    ge_done        = 0u;
    ge_straight    = 0u;
    s_wave_odom    = odom_distance_mm();   /* ★同时把"上一个相切点位置"对齐到现在 ——
                                              这样下一圈的第一个相切点不会被上一圈的
                                              位置门槛误挡掉 */
  }
#endif
#endif /* USE_GOURD_EXIT || USE_GOURD_LOST_GUARD */

  /* ★★★ 葫芦圈【后半段减速】（2026-09-23）
     用户实测："有时候走在圆圈的衔接处会直直的开出去，传感器没识别到就停车了。
               感觉最后半圈可以减个速。"
     原因：外切点 = 曲率突变点（小圆→大圆）。PD 慢一拍 → 在衔接处切内角 →
           车身斜着出切点 → 从两圆之间的缝隙穿过去（无线）→ 全灭 → 丢线停车。
     减速对症：速度低 → 每拍位移小 → 修正更充分 → 跟得上曲率突变。
     ★这里按【拍数】计时（进圈那一刻为 0）。
       ★加在 boost 之后：否则直线提速会把减速又拉回去。 */
#if USE_GOURD_SLOWDOWN
  if (ge_in_gourd)
  {
    if (ge_frames < 60000u) ge_frames++;
    if (ge_frames >= (uint16_t)GOURD_SLOW_AFTER)
    {
      sp = (int16_t)((sp * GOURD_SLOW_PCT) / 100);
      if (sp < 10) sp = 10;            /* 别减到停住 */
    }
  }
  else
  {
    ge_frames = 0u;
  }
#endif

  /* ==========================================================================
   * ★★★ 葫芦圈【出口】：只有最左 N 路亮 → 直接右转出圈
   *  优先级最高（出口特征比"数最右一路"更直接）。
   *  转的时候【闭着眼转】：故意急转时传感器误差是误导的，会把自己拽回来 → 来回摆。
   *
   *  "转够了没有"用三级判据（见 app_config.h 的说明）：
   *      ① 积分航向角 >= GOURD_EXIT_TURN_DEG   ← 最准，需 IMU 通
   *      ② 编码器里程 >= GOURD_EXIT_TURN_DIST  ← 不受电量影响
   *      ③ 拍数 >= GOURD_EXIT_TURN_FRAMES       ← 最后保险
   * ========================================================================== */
#if USE_GOURD_EXIT
  {
    /* 冷却递减 */
    if (ge_cool > 0u) ge_cool--;

    /* ★闸门 gate_ok 只被【图案判据①】消费 → 跟着 GOURD_EXIT_USE_PATTERN 走，
       否则关掉图案判据时会报 `#550-D: gate_ok was set but never used`。 */
#if GOURD_EXIT_USE_PATTERN
    uint8_t gate_ok = 1u;

    /* 闸门（默认关）：要求已经数到足够多的相切点 = 确实在葫芦圈里 */
#if (GOURD_EXIT_GATE_WAVES > 0)
    if (gourd_waves < (uint8_t)GOURD_EXIT_GATE_WAVES) gate_ok = 0u;
#endif
#if GOURD_EXIT_REQUIRE_ENTRY
    /* 只有进过圈才允许出圈转向（右直角弯入口的图案和出圈一样，靠这个挡住） */
    if (!ge_in_gourd) gate_ok = 0u;
#endif
#if GOURD_EXIT_ONE_SHOT
    /* ★一次性锁存：本次进圈已经转过一次 → 不再转。
       为什么必须要有：葫芦圈的路径是【蛇行波浪】，
       "只有最左两路亮"这个特征在每个拐点都可能出现，不只是出口。
       不锁存的话一次进圈会转好几次，把波浪走废。 */
    if (ge_fired) gate_ok = 0u;
#endif
#endif /* GOURD_EXIT_USE_PATTERN */

#if USE_IMU
    /* IMU 在 + 已标定 → 才允许用角度闭环 */
    uint8_t yaw_ok = (uint8_t)((GOURD_EXIT_USE_YAW && imu_yaw_is_valid()) ? 1u : 0u);
#else
    uint8_t yaw_ok = 0u;
#endif

    /* 读两轮编码器 → 本拍前进的计数（距离兜底用） */
    {
      int32_t curL = encoder_get_count(MOTOR_LEFT);
      int32_t curR = encoder_get_count(MOTOR_RIGHT);
      int32_t dL = curL - ge_encL, dR = curR - ge_encR;
      ge_encL = curL; ge_encR = curR;
      if (dL < 0) dL = -dL;
      if (dR < 0) dR = -dR;
      ge_moved = (uint32_t)(dL + dR);
    }

    /* ==========================================================================
     * ★★★ 出口识别：两条判据，满足任一条就起转（2026-09-23）★★★
     *
     *  判据①（图案）：只有最左 N 路亮（默认 `11000000`）
     *      —— 用户实测过这个特征，但"可能只闪一下、甚至不出现"。
     *
     *  判据②（兜底，不依赖图案）：**在葫芦圈里 + 持续丢线**
     *      为什么它是对的：葫芦圈内传感器【一直】能看到某种图案（哪怕是零散的），
     *      `line_lost` 基本不会被置起来；等绕到最后一个圆结束、线没了 →
     *      才会出现"连续好几拍全灭" → 这就是出口。
     *      所以"圈内持续丢线"几乎是出口的专属信号，且不依赖任何具体图案。
     *
     *  ★两条都用：①先命中就用①，①没命中就用②兜底 → 最大限度避免"出不去"。
     *  ★一次性锁存(gear_fired)保证一次进圈只转一次，不会两条判据各转一次。
     * ========================================================================== */
    uint8_t trig_now = 0u;

    if ((ge_cool == 0u) && (ge_turning == 0u) && (!ge_fired))
    {
#if GOURD_EXIT_USE_PATTERN
      /* ---- 判据①：出口图案（滑动窗口累加，容忍"闪一下"） ---- */
      if (ge_win > 0u) ge_win--;        /* 窗口倒计时 */

      if (gate_ok && ge_is_exit_pattern(r.raw))
      {
        if (ge_exit_cnt < 250u) ge_exit_cnt++;
        ge_win = (uint16_t)GOURD_EXIT_WINDOW;      /* 见到就续窗口 */
      }
      if (ge_win == 0u)
      {
        ge_exit_cnt = 0u;             /* 窗口过期 → 重新开始累加 */
      }
      if (ge_exit_cnt >= (uint8_t)GOURD_EXIT_CONFIRM) trig_now = 1u;
#endif /* GOURD_EXIT_USE_PATTERN */

#if GOURD_EXIT_ON_LOST
      /* ---- 判据②：圈内持续丢线 = 出口兜底 ----
         ★用 lost_cycles 判断，不用 line_lost：
           `line_lost` 是在本函数【下面】的丢线处理里更新的 → 这里读到的是上一拍的值。
           `lost_cycles` 是本拍已经累加过的全灭计时器，不受位置影响。
           条件与丢线处理的判定一致：lost_cycles > LOST_SPIN_DELAY 才算"持续丢线"。 */
      if ((trig_now == 0u) && ge_in_gourd && (lost_cycles > (uint16_t)LOST_SPIN_DELAY))
      {
        trig_now = 2u;
      }
#endif

#if USE_GOURD_EXIT_ODOM
      /* ---- 判据③：按【里程】强制右转（用户方案，2026-09-23）----
         ★它和"看到了什么图案"完全无关 → 右直角弯出口的 `11000000` 骗不到它。
         ★它和"圈内是否丢线"也无关   → 印记偶尔乱跳也不怕。

         两种起算基准，由 GOURD_EXIT_ODOM_FROM_START 选（默认 0）：
           0 = 从【进圈那一刻】起算：odom_distance_mm() - ge_odom_entry
               要求 ge_in_gourd（没进过圈就不许按里程转 → 防止在别的路段乱转）。
               ✔ 与"车从哪儿发车"无关。
               ✔ 对右直角弯天生免疫：它要求 ge_in_gourd 连续保持整整 403mm，
                 而直角弯的印记 200ms 就清了，撑不到。
           1 = 从【起步】起算：odom_distance_mm()
               ⚠️ 车只要跑够 403mm 就转，与在赛道哪个位置无关 ——
                  只有"403 就是从起点线量的、且每次从同一点发车"时才用。 */
      if (trig_now == 0u)
      {
        int32_t od_ref = 0;
        uint8_t od_ok  = 1u;
#if GOURD_EXIT_ODOM_FROM_START
        od_ref = 0;                        /* 从起步(odom_reset)起算 */
#else
        od_ref = ge_odom_entry;            /* 从进圈那一刻起算 */
        if (!ge_in_gourd) od_ok = 0u;      /* ★没进过圈 → 不许按里程转 */
#endif
        if (od_ok && ((odom_distance_mm() - od_ref) >= (int32_t)GOURD_EXIT_ODOM_MM))
        {
          trig_now = 3u;
        }
      }
#endif

#if USE_GOURD_SM
      /* ---- 判据④：相切点计数归 0（= 第 3 个相切点）= 出圈 ★2026-09-24 队友方案第 3 步★
         GW 只在 IG=1 时累计、IG=0 时强制清零（见相切点检测那一段和 IG 块末尾）——
         所以圈外（直角弯）的误报"进门不许计、出门即清"，不依赖"误报是否稳定"。
         第 3 个相切点一到，GW 归 0 → 这一个事件就是出口。
         ★不直接用 `gourd_waves == 0` 判：发车时它本来就是 0，会一起步就右转。

         ★2026-09-24 晚：本判据已由 GOURD_EXIT_USE_GW 【关掉】——
           实车验证"仍然走不了葫芦圈"，用户要求改回【里程】判据。
           GW 的计数与遥测照旧（还能继续观察相切点判据准不准）。
         ★事件位无论开关都【照样消费】：这样 gourd_wrap_evt 永远是被读的，
           不会退化成"只写不读"→ ARMCC #550-D 告警（本项目要求 0 警告）。 */
      if ((trig_now == 0u) && gourd_wrap_evt)
      {
#if GOURD_EXIT_USE_GW
        trig_now       = 4u;
#endif
        gourd_wrap_evt = 0u;          /* 消费掉，一次只转一次（关掉时也不留残值） */
      }
#endif

    }

    /* ---- 起转 ---- */
    if (trig_now != 0u)
    {
      ge_turning = 1u;
      ge_settle  = 0u;                              /* ★起转时清掉"停一停"计数 */
      ge_cool    = (uint16_t)GOURD_EXIT_COOLDOWN;   /* 起转就进冷却 → 防同一次出口重复起转 */
#if GOURD_EXIT_USE_PATTERN
      ge_exit_cnt = 0u;
      ge_win      = 0u;
#endif
      ge_turn_dist = 0u;
      ge_turn_frames = 0u;
#if GOURD_EXIT_ONE_SHOT
      ge_fired = 1u;                                /* ★上锁：本次进圈不再转第二次 */
#endif
      if (yaw_ok) { imu_yaw_reset(); }              /* 以起转时刻为 0°，之后读到的是转了多少 */
      if (ge_events < 60000u) ge_events++;
      /* ★事件行里带上 ODE= —— 这就是"进圈→出口"的真实里程，标定 GOURD_EXIT_ODOM_MM 直接抄它 */
      {
        char ev[128];
        (void)snprintf(ev, sizeof(ev),
          "GOURD-EXIT: trig=%u -> turn RIGHT   ODE=%ldmm  yaw=%u GW=%u  (%s)",
          (unsigned)trig_now,
          (long)(odom_distance_mm() - ge_odom_entry),
          (unsigned)yaw_ok,
          (unsigned)gourd_waves,
          (trig_now == 4u) ? "by TANGENT-COUNT (GW wrapped to 0)"
                           : ((trig_now == 3u) ? "by MILEAGE"
                           : ((trig_now == 2u) ? "in-gourd + lost line"
                                               : (yaw_ok ? "left2 lit, yaw closed loop"
                                                         : "left2 lit, dist fallback"))));
        telemetry_msg(ev);
      }
    }

    /* ---- 转的过程中：闭眼给固定差速, 直到"转够" ---- */
    if (ge_turning)
    {
      int16_t a = (int16_t)(sp_curve + GOURD_EXIT_TURN_PWM);   /* 左快右慢 = 右转 */
      int16_t b = (int16_t)(sp_curve - GOURD_EXIT_TURN_PWM);
      if (a >  99) a =  99;  if (a < -99) a = -99;
      if (b >  99) b =  99;  if (b < -99) b = -99;
      motor_set_differential(a, b);

      last_error  = 3;          /* 给退出后的 PD 一个合理起点（偏右） */
      lost_cycles = 0;
      line_lost   = 0;

      ge_turn_dist   += ge_moved;
      ge_turn_frames++;

#if USE_IMU
      if (yaw_ok) ge_turn_deg = imu_get_yaw();      /* 负=右转, 取绝对值 */
      else        ge_turn_deg = 0.0f;
#endif

      /* ---- 转够了没有？（三级判据）---- */
      {
        uint8_t done = 0u;
#if USE_IMU
        float deg_abs = (ge_turn_deg < 0.0f) ? -ge_turn_deg : ge_turn_deg;
        if (yaw_ok && (deg_abs >= GOURD_EXIT_TURN_DEG)) done = 1u;          /* ① 角度够了 */
#else
        float deg_abs = 0.0f;
#endif
        if (ge_turn_dist   >= (uint32_t)GOURD_EXIT_TURN_DIST)  done = 1u;   /* ② 距离上限 */
        if (ge_turn_frames >= (uint16_t)GOURD_EXIT_TURN_FRAMES) done = 1u;  /* ③ 拍数上限 */

        if (done)
        {
          char msg[96];
          ge_turning = 0u;
          ge_settle  = (uint16_t)GOURD_EXIT_SETTLE_FRAMES;   /* ★转完先停一停再走 */
          ge_last_deg = deg_abs;
          snprintf(msg, sizeof(msg), "GOURD-EXIT done: deg=%.1f dist=%lu frames=%u",
                   (double)deg_abs, (unsigned long)ge_turn_dist, (unsigned)ge_turn_frames);
          telemetry_msg(msg);
        }
      }
      return;                    /* ★转弯期间完全不看红外 */
    }

    /* ==========================================================================
     * ★★★ 转完之后的"停一停"（2026-09-23，用户要求"转弯停一会在走"）★★★
     *
     *  为什么加：上面那段转动是"闭着眼"给固定差速做的。到目标角度那一刻
     *           车【还在转】（有角速度）→ 立刻交回 PD，PD 会在车没停稳时
     *           就开始修正，更容易转过头。
     *  做法：转完先让两轮 PWM 归 0（滑行），保持 GOURD_EXIT_SETTLE_FRAMES 拍
     *       （默认 30 拍 = 300ms）不动，把惯性吃掉，再交回 PD。
     *  ★期间也必须 return —— 否则会掉进下面的 PD，等于白停。
     *  ★同时清掉丢线计数：停着不动时红外可能读不到线，不清会把"丢线"积起来，
     *    一交回 PD 就立刻进入原地找线。
     * ========================================================================== */
    if (ge_settle > 0u)
    {
      ge_settle--;
      motor_set_differential(0, 0);   /* 两轮停：把转动惯性放掉 */
      lost_cycles = 0;
      line_lost   = 0;
      return;
    }
  }
#endif /* USE_GOURD_EXIT */

  /* ---- ★强制右转提交：硬转期间【闭着眼转】，忽略传感器，转完再交回 PD ----
     优先级高于丢线/PD：故意急转的时候传感器误差是误导的（会把自己拽回来 → 来回摆）。 */
#if USE_G7_COUNT && USE_G7_TURN
  if (g7_turn)
  {
    g7_turn--;
    int16_t a = (int16_t)(sp_curve + G7_TURN_PWM);   /* 左快右慢 = 右转 */
    int16_t b = (int16_t)(sp_curve - G7_TURN_PWM);
    if (a >  99) a =  99;  if (a < -99) a = -99;
    if (b >  99) b =  99;  if (b < -99) b = -99;
    motor_set_differential(a, b);
    last_error  = 3;                 /* 给退出后的 PD 一个合理起点（偏右） */
    lost_cycles = 0;
    line_lost   = 0;
    return;
  }
#endif

  /* ---- 丢线处理 ----
     ★★★ 葫芦圈内【屏蔽丢线兜底】（2026-09-23）
     用户实测："过葫芦圈的时候防丢线机制经常干扰小车的运动，
               但它在除了葫芦圈以外的地方又确实挺有用。"
     原因：圈内传感器一直看到"有暗缝的分离图案"，丢线机制误判成"找不到线" →
           先加倍修正、再原地旋转找线 → 把车从赛道上拽走。
     做法：圈内走"保持航向"，圈外完全保留原有兜底（一行没改）。 */
#if USE_GOURD_LOST_GUARD
  if (ge_in_gourd)
  {
    /* 圈内：不做丢线兜底，只维持一个前向误差让车继续按当前航向走。
       ★这里【不清】last_error（冻结方向），也不累加 lost_cycles、不置 line_lost，
         所以下面"持续丢线原地旋转"那段不会被触发。 */
#if (GOURD_LOST_MODE == 1)
    e = (GOURD_LOST_HOLD_ERR != 0) ? (int8_t)GOURD_LOST_HOLD_ERR
                                   : last_error;     /* 冻结：沿用最后一拍的真实方向 */
#elif (GOURD_LOST_MODE == 2)
    e = 0;                                           /* 滑行直行 */
#else
    /* MODE 3：什么都不做，原样交给 PD（全灭时 error 本来就是 0） */
#endif
    lost_cycles = 0;
    line_lost   = 0;
  }
  else
#endif /* USE_GOURD_LOST_GUARD */
  if (r.active == 0)
  {
    lost_cycles++;
    if (lost_cycles <= LOST_SPIN_DELAY)
    {
      line_lost = 0;
#if GOURD_LOST_HOLD
      /* ★★★ 短暂丢线时"保持上次方向"而不是"把 last_error 翻倍"（2026-09-23 修）
         原写法 e = last_error × 2 有一个致命失效模式（文档里早就写了）：
             若丢线那一拍车【正好居中】(last_error ≈ 0) → 翻倍还是 0 →
             **完全没有转向指令 → 直着冲出去**。
         葫芦圈的外切点正好会撞上这个：切点处线从中间滑走、车身居中，
         然后从两圆之间的缝隙穿过去 → 全灭 → e=0 → 直行 → 越走越远。
         新写法：**冻结丢线前那一刻的 error**（含正负号、不翻倍、不清零），
         让车"按丢线前的转向继续转" → 正好是过圆需要的动作。
         ★注意：这不是"取消兜底"，圈内/圈外都只是把"短暂丢线"这一段改得更稳；
           持续丢线（超过 LOST_SPIN_DELAY 拍）仍然会原地旋转找线。 */
      if (ge_in_gourd)
      {
        /* 圈内：连正负号都保住，方向最真实 */
        e = (GOURD_LOST_HOLD_ERR != 0) ? (int8_t)GOURD_LOST_HOLD_ERR : last_error;
      }
      else
      {
        /* 圈外：原来的兜底意图保留（沿上次方向加强），但居中时给个最小方向，
           不再出现"e=0 → 直行"的死角 */
        int16_t ee = (int16_t)(last_error * 2);
        if (ee == 0 && last_error != 0) ee = last_error;   /* 防截断成 0 */
        if (ee == 0) ee = GOURD_LOST_MIN_ERR;              /* 居中丢线：给最小方向 */
        e = (int8_t)ee;
      }
#else
      e = (int8_t)(last_error * 2);   /* 短暂丢线: 沿上次方向加强修正 */
#endif
#if USE_GOURD_SM
#if GOURD_USE_FLIP
      if (gourd_flip) e = (int8_t)(-gourd_dir * GOURD_FLIP_ERR);
#endif
#endif
    }
    else
    {
      line_lost = 1;                  /* 持续丢线: 原地旋转找线 */
      e = 0;
    }
  }
  else
  {
    last_error = r.error;
    lost_cycles = 0;
    line_lost = 0;
  }

  /* ---- 持续丢线: 原地旋转找线（超时停车防跑飞）---- */
  if (line_lost)
  {
    if (lost_cycles > LOST_MAX_CYCLES)
    {
      lost_cycles = (uint16_t)(LOST_MAX_CYCLES + 1);   /* 钉死，别让计数继续涨 */
      motor_stop();
    }
    else if (last_error >= 0)
    {
      motor_set_differential(LOST_SPIN_SPEED, -LOST_SPIN_SPEED);
    }
    else
    {
      motor_set_differential(-LOST_SPIN_SPEED, LOST_SPIN_SPEED);
    }
    return;
  }

#if GOURD_ARC_FLIP
  /* ★★★ 2026-09-24 晚【绕圈脱困】同号偏航累计 → 到阈值就反向打舵 ★★★
     为什么必须靠陀螺、不能靠 IR 图案（这是本轮最重要的结论）：
       两个【外切】圆在相切点共用一条切线 → 走线 C¹ 连续、只换曲率符号，
       传感条看到的是一条【普通光滑的线】，不变宽也不分叉
       —— 日志实证：车在圆上绕了 2 圈（1240mm），一次宽图案都没出现。
       所以"靠宽图案/咬线找相切点"这条路【在几何上就不成立】。
     靠陀螺：葫芦圈是 S 形 → 带符号积分左右抵消 ≈0；
             绕圈是单方向 → 累计到 ±180° 以上。
             而相切点正好在"同向转够约半圈"处（曲率换号点）。
     动作：朝【当前误差的反面】打舵 GOURD_ARC_FLIP_MAX 拍，
           打完积分清零，等下一次同号累计 —— 即"每到半圈换一次向"。
           方向用误差的反面推出来，所以不依赖陀螺/电机的符号约定。 */
  {
    int16_t gz = (int16_t)imu_get_gyro_z();              /* dps */
    if (!ge_in_gourd)
    {
      s_arc_mdeg = 0; s_arc_push = 0; s_arc_dir = 0;     /* 出圈即复位 */
    }
    else
    {
      if ((gz > GOURD_ARC_DB_DPS) || (gz < -GOURD_ARC_DB_DPS))
        s_arc_mdeg += (int32_t)gz * 10;                  /* dps×10ms = 0.01° = 10 毫度 */
      else
        s_arc_mdeg -= s_arc_mdeg / 16;                   /* 死区内缓慢衰减，抑制零漂 */

      if (s_arc_push)
      {
        s_arc_push--;                                    /* 反向打舵中 */
      }
      else if ((s_arc_mdeg > (int32_t)GOURD_ARC_FLIP_MDEG) ||
               (s_arc_mdeg < -(int32_t)GOURD_ARC_FLIP_MDEG))
      {
        if (last_error != 0)                             /* 方向 = 当前误差的【反面】 */
        {
          s_arc_dir  = (int8_t)((last_error > 0) ? -1 : 1);
          s_arc_push = (uint8_t)GOURD_ARC_FLIP_MAX;
          s_arc_mdeg = 0;                                /* 清零，等下一次同号累计 */
          {
            char _m[80];
            (void)snprintf(_m, sizeof(_m), "GOURD-ARC: flip dir=%d OD=%ld GW=%d",
                           (int)s_arc_dir, (long)odom_distance_mm(), (int)gourd_waves);
            telemetry_msg(_m);
          }
        }
      }
    }
  }
#endif

  /* ---- 真十字: 强制直行（清 last_e，防出十字瞬间 D 项踢一脚）---- */
#if WIDE_ONE_GO_STRAIGHT
  if (s_wo_hold) s_wo_hold--;   /* ★相切点直行窗口按拍递减（本拍仍 >0 就直行）*/
#endif
  if (on_cross || s_wo_hold)    /* ★2026-09-25 相切点(只贴一端)也强制直行 */
  {
    e = 0;
    last_e = 0;
  }
  /* ---- 宽图案持续过久 → 不是十字（葫芦出口/直角入口）→ 该怎么走？ ---- */
  else if (wide_long)
  {
#if WIDE_LONG_GO_STRAIGHT
    /* ★★★ 2026-09-24 晚【改成走直线】—— 这是"绕圈"的头号嫌疑 ★★★
       原来是 e = last_error * 2（沿上次方向【加倍】接着转）。
       但葫芦圈【内部】的相切点恰恰是"宽图案 + 只贴一端"
       （日志里 BRANCH #1..#20 一次跑就出现 20 次 —— 这个特征在圈里极其常见），
       于是车在相切点被命令【加倍往同一个方向转】→ 永远顺着圆走 → 绕圈出不来。
       ★几何上正确的动作是【直着穿过去】：
         相切点处走线要从圆 1 跨到圆 2，两个圆的曲率方向相反，
         只有"先直行、再让 PD 自己去抓下一条线"才能跨过去；
         继续加倍转 = 必然留在原圆上。
       ★这与"直角入口该直行"也是一致的（直角入口同样不需要加倍转）。
       ★牺牲：真正的"葫芦出口该接着转"这个动作交给【出圈右转】专门管
         —— 而当前右转已被按要求关掉，所以不受影响。
       想回退：WIDE_LONG_GO_STRAIGHT 改 0。 */
    e = 0;
    last_e = 0;      /* 清 D 项，防直行瞬间被微分踢一脚（和 on_cross 一致） */
#else
    e = (int8_t)(last_error * 2);
#if USE_GOURD_SM
#if GOURD_USE_FLIP
    if (gourd_flip) e = (int8_t)(-gourd_dir * GOURD_FLIP_ERR);
#endif
#endif
#endif
  }

#if GOURD_ARC_FLIP
  /* ---- ★绕圈脱困：反向打舵（放在整条 if/else 链之后，优先级最高）----
     为什么放在最后：相切点是 C¹ 连续的，IR 那边【看不出任何异常】，
     上面所有基于图案的判定都不会命中 → 必须由陀螺这条线单独接管。 */
  if (s_arc_push)
  {
    e = (int8_t)(s_arc_dir * (int8_t)GOURD_ARC_FLIP_PUSH);
    last_e = 0;      /* 清 D 项，防换向瞬间被微分踢一脚 */
  }
#endif

  /* ==========================================================================
   * ★★★ 2026-09-24 晚【已回退，默认关闭】：正常路径上的相切点翻转覆盖 ★★★
   *
   *  曾经的原因：翻转覆盖只写在两个分支里
   *     ① if (r.active == 0)   短暂丢线
   *     ② else if (wide_long)  宽图案持续过久
   *  而相切点"线一直都在"、图案是"短暂宽" —— 两个分支都不进，
   *  于是 gourd_flip 被置了却从没作用到 e 上（实车："抓到窗口但不往外拐"）。
   *
   *  ★为什么又关掉（实车 2026-09-24 晚）：接上之后
   *     "没有效果，而且头左右晃的更厉害了" ——
   *     · 摆头变厉害：翻转触发门是 ge_in_gourd(IG)，而 IG 本身会误触发
   *       （GOURD_IG_SUM_TH=1000 是陀螺量纲修好【之前】定的）。
   *       以前误触发只是 GW 计错一下（无害）；接到正常路径后，
   *       每次误触发都会给车 15 拍【错误的反向修正】→ 摆头明显加重。
   *     · 葫芦圈没变化：说明它在【真正的相切点】反而没触发。
   *   ⇒ 两头都不对 = 触发时机本身不可靠。在这种前提下继续加码只会越修越坏。
   *
   *  ★想再试：把 GOURD_FLIP_ON_NORMAL 改 1。但【前提】是先把触发时机做可靠
   *    （至少要把 IG 的阈值按修正后的陀螺量纲重新标定），否则就是在放大误触发。
   * ========================================================================== */
#if USE_GOURD_SM
#if GOURD_USE_FLIP
#if GOURD_FLIP_ON_NORMAL
  if ((!on_cross) && gourd_flip) e = (int8_t)(-gourd_dir * GOURD_FLIP_ERR);
#endif
#endif
#endif

  /* ---- PD 差速校正 ---- */
#if USE_D_FILTER
  e = (int8_t)lpf_update(&d_lpf, (float)e);   /* 对误差低通, 压 D 项高频噪声 */
#endif
  int16_t d_term = (int16_t)(e - last_e);
  int16_t corr   = (int16_t)((kp_x10 * e) / 10 + (kd_x10 * d_term) / 10);
  last_e = e;
  if (corr >  99) corr =  99;
  if (corr < -99) corr = -99;

  int16_t m1 = sp + corr;         /* 左轮: 线偏右时加速 */
  int16_t m2 = sp - corr;         /* 右轮: 线偏右时减速 */
  if (m1 >  99) m1 =  99;
  if (m1 < -99) m1 = -99;
  if (m2 >  99) m2 =  99;
  if (m2 < -99) m2 = -99;

#if USE_SPEED_LOOP
  /* ★2026-09-23 修一个会让车"像爬一样"的 bug：目标必须换算成【真正的 RPM】再交给 PID。
     原代码直接把 PWM(0~99) 当目标 RPM 传进去，而实测转速是【几百 RPM】
     （README：RPM ≈ 3.59 × PWM，PWM 70 → ≈250RPM）→
     PID 永远认为"离目标还差 200 多" → 积分顶到上限 → 输出恒为 50。
     换算系数 RPM_PER_PWM_X100 在 app_config.h（标定依据见那里的注释）。 */
  tgt_rpm[0] = (float)((int32_t)m1 * (int32_t)RPM_PER_PWM_X100) / 100.0f;
  tgt_rpm[1] = (float)((int32_t)m2 * (int32_t)RPM_PER_PWM_X100) / 100.0f;
#if SPD_USE_FEEDFORWARD
  /* ★2026-09-24 加【前馈】：把"目标 PWM"直接当 PID 的基础输出，PID 只修误差。
     没有它的时候：起步瞬间 目标=236RPM、实测=0 → P 项 = 0.4×236 ≈ 94
       → 输出被顶到 99 → 两轮满油门窜出去（"莫名其妙猛冲"），等测速追上才回落。
     有了它：静止时 目标=实测 → 输出 = 前馈（= 开环那个 PWM 值）→ 起步平顺，
       而且积分项不必再顶到上限，稳态误差也随之变小。 */
  spd_pid[0].bias = (float)m1;
  spd_pid[1].bias = (float)m2;
#endif
  {
    float dt = (float)CTRL_PERIOD_MS / 1000.0f;
    m1 = (int16_t)pid_update(&spd_pid[0], tgt_rpm[0], (float)speed_get_rpm(MOTOR_LEFT),  dt);
    m2 = (int16_t)pid_update(&spd_pid[1], tgt_rpm[1], (float)speed_get_rpm(MOTOR_RIGHT), dt);
    if (m1 >  99) m1 =  99; if (m1 < -99) m1 = -99;
    if (m2 >  99) m2 =  99; if (m2 < -99) m2 = -99;
  }
#endif

  motor_set_differential(m1, m2);

#if USE_GOURD_SM && GOURD_USE_FLIP
  if (gourd_flip) gourd_flip--;   /* 翻转窗口倒计时 */
#endif
}

void line_follow_stop(void)
{
  motor_stop();
}

uint8_t line_follow_is_lost(void)
{
  return line_lost;
}

uint8_t line_follow_on_cross(void)
{
  return on_cross;
}

uint8_t line_follow_gourd_waves(void)
{
  return gourd_waves;
}

/* ★本窗口里"100Hz 漏看了几拍 1kHz 采样"（遥测 K1=）。
     >0 就说明有东西一闪而过 —— 现在这个数据不只是观测，
     它背后的 maxraw 位图已经真的参与相切点识别了。
     ★由 line_follow_control 里的相切点识别处填充（那里调 line_1k_take 取走窗口）。 */
uint16_t line_follow_1k_missed(void)
{
  return s_1k_missed;
}

/* ★本窗口 1kHz 采样的完整统计（遥测 K1=missed/total 与 MX=maxact/maxraw）。
     传 NULL 就不取那一项。★必须在 line_follow_control 之后读（它负责填充）。 */
void line_follow_1k_stats(uint16_t* missed, uint16_t* total, uint8_t* maxact, uint16_t* maxraw)
{
  if (missed) *missed = s_1k_missed;
  if (total)  *total  = s_1k_total;
  if (maxact) *maxact = s_1k_maxact;
  if (maxraw) *maxraw = s_1k_maxraw;
}

uint8_t line_follow_boost_active(void)
{
  return boost_active;
}

/* ★弯道计数（2026-09-22 他提的方案）—— 这三个无条件提供，遥测/状态机直接用 */
uint16_t line_follow_g7_count(void)
{
  return g7_count;      /* 最右一路已触发次数（实时，跑一圈看它在出口读到多少） */
}

uint8_t line_follow_g7_flag(void)
{
  return g7_flag;       /* 1 = 触发过"出葫芦弯道"（锁存，看到它说明真的触发了） */
}

uint8_t line_follow_turn_left(void)
{
  return g7_turn;       /* 硬转剩余拍数（>0 = 正在硬转） */
}

/* ★十字事件计数（2026-09-22 15:31）—— 跑一圈看 CX= 就知道一趟里被判了几次十字 */
uint16_t line_follow_cross_events(void)
{
  return cross_events;
}

/* ★"宽图案只贴一端 → 拒绝当十字"的次数（证明"出口不再被误当十字"的修复在起作用） */
uint16_t line_follow_branch_events(void)
{
  return branch_events;}

/* ★葫芦圈出口触发过几次（遥测 GE=）—— 跑一圈看它。
     正常应等于"葫芦圈出口出现的次数"；若在右直角弯处乱涨, 说明该开闸门
     （把 app_config.h 的 GOURD_EXIT_GATE_WAVES 从 0 改成 2）。 */
uint16_t line_follow_gourd_exit_events(void)
{
#if USE_GOURD_EXIT
  return ge_events;
#else
  return 0u;
#endif
}

/* ★最边两路"掉路容忍"补过多少次（遥测 EH=）—— 跑一圈看它涨多少，
   涨得多说明"相切处最边传感器熄灭"确实在发生。 */
uint16_t line_follow_edge_hold_cnt(void)
{
#if USE_EDGE_HOLD
  return edge_hold_cnt;
#else
  return 0u;
#endif
}

/* ★当前是否在葫芦圈里（遥测 IG=）—— 出圈闸门与丢线屏蔽都用这个状态 */
uint8_t line_follow_in_gourd(void)
{
#if (USE_GOURD_EXIT || USE_GOURD_LOST_GUARD)
  return ge_in_gourd;
#else
  return 0u;
#endif
}

/* ★★★ 新的一趟开始：给"出圈右转"重新上膛（一次性锁存复位）★★★
     由 car_fsm.c 在 CAR_RUN 入口、紧跟 odom_reset() 之后调用。

     ★为什么需要它（2026-09-23 发现的真实隐患）：
       GOURD_EXIT_ODOM_FROM_START=1（从起步起算）时，判据是
           `odom_distance_mm() >= GOURD_EXIT_ODOM_MM`
       —— **这个条件一旦成立就永远成立**（里程只增不减）。
       而 GOURD_EXIT_ONE_SHOT 的锁存原本会在两处被解开：
         · "离开葫芦圈"（印记被清）
         · "进圈边沿"（ge_in_gourd 0→1）
       两者一撞的后果：车出圈后跑回普通线 → 锁存被解开 →
       里程仍然 ≥ 阈值 → **车又猛地右转一次**，然后无限重复。
       ⇒ 所以那种模式下，锁存【只】在这里解锁（每趟一次）。
       具体位置见本文件里 `#if !(USE_GOURD_EXIT_ODOM && GOURD_EXIT_ODOM_FROM_START)`
       那两处的说明。 */
void line_follow_gourd_rearm(void)
{
#if USE_GOURD_EXIT
  ge_fired = 0u;
#endif
#if USE_START_BLIND
  /* ★每趟起步的瞬间装载"盲直行"：这里正是 FSM 读完秒、清零里程、进 RUN 的那一处
     （car_fsm.c 的 CAR_COUNTDOWN→CAR_RUN），所以不会在读秒期间被消耗掉。 */
  start_blind = (uint16_t)(START_BLIND_MS / CTRL_PERIOD_MS);
#endif
}

/* ★"从进圈那一刻起算"的净位移(mm)（遥测 ODE=）。
     用途：① 标定 GOURD_EXIT_ODOM_MM —— 出圈事件行里会带上这个值，直接抄
           ② 跑车时肉眼确认"是不是快到该转的里程了"
     ★不在圈里时返回 0（ge_in_gourd=0 → 这个数没有意义，不要拿它当里程表用；
       要看绝对里程请用遥测的 OD= / PATH=）。 */
int32_t line_follow_gourd_odom_mm(void)
{
#if USE_GOURD_EXIT
  if (!ge_in_gourd) return 0;
  return odom_distance_mm() - ge_odom_entry;
#else
  return 0;
#endif
}

/* ★上次出口右转【结束时】转了多少度（遥测 GD=）。
     用途：标定 GOURD_EXIT_TURN_DEG。
     跑一次出圈，看 GD 是 90 就对了；小于 90 → 车没转够（会冲出去）；
     大于 90 → 转过头。IMU 没通时这里恒为 0。 */
float line_follow_gourd_exit_last_deg(void)
{
#if USE_GOURD_EXIT
  return ge_last_deg;
#else
  return 0.0f;
#endif
}

void line_follow_init(void)
{
  last_error  = 0;
  last_e      = 0;
  lost_cycles = 0;
  line_lost   = 0;
  wide_cnt    = 0;
  on_cross    = 0;
  wide_long   = 0;
  s_wo_hold   = 0;   /* ★相切点强制直行窗口 */
#if GOURD_ARC_FLIP
  s_arc_mdeg  = 0;   /* ★绕圈脱困：偏航积分/打舵状态 */
  s_arc_push  = 0;
  s_arc_dir   = 0;
#endif
#if USE_STRAIGHT_BOOST
  straight_cycles = 0;   /* 只在提速开着时才存在（见顶部声明的 #if） */
#endif
  boost_active = 0;
  gourd_latch  = 0;
  gourd_waves  = 0;
#if USE_GOURD_SM
  gourd_wrap_evt = 0;   /* ★出圈事件（跟 gourd_waves 同一个开关，声明/复位/使用同进同出） */
  s_ig_leave     = 0;   /* ★"离开确认"计数（IG=0 连续多少拍才清 GW） */
  s_wave_odom    = odom_distance_mm();   /* ★上一个相切点位置（最小间距门槛用） */
#endif
#if GOURD_USE_FLIP
  gourd_flip   = 0;
  gourd_dir    = 1;
#endif
  g7_count   = 0;      /* ★弯道计数（无条件复位，别放进 #if —— 上次编译事故就是这么来的） */
  g7_prev    = 0;
  g7_lock    = 0;
  g7_waitmsg = 0;
  (void)g7_waitmsg;    /* ★USE_G7_TURN=0 时它只被写不被读 → ARMCC #550-D 警告。
                          显式"用"一下消警告, 不影响任何行为。 */
  g7_flag    = 0;
  g7_turn    = 0;
  cross_events  = 0;   /* ★十字事件计数（2026-09-22 15:31） */
  branch_events = 0;
  one_end_prev  = 0;
  det_prev      = 0;
  wide_gap      = 0;   /* ★事件计数防抖（2026-09-22 16:07） */
  wide_armed    = 1;
  /* ★★ 葫芦圈相关状态 —— ⚠️ 这里【必须】和变量的声明条件一致！
     本文件开头（第 48 行附近）就记着上一次"宏开关漏罩 init"的编译事故：
     变量在 #if 里声明，复位却写在 #if 外面 → 一关开关就 24 个"未定义"错误。
     （2026-09-23 我把这几个开关全关掉时，又踩了一次，就是这条修好的。） */
#if (EDGE_HOLD_CONFIRM > 0)
  ge_dwell_l    = 0;
  ge_dwell_r    = 0;
  edge_rej_cnt  = 0;
#endif
  edge_hold_cnt = 0;
#if (USE_GOURD_EXIT || USE_GOURD_LOST_GUARD)
#if GOURD_EXIT_USE_PATTERN
  ge_exit_cnt   = 0;
  ge_win        = 0;
#endif
  ge_cool       = 0;
  ge_events     = 0;
  ge_in_gourd   = 0;
  ge_mark_ttl   = 0;
  ge_fired      = 0;
  ge_plain_cnt  = 0;
  ge_confirmed  = 0;   /* ★CARD-003：四个新状态与 ge_in_gourd 同处复位 */
  ge_done       = 0;
  ge_straight   = 0;
  ge_stay       = 0;
#if USE_GOURD_SLOWDOWN
  ge_frames     = 0;
#endif
#endif
#if GE_GZ_READY
  {
    uint8_t k;
    for (k = 0u; k < (uint8_t)GOURD_IG_WINDOW; k++) ge_gz_ring[k] = 0u;
    ge_gz_idx  = 0u;
    ge_gz_fill = 0u;
    ge_gz_sum  = 0u;
  }
#endif
#if USE_GOURD_EXIT
  ge_turning     = 0;
  ge_settle      = 0;                    /* ★"停一停"计数 */
  ge_turn_dist   = 0;
  ge_turn_frames = 0;
  ge_moved       = 0;
  ge_encL        = encoder_get_count(MOTOR_LEFT);
  ge_encR        = encoder_get_count(MOTOR_RIGHT);
  ge_turn_deg    = 0.0f;
  ge_last_deg    = 0.0f;
  ge_odom_entry  = odom_distance_mm();   /* ★里程判据的 0 点：以"初始化那一刻"为基准 */
#endif
#if USE_D_FILTER
  lpf_init(&d_lpf, D_FILTER_ALPHA);
#endif
#if USE_SPEED_LOOP
  pid_init(&spd_pid[0], SPD_PID_KP, SPD_PID_KI, SPD_PID_KD, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, SPD_PID_IMAX);
  pid_init(&spd_pid[1], SPD_PID_KP, SPD_PID_KI, SPD_PID_KD, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, SPD_PID_IMAX);
#endif
}
