#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* 顶层循迹控制: P/D 差速 + 速度自适应 + 丢线旋转 + 十字强制直行。
   由 car_fsm 在 CAR_RUN 状态每 CTRL_PERIOD_MS 调用一次。 */
void line_follow_init(void);
void line_follow_control(int16_t base);
void line_follow_stop(void);

/* 状态查询(给状态机做指示灯/终点判定用) */
uint8_t line_follow_is_lost(void);    /* 1 = 持续丢线(正在原地找线) */
uint8_t line_follow_on_cross(void);   /* 1 = 当前判定为十字(强制直行中) */

uint8_t line_follow_gourd_waves(void);   /* 葫芦弯已识别的相切点个数(0~2, 到3清零) */
uint16_t line_follow_1k_missed(void);    /* ★本窗口 100Hz 漏看了几拍 1kHz 采样(遥测 K1=) */
/* ★本窗口 1kHz 采样的完整统计（遥测 K1=missed/total 与 MX=maxact/maxraw）。
   ★必须读这个、不要再直接调 line_1k_take —— 那个会清零窗口，
     相切点识别已经取走了 maxact/maxraw，两边各取一次谁都看不全。 */
void line_follow_1k_stats(uint16_t* missed, uint16_t* total, uint8_t* maxact, uint16_t* maxraw);
uint8_t line_follow_boost_active(void);  /* 1 = 直线提速档生效中 */

/* ★弯道计数 → 强制右转（2026-09-22 他提的方案） */
uint16_t line_follow_g7_count(void);     /* 最右一路(bit7)已触发次数(遥测 G7=, 边沿计数) */
uint8_t  line_follow_g7_flag(void);      /* 1 = 已触发过"出葫芦弯道"(锁存, 遥测 GX=) */
uint8_t  line_follow_turn_left(void);    /* 硬转剩余拍数(>0 = 正在硬转, 遥测 GR=) */

/* ★十字事件计数（2026-09-22 15:31 修"葫芦出口被误当十字"时加的） */
uint16_t line_follow_cross_events(void);  /* 一趟里"判成十字"的次数(遥测 CX=) */
uint16_t line_follow_branch_events(void); /* 一趟里"宽图案只贴一端→拒绝当十字"的次数(遥测 BR=) */

/* ★葫芦圈出口（2026-09-23）：'只有最左 N 路亮'→右转出圈；这个是触发次数(遥测 GE=) */
uint16_t line_follow_gourd_exit_events(void);
/* ★上次出口右转实际转了多少度(遥测 GD=) —— 用来标定 GOURD_EXIT_TURN_DEG。
   IMU 没通时恒为 0。 */
float    line_follow_gourd_exit_last_deg(void);
/* ★当前是否在葫芦圈里(遥测 GD 旁边用来核对) —— 出圈闸门与丢线屏蔽都用它 */
uint8_t  line_follow_in_gourd(void);
/* ★2026-09-26 晚：绕圈脱困的可观测性（打进遥测 AR=/AN=）
   AR = 同号偏航累计（毫度，阈值见 GOURD_ARC_FLIP_MDEG）
   AN = 脱困触发次数（只增不减；恒为 0 = 机制从未跑起来） */
int32_t  line_follow_arc_mdeg(void);
uint16_t line_follow_arc_trigs(void);

/* ★★★ 2026-09-26 晚【控制器决策量的黑匣子接口】★★★
   黑匣子原来只记 IR/RPM/PWM/GZ（结果）。实车"一直在绕圈"时无法判断是
   "跟不住线"还是"在丢线原地旋转" —— 两者修法完全不同。
   这里把每拍的关键决策量交给 telemetry_trace_tick 记录：
     e    = 交给 PD 的误差        corr = 算出的差速修正
     sp   = 速度基准              dcy  = 丢线计数（>LOST_SPIN_DELAY = 正在原地旋转）
     flg  = bit0 丢线 / bit1 在葫芦圈 / bit2 相切点直行 */
void line_follow_dbg_set(int8_t e, int16_t corr, int16_t sp, uint16_t dcy, uint8_t flg);
void line_follow_dbg_get(int8_t *e, int16_t *corr, int16_t *sp, uint16_t *dcy, uint8_t *flg);
/* ★★ 新的一趟开始：给"出圈右转"重新上膛（一次性锁存复位）。
   car_fsm.c 在 CAR_RUN 入口、紧跟 odom_reset() 之后调用。
   ★必须在每趟起步时调 —— 否则第二趟就不会再右转了。 */
void     line_follow_gourd_rearm(void);
/* ★"从进圈那一刻起算"的净位移(mm)（遥测 ODE=）—— 标定 GOURD_EXIT_ODOM_MM 用。
   不在圈里时返回 0。 */
int32_t  line_follow_gourd_odom_mm(void);
uint16_t line_follow_edge_hold_cnt(void);  /* 最边两路掉路容忍补过几次(遥测 EH=) */

/* ★★★ 2026-09-26 加的【诊断用】观测口：控制器【实际输出】的 PWM（遥测 PWM=l/r）★★★
   为什么要它：排查"走直线一抽一抽"时，只看 RPM 反推 PWM 会陷入死结 ——
     "电机在带载下不响应" 和 "控制器根本没出力" 两种解释都成立，无法区分。
   按 pid_update 公式手算，目标 132RPM/实测 71RPM 时输出【应当】≥48 PWM（≈266RPM），
   而实测只有 71RPM（≈13 PWM），差了约 35 个 PWM 无法解释。
   打了这个字段就能一刀切开：
     PWM 高(>40) 而转速低 → 电机/机械在带载下不响应（查电机、齿轮箱打滑）
     PWM 低(≈13) 而误差大 → 控制环逻辑问题（查积分是否被顶死在负值）
   ⚠️ 纯观测，不参与控制。诊断完可以留着（开销可忽略）。 */
void line_follow_last_pwm(int16_t* left, int16_t* right);
/* ★2026-09-26 配合开机开环测试：line_follow_control 不参与时（app.c 的开环分支
   直接调 motor_set_differential），把"实际下发的 PWM"告知遥测，
   否则遥测 PWM= 字段会一直显示上一次循迹时的旧值，误导判断。 */
void line_follow_report_pwm(int16_t left, int16_t right);

/* ★★★ 2026-09-26 【闭环测试】：只跑速度 PID，不跑循迹 PD ★★★
   用途：把"速度环"和"循迹环"分开验证。
     · tgt_pwm  = 目标转速档位（换算成 RPM：目标 = tgt_pwm × RPM_PER_PWM_X100/100），两轮同目标
     · max_pwm  = 输出上限（防窜车；正常循迹允许 -99~99）
   把"目标"和"输出上限"分开传，是为了能单独扫目标而不用动上限 ——
   否则一改目标就把输出余量也改了，两个变量纠缠在一起没法判断。
   行为：
     · 积分清零（避免上次运行残留），每拍把积分限幅临时压到 10
     · 头 1500ms 只测速、不下发（让车先自然起转），之后 PID 才接管
   由 app.c 的测试态在 TEST_SPEEDLOOP_MODE=1 时每 10ms 调用一次。 */
void line_follow_test_speedloop(int16_t tgt_pwm, int16_t max_pwm);
/* ★2026-09-26 【诊断摘要】：把测试期的 PWM / 目标转速 / 实测转速统计出来，
   用 4 行短行打印。
   为什么要它：原先靠"轨迹回放"（150+ 行）取数据，但本车串口在丢字符、
   长回放根本抄不全。短行只有 ~50 字符，丢几个字符也能读 —— 而且板子自己算，
   比人眼从 150 行里数可靠得多。
   ★每拍（10ms）调用一次；内部自带"前 1.5 秒起转期不统计"的处理。 */
void line_follow_test_stats_tick(int16_t tgt_pwm);
void line_follow_test_stats_dump(void);

/* ★★★ 2026-09-26 【左右不对称专项测试】：真车同 PWM 直行，对比两轮各走了多少 mm ★★★
   为什么要它：实测数据显示两轮严重不对称（左轮均值 117 / 右轮 155 RPM，
     41% 的样本差 >100 RPM，17% 的拍子"一轮塌到<50、另一轮>100"）——这正是"弯道还抽"的来源。
   但【合成里程 OD】看不出单轮，无法区分：
     ① 左轮机械偏弱（真的转得慢）        → 修硬件（联轴器/齿轮箱/电机）
     ② 转向时内侧轮本来就该慢（正常差速）→ 属正常，别乱改控制
   本测试两轮【同 PWM 直行】、清里程后只比 mm —— 判据干净：
     L 与 R 相差 >10%  → 真不对称，是硬件问题
     L 与 R 相差 <5%   → 轮子没问题，弯道不对称来自控制环/差速分配
   行为：先停 TEST_ASYM_MS(起转前清零+静置)，再跑 TEST_ASYM_MS，然后停车打印结果。
   由 app.c 在 TEST_SPEEDLOOP_MODE=3 时每 10ms 调用一次。 */
void line_follow_test_asym(int16_t pwm);

/* ★★★ 2026-09-26 【手的判别测试】：区分"机械阻力"vs"编码器计数偏少" ★★★
   为什么要它：实测发现 45 PWM 直行时左轮里程比右轮少 11.5%（1854 vs 2095 mm）。
   但"里程偏少"有两个完全不同的来源：
     ① 左轮真的转得少（机械阻力大 / 齿轮箱效率低）→ 修硬件
     ② 左轮【编码器每圈计数】比右轮少 11.5%（尺子不准）→ 两轮实际一样快，只需软件标定
   区分办法：**用手把两个轮子各慢慢转整整一圈**，比较两轮报告的里程。
     两轮都转一圈 => 物理位移完全相同（都是一圈），
     此时若里程仍差 11.5%，那差异只能来自【编码器计数】= 可能 ②。
   ★本车 PC→车 RX 不通，发不了 `O` 指令读里程，所以做成【开机自动】的：
     全程只测速、不动电机。把车架空/拿在手上，按下面提示转轮子。
   行为：上电 → 静置 WAIT → 自动记录 3 秒"左轮基准"→ 提示"转左轮"→
         记录 3 秒 → 提示"转右轮" → 记录 3 秒 → 打印 L/R 各转了多少 mm。
   判据：两段都"慢慢转整整一圈"的前提下
     L ≈ R（差 <5%）  → 编码器一致，11.5% 是【机械】问题
     L 比 R 少 ~11%   → 编码器计数不一致，是【标定/计数】问题
   由 app.c 在 TEST_SPEEDLOOP_MODE=4 时每 10ms 调用一次。 */
void line_follow_test_handturn(void);

/* ★★★ 2026-09-26 【闭环直行 + 左右不对称】：验证 WHEEL_GAIN 补偿效果 ★★★
   为什么要另开一个：模式 3 是【开环】(motor_set_differential 直接给 PWM)，
     走不到 WHEEL_GAIN 所在的代码路径 → 拿它验证补偿是无效的。
     补偿在速度环内部，必须用【闭环】来验证。
   行为：两轮同目标转速、直行（不循迹），清里程后比两轮各走多少 mm。
     pwm      = 目标转速档位（换算成 RPM）
     max_pwm  = 输出上限
   判据：补偿前 45PWM 开环下 左1854/右2095mm（差 11.5%）；
     WHEEL_GAIN_L=1.11 生效后，两轮里程差应【明显缩小】（目标 <5%）。
   由 app.c 在 TEST_SPEEDLOOP_MODE=5 时每 10ms 调用一次。 */
void line_follow_test_asym_cl(int16_t pwm, int16_t max_pwm);
#endif /* LINE_FOLLOW_H */
