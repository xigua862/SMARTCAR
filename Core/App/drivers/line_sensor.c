#include "line_sensor.h"
#include "app_config.h"     /* LINE_CHANNELS / LINE_CHANNEL_TABLE / LINE_ACTIVE_LEVEL */
#include "main.h"

/* 循迹通道表(接线顺序 左->右)。改 8 路循迹只改 app_config.h 的 LINE_CHANNELS + 表 */
static const line_channel_t kChannels[LINE_CHANNELS] = { LINE_CHANNEL_TABLE };

/* ============================================================================
 * ★★ 突发过采样（2026-09-23）：在一拍内连读 N 次再表决，提高有效采样频率。
 *    只改本文件，三个调用点（循迹 1 处 + 遥测 2 处）自动受益。
 *    配置见 app_config.h 的 LINE_OS_*。
 * ==========================================================================*/
#if (LINE_OS_SAMPLES < 1) || (LINE_OS_SAMPLES > 64)
#error "LINE_OS_SAMPLES 必须在 1~64"
#endif

/* ---- DWT 周期计数器：给突发采样提供微秒级时间基准 ----
   Cortex-M3 有 DWT->CYCCNT（72MHz 下 1µs = 72 周期）。
   注意：HAL 不会自动使能它，必须自己置 DEMCR.TRCENA 并清 CYCCNT。 */
static uint8_t  dwt_ok    = 0;
static uint32_t us_cycles = 72u;    /* 1µs 对应的 CPU 周期数，初始化时按 SystemCoreClock 重算 */

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能跟踪/调试模块 */
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* 使能周期计数器 */
  if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)
  {
    dwt_ok    = 1u;
    us_cycles = SystemCoreClock / 1000000u;         /* 72MHz → 72 */
    if (us_cycles == 0u) us_cycles = 1u;
  }
  else
  {
    dwt_ok = 0u;                                     /* 读不到就退化成"背靠背采样" */
  }
}

/* 忙等 us 微秒。DWT 不可用时退化成空操作（= 背靠背采样，仍能工作，只是间距不可控） */
static void dwt_delay_us(uint16_t us)
{
  uint32_t start, want;
  if ((us == 0u) || (dwt_ok == 0u)) return;
  start = DWT->CYCCNT;
  want  = (uint32_t)us * us_cycles;
  /* 用无符号差值比较 → CYCCNT 回绕(约 59.6s)天然安全 */
  while ((uint32_t)(DWT->CYCCNT - start) < want) { /* spin */ }
}

/* 读一次原始位图（不含任何表决） */
static uint16_t line_raw_once(void)
{
  uint16_t bm = 0u;
  uint8_t  i;
  for (i = 0; i < LINE_CHANNELS; i++)
  {
    if (HAL_GPIO_ReadPin(kChannels[i].port, kChannels[i].pin) == LINE_ACTIVE_LEVEL)
    {
      bm |= (uint16_t)(1u << i);
    }
  }
  return bm;
}

#if LINE_OS_ENABLE
/* ★过采样"有没有起作用"的证据计数器（2026-09-23）
   统计：有多少拍，出现了"同一拍内子采样之间位图不一致"。
   为什么它能当证据：
     · 若红外在这几个µs内完全不变 → 所有子采样位图相同 → 计数器不涨 → 过采样白做
     · 只要它 >0，就说明**单次采样会漏掉的东西被快采样抓到了**（这正是加它的目的）
   增量语义：读一次清一次（telemetry 每 50ms 读一行，正好是"这个窗口内涨了多少"）。 */
static uint32_t os_ticks     = 0;   /* 已过采样的拍数 */
static uint32_t os_disagree  = 0;   /* 其中"子采样位图不一致"的拍数 */
static uint32_t os_disagree_acc = 0;/* 累积值（给"读一次清一次"用） */

/* 突发采样 N 次 → 合成一个位图 */
static uint16_t line_raw_burst(void)
{
  uint8_t  cnt[LINE_CHANNELS];
  uint16_t or_all   = 0u;
  uint16_t and_all  = 0xFFFFu;
  uint16_t bm       = 0u;
  uint8_t  i;
  uint8_t  n;

  for (i = 0; i < LINE_CHANNELS; i++) cnt[i] = 0u;

  for (n = 0; n < (uint8_t)LINE_OS_SAMPLES; n++)
  {
    uint16_t s = line_raw_once();
    or_all  |= s;
    and_all &= s;
    for (i = 0; i < LINE_CHANNELS; i++)
    {
      if (s & (uint16_t)(1u << i)) cnt[i]++;
    }
    /* 最后一次之后不用再等 */
    if ((uint8_t)(n + 1u) < (uint8_t)LINE_OS_SAMPLES) dwt_delay_us((uint16_t)LINE_OS_SPACING_US);
  }

  /* ★证据统计：or != and 就说明这几个子采样之间有一个 bit 变过 */
  os_ticks++;
  if (or_all != and_all) { os_disagree++; os_disagree_acc++; }

#if LINE_OS_VOTE_MAJORITY
  /* 多数表决：亮的路数 >= 半数 才算亮。
     用 cnt*2 >= N 的整数写法，避免浮点。
     好处：单次毛刺不会改变图案 → active/dark_groups 与单采样同量级。 */
  for (i = 0; i < LINE_CHANNELS; i++)
  {
    if ((uint16_t)cnt[i] * 2u >= (uint16_t)LINE_OS_SAMPLES)
    {
      bm |= (uint16_t)(1u << i);
    }
  }
  return bm;
#else
  (void)cnt;
  return or_all;      /* 按位或：任一子采样亮就算亮 */
#endif
}
#endif /* LINE_OS_ENABLE */

void line_sensor_init(void)
{
  /* 引脚已在 CubeMX 的 gpio.c 配成 GPIO_MODE_INPUT, 这里无需额外动作。
     若改用 ADC 模拟量, 在此初始化 ADC。 */
  dwt_init();          /* ★给突发过采样准备微秒基准（LINE_OS_ENABLE=0 时也备着，无害） */
}

line_reading_t line_read(void)
{
  line_reading_t r = {0, 0, 0};
  uint16_t bm;
  uint8_t  i;

#if LINE_OS_ENABLE
  bm = line_raw_burst();
#else
  bm = line_raw_once();
#endif

  /* ★error 与 raw 【必须同源】：都由最终位图算出来。
     这样任何"按位图做图案匹配"的判据（十字/出口/葫芦）都不会和 error 打架。
     （如果把 N 次 error 求平均，平均出来的方向可能对应一个不存在的图案。） */
  for (i = 0; i < LINE_CHANNELS; i++)
  {
    if (bm & (uint16_t)(1u << i))
    {
      r.error += kChannels[i].weight;
      r.active++;
    }
  }
  r.raw = bm;
  return r;
}

/* ★过采样证据（读一次清一次）：
     返回窗口内"子采样位图不一致"的拍数。
     >0  = 快采样确实抓到了单次采样会漏掉的东西（过采样在起作用）
     =0  = 这几个 µs 内红外完全没变（要么模块跟不上，要么间距太小）
   LINE_OS_ENABLE=0 时恒返回 0。 */
uint32_t line_os_disagree_take(void)
{
#if LINE_OS_ENABLE
  uint32_t v = os_disagree_acc;
  os_disagree_acc = 0u;
  return v;
#else
  return 0u;
#endif
}

/* ★过采样统计（不清零，累计值）：拍数 / 不一致拍数 */
void line_os_stats(uint32_t* ticks, uint32_t* disagree)
{
#if LINE_OS_ENABLE
  if (ticks)    *ticks    = os_ticks;
  if (disagree) *disagree = os_disagree;
#else
  if (ticks)    *ticks    = 0u;
  if (disagree) *disagree = 0u;
#endif
}
