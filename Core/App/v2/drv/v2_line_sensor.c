#include "v2_line_sensor.h"
#include "main.h"

/* ---- 常量写死在本文件顶部（框架：v2 不依赖旧 app_config.h）---- */
#define V2_LINE_CHANNELS      8
#define V2_LINE_ACTIVE_LEVEL  GPIO_PIN_SET   /* 白线 = 高（实测定稿） */
#define V2_LINE_OS_SAMPLES    8              /* 每拍突发采样次数 */
#define V2_LINE_OS_SPACING_US 1              /* 子采样间隔(µs) */

typedef struct { GPIO_TypeDef* port; uint16_t pin; int8_t weight; } v2_line_ch_t;

static const v2_line_ch_t kV2Ch[V2_LINE_CHANNELS] = {
  { GPIOA, GPIO_PIN_0,  -7 },
  { GPIOA, GPIO_PIN_1,  -5 },
  { GPIOA, GPIO_PIN_8,  -3 },
  { GPIOA, GPIO_PIN_15, -1 },
  { GPIOB, GPIO_PIN_0,  +1 },
  { GPIOB, GPIO_PIN_1,  +3 },
  { GPIOB, GPIO_PIN_12, +5 },
  { GPIOB, GPIO_PIN_13, +7 },
};

/* ---------------- DWT 周期计数器 ---------------- */
static uint8_t  s_dwt_ok    = 0u;
static uint32_t s_us_cycles = 72u;

void v2_dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)
  {
    s_dwt_ok    = 1u;
    s_us_cycles = SystemCoreClock / 1000000u;
    if (s_us_cycles == 0u) s_us_cycles = 1u;
  }
  else
  {
    s_dwt_ok = 0u;
  }
}

uint32_t v2_dwt_cycles(void) { return DWT->CYCCNT; }

void v2_dwt_delay_us(uint16_t us)
{
  uint32_t start, want;
  if ((us == 0u) || (s_dwt_ok == 0u)) return;
  start = DWT->CYCCNT;
  want  = (uint32_t)us * s_us_cycles;
  while ((uint32_t)(DWT->CYCCNT - start) < want) { }
}

/* ---------------- 采样 ---------------- */
uint8_t v2_line_raw_once(void)
{
  uint8_t bm = 0u;
  uint8_t i;
  for (i = 0u; i < V2_LINE_CHANNELS; i++)
  {
    if (HAL_GPIO_ReadPin(kV2Ch[i].port, kV2Ch[i].pin) == V2_LINE_ACTIVE_LEVEL)
    {
      bm |= (uint8_t)(1u << i);
    }
  }
  return bm;
}

static uint8_t v2_line_burst(void)
{
  uint8_t cnt[V2_LINE_CHANNELS];
  uint8_t bm  = 0u;
  uint8_t i, n;

  for (i = 0u; i < V2_LINE_CHANNELS; i++) cnt[i] = 0u;

  for (n = 0u; n < (uint8_t)V2_LINE_OS_SAMPLES; n++)
  {
    uint8_t s = v2_line_raw_once();
    for (i = 0u; i < V2_LINE_CHANNELS; i++)
    {
      if (s & (uint8_t)(1u << i)) cnt[i]++;
    }
    if ((uint8_t)(n + 1u) < (uint8_t)V2_LINE_OS_SAMPLES)
    {
      v2_dwt_delay_us((uint16_t)V2_LINE_OS_SPACING_US);
    }
  }

  /* 多数表决：cnt*2 >= N 才算亮（整数写法，无浮点） */
  for (i = 0u; i < V2_LINE_CHANNELS; i++)
  {
    if ((uint16_t)cnt[i] * 2u >= (uint16_t)V2_LINE_OS_SAMPLES)
    {
      bm |= (uint8_t)(1u << i);
    }
  }
  return bm;
}

void v2_line_init(void)
{
  /* 引脚模式由 CubeMX 的 gpio.c 配成输入，这里不用动 */
  v2_dwt_init();
}

line_reading_t v2_line_read(void)
{
  line_reading_t r;
  int32_t sum_w = 0;
  uint8_t bm, i;

  r.bitmap = 0u;
  r.active = 0u;
  r.err_x100 = 0;

  bm = v2_line_burst();
  r.bitmap = bm;

  for (i = 0u; i < V2_LINE_CHANNELS; i++)
  {
    if (bm & (uint8_t)(1u << i))
    {
      sum_w += (int32_t)kV2Ch[i].weight;
      r.active++;
    }
  }

  /* 差比和（框架 §④）：分母 = 触发路数 + 1（兼除零保护 + 少路数阻尼） */
  r.err_x100 = (int16_t)((100 * sum_w) / ((int32_t)r.active + 1));

  return r;
}
