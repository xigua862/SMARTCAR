#include "v2_odom.h"
#include "v2_encoder.h"

#define V2_UM_PER_COUNT  491        /* 491.3 µm/计数（66mm 轮、422 计数/圈） */
#define V2_UM_PER_MM     1000

static int64_t s_um     = 0;
static uint8_t s_inited = 0u;

void v2_odom_init(void)
{
  s_um = 0;
  s_inited = 1u;
  /* 丢弃上电到此刻的增量，避免把静止抖动算进里程 */
  (void)v2_enc_take_delta(V2_ENC_LEFT);
  (void)v2_enc_take_delta(V2_ENC_RIGHT);
}

void v2_odom_reset(void)
{
  v2_odom_init();
}

void v2_odom_tick(void)
{
  int32_t d_l, d_r;
  int64_t a_l, a_r;

  if (!s_inited) { v2_odom_init(); return; }

  d_l = v2_enc_take_delta(V2_ENC_LEFT);
  d_r = v2_enc_take_delta(V2_ENC_RIGHT);

  a_l = (d_l < 0) ? -(int64_t)d_l : (int64_t)d_l;
  a_r = (d_r < 0) ? -(int64_t)d_r : (int64_t)d_r;

  s_um += (a_l + a_r) * (int64_t)V2_UM_PER_COUNT / 2;
}

int32_t v2_odom_mm(void)
{
  return (int32_t)(s_um / V2_UM_PER_MM);
}
