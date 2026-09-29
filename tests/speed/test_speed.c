#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "speed.h"
void test_steering_trim(void);
static uint32_t tick;
static uint16_t counts[2];
uint32_t HAL_GetTick(void) { return tick; }
void encoder_init(void) { }
int32_t encoder_get_count(int vol) { return counts[vol]; }
static void reset(uint32_t now, uint16_t left, uint16_t right)
{
  tick=now; counts[0]=left; counts[1]=right; speed_init();
  assert(speed_period_ms()==0 && speed_feedback_valid()==0);
  assert(speed_get_rpm(0)==0 && speed_get_rpm(1)==0);
}
static void step(uint32_t dt, uint16_t left, uint16_t right)
{
  tick+=dt; counts[0]=left; counts[1]=right; speed_update();
}
int main(void)
{
  /* 非零初值不会被计入首个窗口，正反方向的幅值一致。 */
  reset(100,60000,12345); step(10,60010,12335);
  assert(speed_feedback_valid() && speed_period_ms()==10);
  assert(speed_get_rpm(0)==142 && speed_get_rpm(1)==142);
  /* 两种跨零方向都保留真实小增量；变化率限制仍为原值。 */
  reset(0,65530,5); step(10,5,65530);
  assert(speed_feedback_valid() && speed_get_rpm(0)==150 && speed_get_rpm(1)==150);
  step(10,16,65519);
  assert(speed_get_rpm(0)==156 && speed_get_rpm(1)==156);
  /* 同毫秒调用不消耗计数或刷新有效性。 */
  reset(0,100,100); step(0,110,90);
  assert(!speed_feedback_valid() && speed_period_ms()==0);
  step(10,110,90);
  assert(speed_get_rpm(0)==142 && speed_get_rpm(1)==142);
  /* 30ms 内正常超过旧70计数门限的采样仍有效。 */
  reset(0,0,1000); step(30,180,820);
  assert(speed_feedback_valid() && speed_period_ms()==30);
  assert(speed_get_rpm(0)==150 && speed_get_rpm(1)==150);
  /* 异常采样保留显示值并报无效，下一窗口从新计数基准恢复。 */
  step(10,680,320); assert(!speed_feedback_valid() && speed_get_rpm(0)==150);
  step(10,690,310); assert(speed_feedback_valid() && speed_get_rpm(0)==142);
  /* 超时反馈失效，长窗口不生成伪造的新转速。 */
  tick+=101; assert(!speed_feedback_valid()); speed_update();
  assert(!speed_feedback_valid() && speed_period_ms()==101 && speed_get_rpm(0)==142);
  step(10,700,300); assert(speed_feedback_valid());
  /* 毫秒时间戳回绕按无符号时间差计算。 */
  reset(UINT32_MAX-5,200,200); step(10,210,190);
  assert(speed_period_ms()==10 && speed_feedback_valid() && speed_get_rpm(0)==142);
  /* 计数可信度门限含两个量化计数的裕度。 */
  reset(0,0,0); step(10,72,72); assert(speed_feedback_valid());
  step(10,145,145); assert(!speed_feedback_valid());
  assert(speed_get_rpm(9)==0 && speed_get_count(9)==0);
  puts("PASS: 9 speed feedback regression scenarios");
  test_steering_trim();
  return 0;
}
