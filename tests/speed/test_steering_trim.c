#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "app_config.h"
#include "control/steering_trim.h"

static float magnitude(float value)
{
  return value < 0.0f ? -value : value;
}

static float update(float *state, float demand)
{
  return steering_trim_update(state, demand, CORR_TRIM_UP, CORR_TRIM_DOWN,
                              CORR_TRIM_LEAK, CORR_TRIM_MAX);
}

static void assert_mirror(float left, float right)
{
  assert(magnitude(left + right) < 0.00001f);
  assert(magnitude(left) <= CORR_TRIM_MAX);
  assert(magnitude(right) <= CORR_TRIM_MAX);
}

void test_steering_trim(void)
{
  float positive = 0.0f;
  float negative = 0.0f;
  uint32_t random_state = 0x09290926u;
  unsigned int i;

  /* 恒定左右需求必须从零开始逐拍镜像，并建立非零的稳定记忆。 */
  for (i = 0; i < 500; i++)
  {
    update(&positive, 30.0f);
    update(&negative, -30.0f);
    assert_mirror(positive, negative);
  }
  assert(positive > 1.0f && negative < -1.0f);

  /* 固定种子的随机需求包含换向与零值，检查每拍镜像而非只比较最终值。 */
  positive = 0.0f;
  negative = 0.0f;
  for (i = 0; i < 10000; i++)
  {
    float demand;
    random_state = random_state * 1664525u + 1013904223u;
    demand = (float)((int)((random_state >> 8) % 199u) - 99);
    update(&positive, demand);
    update(&negative, -demand);
    assert_mirror(positive, negative);
  }

  /* 换向后的第一拍应与从零开始的新方向完全相同，不能夹带旧方向记忆。 */
  {
    float from_old = 19.0f;
    float from_zero = 0.0f;
    update(&from_old, -40.0f);
    update(&from_zero, -40.0f);
    assert(from_old < 0.0f && magnitude(from_old - from_zero) < 0.00001f);
    from_old = -19.0f;
    from_zero = 0.0f;
    update(&from_old, 40.0f);
    update(&from_zero, 40.0f);
    assert(from_old > 0.0f && magnitude(from_old - from_zero) < 0.00001f);
  }

  /* 强制直行期间的零需求必须单调释放记忆，泄漏不能跨零制造反向转向。 */
  positive = CORR_TRIM_MAX;
  negative = -CORR_TRIM_MAX;
  for (i = 0; i < 100; i++)
  {
    float previous = positive;
    update(&positive, 0.0f);
    update(&negative, 0.0f);
    assert(positive >= 0.0f && positive <= previous && negative <= 0.0f);
    assert_mirror(positive, negative);
  }
  assert(positive == 0.0f && negative == 0.0f);
  positive = CORR_TRIM_LEAK * 0.25f;
  negative = -positive;
  update(&positive, 0.0f);
  update(&negative, 0.0f);
  assert(positive == 0.0f && negative == 0.0f);

  /* 极大需求必须在两侧都受同一上限约束。 */
  positive = 0.0f;
  negative = 0.0f;
  update(&positive, 100000.0f);
  update(&negative, -100000.0f);
  assert(positive == CORR_TRIM_MAX && negative == -CORR_TRIM_MAX);
  assert_mirror(positive, negative);
  puts("PASS: 5 steering trim regression scenarios (10000 random steps)");
}
