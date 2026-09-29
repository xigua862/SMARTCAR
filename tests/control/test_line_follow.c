#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "app_config.h"
#if TEST_LEGACY_TRIM
/* 仅在测试翻译单元内选择回退分支，不修改实际固件配置。 */
#undef USE_SYMMETRIC_CORR_TRIM
#define USE_SYMMETRIC_CORR_TRIM 0
#endif

/* 直接编译实际控制源码，只替换硬件输入和输出，以观察内部状态切换。 */
#include "../../Core/App/control/line_follow.c"

int16_t kp_x10 = (int16_t)(KP * 10);
int16_t kd_x10 = (int16_t)(KD * 10);
int16_t base_speed = BASE_SPEED;
int16_t sp_straight = SPEED_STRAIGHT;
int16_t sp_curve = SPEED_CURVE;
uint8_t test_mode;
uint16_t test_run_ms;
uint16_t loop_dt_ms = CTRL_PERIOD_MS;
UART_HandleTypeDef huart1;

static line_reading_t input_line;
static int16_t input_rpm[2];
static float input_gyro;
static int16_t motor_pwm[2];

line_reading_t line_read(void) { return input_line; }
uint16_t line_1k_take(uint16_t *total, uint8_t *active, uint16_t *raw)
{
  if (total) *total = 10;
  if (active) *active = input_line.active;
  if (raw) *raw = input_line.raw;
  return 0;
}
int16_t speed_get_rpm(int side) { return input_rpm[side]; }
uint32_t speed_period_ms(void) { return CTRL_PERIOD_MS; }
uint8_t speed_feedback_valid(void) { return 1; }
float imu_get_gyro_z(void) { return input_gyro; }
float imu_get_yaw(void) { return 0.0f; }
void imu_yaw_reset(void) { }
uint8_t imu_yaw_is_valid(void) { return 0; }
int32_t encoder_get_count(int side) { (void)side; return 0; }
int32_t odom_distance_mm(void) { return 0; }
int32_t odom_left_mm(void) { return 0; }
int32_t odom_right_mm(void) { return 0; }
void odom_reset(void) { }
void motor_set_differential(int16_t left, int16_t right)
{
  motor_pwm[0] = left;
  motor_pwm[1] = right;
}
void motor_stop(void) { motor_set_differential(0, 0); }
void telemetry_msg(const char *message) { (void)message; }
void telemetry_trace_reset(void) { }
int HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *bytes,
                      uint16_t count, uint32_t timeout)
{
  (void)uart; (void)bytes; (void)count; (void)timeout;
  return 0;
}

/* 从实际权重生成一致的输入，不手工伪造 error/raw 不匹配的样本。 */
static void set_line(uint8_t raw)
{
  static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
  unsigned i;
  input_line.raw = raw;
  input_line.active = 0;
  input_line.error = 0;
  for (i = 0; i < 8; ++i)
  {
    if (raw & (1u << i))
    {
      ++input_line.active;
      input_line.error += weights[i];
    }
  }
}

static void restart(void)
{
  input_rpm[0] = input_rpm[1] = 0;
  input_gyro = 0.0f;
  motor_pwm[0] = motor_pwm[1] = 0;
  set_line(0x18);
  line_follow_init();
}

static void tick(uint8_t raw)
{
  set_line(raw);
  line_follow_control(base_speed);
}

static void seed_old_history(void)
{
  last_e = 12;
  d_lpf.y = 12.0f;
  s_corr_trim = 20.0f;
  spd_pid[0].acc = spd_pid[1].acc = -25.0f;
  spd_pid[0].prev_err = spd_pid[1].prev_err = 1000.0f;
  spd_pid[0].first = spd_pid[1].first = 0;
}

static void test_one_missing_frame_preserves_steering(void)
{
  restart();
  tick(0x80);
  tick(0x00);
  assert(lost_cycles == 1);
  assert(!line_lost);
  tick(0x80);
  assert(lost_cycles == 0);
  assert(lost_settle == 0);
  assert(dbg_e > LOST_SETTLE_ERR);
  puts("PASS: one missing frame does not suppress recovered edge steering");
}

static void test_spin_and_reacquisition_clear_history(void)
{
  restart();
  tick(0xC0);
  tick(0x00);
  tick(0x00);
  seed_old_history();
  tick(0x00);
  assert(line_lost && lost_cycles == 3);
  assert(motor_pwm[0] == LOST_SPIN_SPEED);
  assert(motor_pwm[1] == -LOST_SPIN_SPEED);
  assert(pwm_out[0] == motor_pwm[0] && pwm_out[1] == motor_pwm[1]);
  assert(last_e == 0 && d_lpf.y == 0.0f && s_corr_trim == 0.0f);
  assert(spd_pid[0].acc == 0.0f && spd_pid[1].acc == 0.0f);
  assert(spd_pid[0].first && spd_pid[1].first);

  /* 故意再次污染历史，验证找回线的分支自身也会复位。 */
  seed_old_history();
  tick(0x18);
  assert(!line_lost && lost_cycles == 0);
  assert(lost_settle == LOST_SETTLE_FRAMES - 1);
  assert(last_e == 0 && d_lpf.y == 0.0f && s_corr_trim == 0.0f);
  assert(dbg_e == 0 && dbg_corr == 0);
  assert(spd_pid[0].acc == SPD_PID_IMAX);
  assert(spd_pid[1].acc == SPD_PID_IMAX);
  puts("PASS: spin and reacquisition discard stale steering and speed history");
}

static void test_crossing_clears_stale_steering(void)
{
  restart();
  seed_old_history();
  wide_cnt = CROSS_CONFIRM_CNT - 1;
  tick(0xFF);
  assert(on_cross);
  assert(d_lpf.y == 0.0f && s_corr_trim == 0.0f);
  assert(last_e == 0 && dbg_e == 0 && dbg_corr == 0);
  puts("PASS: forced straight crossing clears filter and trim history");
}

static void test_gourd_leave_requires_consecutive_frames(void)
{
  unsigned i;
  restart();
  ge_in_gourd = 1;
  ge_mark_ttl = GOURD_ENTRY_MARK_TTL;
  ge_plain_cnt = GOURD_ENTRY_CLEAR_FRAMES - 1;
  tick(0x67);
  assert(ge_in_gourd && ge_plain_cnt == 0);
  for (i = 1; i < GOURD_ENTRY_CLEAR_FRAMES; ++i)
  {
    tick(0x18);
    assert(ge_in_gourd);
    assert(ge_plain_cnt == i);
  }
  tick(0x18);
  assert(!ge_in_gourd && ge_plain_cnt == 0);
  puts("PASS: renewed gourd evidence restarts consecutive leave confirmation");
}

static void test_stop_updates_motor_and_telemetry(void)
{
  restart();
  tick(0x80);
  assert(motor_pwm[0] != 0 || motor_pwm[1] != 0);
  seed_old_history();
  line_follow_stop();
  assert(motor_pwm[0] == 0 && motor_pwm[1] == 0);
  assert(pwm_out[0] == 0 && pwm_out[1] == 0);
  assert(dbg_e == 0 && dbg_corr == 0 && dbg_sp == 0 && dbg_flg == 0);
  assert(last_e == 0 && d_lpf.y == 0.0f && s_corr_trim == 0.0f);
  assert(spd_pid[0].first && spd_pid[1].first);
  puts("PASS: stop clears motor output, reported PWM and control history");
}

int main(void)
{
#if USE_SYMMETRIC_CORR_TRIM
  puts("Configuration: symmetric steering trim");
#else
  puts("Configuration: legacy steering trim fallback");
#endif
  test_one_missing_frame_preserves_steering();
  test_spin_and_reacquisition_clear_history();
  test_crossing_clears_stale_steering();
  test_gourd_leave_requires_consecutive_frames();
  test_stop_updates_motor_and_telemetry();
  puts("5 control regression cases passed (host simulation, not a track trial)");
  return 0;
}
