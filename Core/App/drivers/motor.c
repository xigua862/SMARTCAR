#include "motor.h"
#include "app_config.h"   /* ★必须有: MOTOR_LEFT_INVERT/MOTOR_RIGHT_INVERT 在这里 */
#include "main.h"
#include "tim.h"        /* htim2 */

/* ★守卫(2026-09-20 血泪): 少了上面的 app_config.h 时, #if MOTOR_LEFT_INVERT 会被当成 0
   → 反转静默失效(编译还不报错, 实测被这坑了一晚上)。留着它, 以后再犯直接编译报错。 */
#ifndef MOTOR_LEFT_INVERT
#error "motor.c 没包含 app_config.h: 电机方向反转宏会失效!"
#endif

/* 单侧电机驱动。逻辑从旧 change1/change2 迁移(含 2026-08-19 电机2方向对调的修复) */
void motor_set_speed(int vol, int16_t speed)
{
  if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
  if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;

  /* ★方向映射开关（见 app_config.h 第四节）：正数 = 前进 */
#if MOTOR_LEFT_INVERT
  if (vol == MOTOR_LEFT)  speed = (int16_t)(-speed);
#endif
#if MOTOR_RIGHT_INVERT
  if (vol == MOTOR_RIGHT) speed = (int16_t)(-speed);
#endif

  if (vol == MOTOR_LEFT)           /* 电机1 = 左轮 */
  {
    if (speed > 0)
    {
      HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed);
    }
    else if (speed < 0)
    {
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -speed);
    }
    else
    {
      HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    }
  }
  else                               /* 电机2 = 右轮 */
  {
    if (speed > 0)
    {
      HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);   /* 实测反转, 电平对调 */
      HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
    }
    else if (speed < 0)
    {
      HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -speed);
    }
    else
    {
      HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }
  }
}

void motor_set_differential(int16_t left, int16_t right)
{
  motor_set_speed(MOTOR_LEFT,  left);
  motor_set_speed(MOTOR_RIGHT, right);
}

void motor_stop(void)
{
  motor_set_differential(0, 0);
}

void motor_init(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  motor_stop();
}
