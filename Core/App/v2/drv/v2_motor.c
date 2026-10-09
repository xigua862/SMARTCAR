#include "v2_motor.h"
#include "main.h"
#include "tim.h"

/* 方向电平逻辑从旧 motor.c 迁移（含 2026-08-19 电机2方向对调、2026-09-23 KP 定稿那轮）。
   反转开关沿用实测定稿值：左=1 / 右=1（旧 app_config.h MOTOR_LEFT/RIGHT_INVERT）。 */
#define V2_MOTOR_LEFT_INVERT   1
#define V2_MOTOR_RIGHT_INVERT  1

void v2_motor_set(int ch, int16_t speed)
{
  if (speed >  V2_MOTOR_MAX) speed =  V2_MOTOR_MAX;
  if (speed < -V2_MOTOR_MAX) speed = -V2_MOTOR_MAX;

#if V2_MOTOR_LEFT_INVERT
  if (ch == V2_MOTOR_LEFT)  speed = (int16_t)(-speed);
#endif
#if V2_MOTOR_RIGHT_INVERT
  if (ch == V2_MOTOR_RIGHT) speed = (int16_t)(-speed);
#endif

  if (ch == V2_MOTOR_LEFT)
  {
    if (speed > 0)
    {
      HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)speed);
    }
    else if (speed < 0)
    {
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(-speed));
    }
    else
    {
      HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0u);
    }
  }
  else
  {
    if (speed > 0)
    {
      HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)speed);
    }
    else if (speed < 0)
    {
      HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(-speed));
    }
    else
    {
      HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0u);
    }
  }
}

void v2_motor_stop(void)
{
  v2_motor_set(V2_MOTOR_LEFT, 0);
  v2_motor_set(V2_MOTOR_RIGHT, 0);
}

void v2_motor_init(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  v2_motor_stop();
}
