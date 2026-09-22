/* ============================================================================
 * i2c.c —— I2C2 初始化（MPU6050 挂在 PB10=SCL / PB11=SDA）
 *  ★2026-09-22 手写：CubeMX 对"PB10/PB11 + I2C"的判定一直失败（试过 I2C1/I2C2、
 *    补 Mode 键都不认），而且文档里长期写的"I2C1=PB10/PB11"本身就是错的
 *    —— **PB10/PB11 上只能是 I2C2**（I2C1 在 PB6/PB7，已被编码器占）。
 *    所以这一份由我自己维护，不依赖 CubeMX；调用点放在 main.c 的 USER CODE 区。
 *  ⚠️ 若以后用 CubeMX 重新生成，本文件不会被删（不归它管），
 *     但 stm32f1xx_hal_conf.h 里的 HAL_I2C_MODULE_ENABLED 会被改回去 → 需手动放开。
 * ==========================================================================*/
#include "i2c.h"

I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void)
{
  hi2c2.Instance             = I2C2;
  hi2c2.Init.ClockSpeed      = 400000;                    /* 400kHz 快速模式 */
  hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1     = 0;
  hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2     = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (i2cHandle->Instance == I2C2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB10 -> I2C2_SCL, PB11 -> I2C2_SDA（复用开漏 + 内部上拉） */
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C2_CLK_ENABLE();
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  if (i2cHandle->Instance == I2C2)
  {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
  }
}
