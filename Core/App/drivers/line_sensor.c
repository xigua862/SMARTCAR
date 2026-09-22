#include "line_sensor.h"
#include "app_config.h"     /* LINE_CHANNELS / LINE_CHANNEL_TABLE / LINE_ACTIVE_LEVEL */
#include "main.h"

/* 循迹通道表(接线顺序 左->右)。改 8 路循迹只改 app_config.h 的 LINE_CHANNELS + 表 */
static const line_channel_t kChannels[LINE_CHANNELS] = { LINE_CHANNEL_TABLE };

void line_sensor_init(void)
{
  /* 引脚已在 CubeMX 的 gpio.c 配成 GPIO_MODE_INPUT, 这里无需额外动作。
     若改用 ADC 模拟量, 在此初始化 ADC。 */
}

line_reading_t line_read(void)
{
  line_reading_t r = {0, 0, 0};
  for (uint8_t i = 0; i < LINE_CHANNELS; i++)
  {
    if (HAL_GPIO_ReadPin(kChannels[i].port, kChannels[i].pin) == LINE_ACTIVE_LEVEL)
    {
      r.error += kChannels[i].weight;
      r.active++;
      r.raw |= (uint16_t)(1u << i);
    }
  }
  return r;
}
