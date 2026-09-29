#ifndef CONTROL_TEST_USART_H
#define CONTROL_TEST_USART_H

#include <stdint.h>

typedef struct { uint32_t unused; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
int HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *bytes,
                      uint16_t count, uint32_t timeout);

#endif
