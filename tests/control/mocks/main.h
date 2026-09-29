#ifndef CONTROL_TEST_MAIN_H
#define CONTROL_TEST_MAIN_H

#include <stdint.h>

/* 宿主测试只需要类型声明，不访问 STM32 外设寄存器。 */
typedef struct { uint32_t unused; } GPIO_TypeDef;

#endif
