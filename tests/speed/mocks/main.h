#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
/* 实际配置仅声明 GPIO 指针，宿主测试无需实现寄存器。 */
typedef struct test_gpio GPIO_TypeDef;
/* 由测试替身提供可控的毫秒时间戳。 */
uint32_t HAL_GetTick(void);
#endif
