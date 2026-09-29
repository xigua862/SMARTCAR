#ifndef TEST_ENCODER_H
#define TEST_ENCODER_H
#include <stdint.h>
/* 由测试替身提供可控的编码器计数。 */
void encoder_init(void);
int32_t encoder_get_count(int vol);
#endif
