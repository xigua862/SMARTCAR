#ifndef V2_PARAM_H
#define V2_PARAM_H

#include <stdint.h>

/* v2 参数持久化（冻结@M0，实现 M4）：
   C8T6 无硬件 EEPROM -> 片内 Flash 最后 2 页(0x0800FC00/0x0800F800) 双页轮换，
   带 magic + 版本号 + CRC。M0 只留签名与空实现。 */
void    v2_param_init(void);
int32_t v2_param_get(uint16_t id);
void    v2_param_set(uint16_t id, int32_t v);
void    v2_param_save(void);

#endif /* V2_PARAM_H */
