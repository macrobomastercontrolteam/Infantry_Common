#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#if defined(INFANTRY_1) || defined(INFANTRY_2)
#define FRIC_UP 1235
#define FRIC_DOWN 1164
#elif defined(INFANTRY_3)
#define FRIC_UP 1400
#define FRIC_DOWN 1320
#endif
#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
