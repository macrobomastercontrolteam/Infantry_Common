#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "global_inc.h"

#define FRIC_UP 1300
#define FRIC_DOWN 1320
#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
