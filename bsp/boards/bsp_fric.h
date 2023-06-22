#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#if defined(INFANTRY_2)
// M3508 as friction wheel motor: 1080us-1920us pulse width range corresponds to 0-v_max rpm
// rpm = (Width-1080)/(1920-1080)*v_max; where v_max = 8000rpm by default, Width= 1000000s/us * CCR/ARR / 50Hz = 1000000*CCR/20000/50 = CCR
// Also, rpm = v/radius*60/2pi; where radius = 0.03m, thus, rpm = 1000/pi*v
// After simplification, CCR=105/pi*v+1080, or v=pi*(CCR-1080)/105
#define FRIC_UP 1648 // 5400rpm, 17m/s
// #define FRIC_UP 1615 // 5093rpm, 16m/s
#define FRIC_DOWN 1648
#elif defined(INFANTRY_1) || defined(INFANTRY_3) || defined(SENTRY_1)
#define FRIC_UP 1400
#define FRIC_DOWN 1320
#endif
#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
