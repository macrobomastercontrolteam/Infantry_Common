#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#if defined(INFANTRY_1)
#define FRIC_UP 1300
#define FRIC_DOWN 1320
#elif defined(INFANTRY_2)
// M3508 as friction wheel motor: 1080us-1920us pulse width range corresponds to 0-v_max rpm
// rpm = (Width-1080)/(1920-1080)*v_max; where v_max = 8000rpm by default, Width= 1000000s/us * CCR/ARR / 50Hz = 1000000*CCR/20000/50 = CCR
// Also, rpm = v/radius*60/2pi; where radius = 0.03m, thus, rpm = 1000/pi*vw
// After simplification, CCR=105/pi*v+1080, or v=pi*(CCR-1080)/105
// Test result after linear regression: CCR = 20.45452*v + 1167.47328
// #define FRIC_UP 1470 // test result: fluctuate between 14.0m/s and 14.7m/s
#define FRIC_UP 1430 // test result: fluctuate between 12.5m/s
#define FRIC_DOWN 1648
#elif defined(INFANTRY_3)
// #define FRIC_UP 1310 // test result on INFANTRY_3 (M2305 snail motor): slightly below 15m/s
#define FRIC_UP 1250 // test result on INFANTRY_3 (M2305 snail motor): 12.8m/s to 13.2m/s
#define FRIC_DOWN 1320
#elif defined(INFANTRY_4)
// @TODO
#define FRIC_UP 1250
#define FRIC_DOWN 1320
#elif defined(SENTRY_1)
// sentry limit by rule: 28m/s
// test result: 24.5m/s for safety
#define FRIC_UP 1500
#define FRIC_DOWN 1320
#else
#error "Robot Index not specified"
#endif

#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
