#ifndef BSP_SERVO_PWM_H
#define BSP_SERVO_PWM_H
#include "global_inc.h"

extern void servo_pwm_set(uint16_t pwm);
extern void servo_open(void);
extern void servo_close(void);
#endif
