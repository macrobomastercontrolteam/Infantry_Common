#ifndef BSP_PWM_H
#define BSP_PWM_H
#include "global_inc.h"

extern void servo_pwm_set(uint16_t pwm, uint8_t i);
extern void pump_pwm_control(fp32 Duty_cycle);
extern void pump_max(void);
extern void pump_stop(void);
#endif
