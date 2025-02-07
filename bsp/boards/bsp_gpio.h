#ifndef BSP_GPIO_H
#define BSP_GPIO_H
#include "gpio.h"
#include "main.h"
#include "global_inc.h"

extern void pump_control(uint8_t on);
extern void holder_control(uint8_t holder,uint8_t state);

extern bool_t gpio_cmd_pitch_forward(void);
extern bool_t gpio_cmd_pitch_backward(void);
#endif
