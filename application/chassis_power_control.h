/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "main.h"
// #include "math.h" // sqrtf

// @TODO: current lower voltage limit value is for a potentially broken supcap manager and/or capacitor, lower it if yours is working by stress testing. Lower limit of a working setup can be around 8V
#define SUPCAP_VOLTAGE_LOWER 25.2f
#define SUPCAP_VOLTAGE_UPPER 26.697f
// #define SUPCAP_ENERGY_CLEARANCE_RATIO 0.1f
// #define SUPCAP_VOLTAGE_LOWER_USE_THRESHOLD (sqrtf((1 - SUPCAP_ENERGY_CLEARANCE_RATIO) * SUPCAP_VOLTAGE_UPPER * SUPCAP_VOLTAGE_UPPER + SUPCAP_ENERGY_CLEARANCE_RATIO * SUPCAP_VOLTAGE_LOWER * SUPCAP_VOLTAGE_LOWER))
#define SUPCAP_VOLTAGE_LOWER_USE_THRESHOLD SUPCAP_VOLTAGE_LOWER
#define SUPCAP_STOP_USE_THRESHOLD_BY_BUFFER 15.0f
#define SUPCAP_CAPACITANCE 5.0f // unit: Farad

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
extern void chassis_power_control(chassis_move_t *chassis_power_control);

#endif
