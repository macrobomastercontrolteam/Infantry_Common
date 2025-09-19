/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control
  * @note       this is only controling 80w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (including max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "chassis_task.h"

// #define WARNING_POWER_RATIO 0.7f
// #define POWER_BUFF_TOTAL 60.0f
// #if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
// #define WARNING_POWER_BUFF (POWER_BUFF_TOTAL / 2.0f)
// #else
// #define WARNING_POWER_BUFF (POWER_BUFF_TOTAL / 3.0f)
// #endif
// #define NO_JUDGE_TOTAL_CURRENT_LIMIT (MAX_3508_MOTOR_CAN_CURRENT * 4.0f)

// #define M3508_CURRENT_TO_POWER_RATIO 0.025f // power is approx 25V * 1/1000 mA
// #define M3508_CURRENT_TO_POWER(_current) ((_current) * M3508_CURRENT_TO_POWER_RATIO)
// #define M3508_POWER_TO_CURRENT(_power) ((_power) / M3508_CURRENT_TO_POWER_RATIO)
// // assumption of time that power stay unchanged despite command changes
// #define M3508_POWER_CONTROL_DELAY_S (0.1f)
// #define M3508_POWER_CONTROL_SHRINKED_DELAY_S (M3508_POWER_CONTROL_DELAY_S / 2.0f)
// #define CURRENT_LIMIT_FILTER_COEFF 0.3f

/**
  * @brief          limit the power, mainly limit driver motor current
  * @retval         none
  */
void chassis_power_control(void)
{
   
}
