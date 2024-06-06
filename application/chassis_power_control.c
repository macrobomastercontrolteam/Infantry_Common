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

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == INFANTRY_2023_SWERVE)
// @TODO: change limit according to chassis_power_limit field of referee serial data
#define POWER_LIMIT         40.0f
#define WARNING_POWER       35.0f
#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define POWER_LIMIT         95.0f
#define WARNING_POWER       65.0f
#else
#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#endif
#define WARNING_POWER_BUFF  8.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

/**
  * @brief          limit the power, mainly limit driver motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    uint8_t robot_id = get_robot_id();
    if(toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if (toe_is_error(SUPCAP_TOE) || (cap_message_rx.cap_message.cap_voltage <= 10000))
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                //scale down WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                //only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //scale down
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //power > WARNING_POWER
            if(chassis_power > WARNING_POWER)
            {
                fp32 power_scale;
                //power < 80w
                if(chassis_power < POWER_LIMIT)
                {
                    //scale down
                    power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                    
                }
                //power > 80w
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //power < WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }
    else
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }

    
    total_current = 0.0f;
    //calculate the original motor current set
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }
    

    if(total_current > total_current_limit)
    {
        // only limit current of driver motors, because power usage by steering motors is small and difficult to estimate. Because we control voltage input to steering motor (GM6020), not current
        fp32 current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out*=current_scale;
        chassis_power_control->motor_speed_pid[1].out*=current_scale;
        chassis_power_control->motor_speed_pid[2].out*=current_scale;
        chassis_power_control->motor_speed_pid[3].out*=current_scale;
    }
}
