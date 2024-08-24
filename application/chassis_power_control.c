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
// #include "referee.h"
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
    // static fp32 total_current_limit = 0;
    // if (toe_is_error(REFEREE_TOE) || (get_robot_id() == 0))
    // {
    //     total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    // }
    // else
    // {
    //     fp32 chassis_power;
    //     fp32 chassis_power_buffer;
    //     fp32 chassis_power_limit;
    //     fp32 cap_voltage = cap_message_rx.cap_message.cap_milivoltage / 1000.0f;
    //     get_chassis_power_data(&chassis_power, &chassis_power_buffer, &chassis_power_limit);

    //     // Not use supercap
    //     if (toe_is_error(SUPCAP_TOE) || (cap_voltage <= SUPCAP_VOLTAGE_LOWER_USE_THRESHOLD))
    //     {
    //         // Avoid windup after power being limited
    //         // @TODO: fix unsynchronization issue of M3508 speed after power limiter is triggered
    //         if (chassis_move.motor_speed_pid[0].max_iout != 0)
    //         {
    //             for (uint8_t i = 0; i < 4; i++)
    //             {
    //                 chassis_move.motor_speed_pid[i].max_iout = 0;
    //                 chassis_move.motor_speed_pid[i].Kp = M3508_MOTOR_SPEED_PID_KP * 0.8f;
    //             }
    //         }

    //         fp32 raw_current_limit = M3508_POWER_TO_CURRENT(chassis_power_limit / M3508_POWER_CONTROL_DELAY_S);
    //         if (chassis_power_buffer >= WARNING_POWER_BUFF)
    //         {
    //             if ((chassis_power <= (chassis_power_limit * WARNING_POWER_RATIO)) || (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT))
    //             {
    //                 raw_current_limit += M3508_POWER_TO_CURRENT(chassis_power_buffer / M3508_POWER_CONTROL_SHRINKED_DELAY_S);
    //             }
    //             else
    //             {
    //                 raw_current_limit += M3508_POWER_TO_CURRENT(chassis_power_buffer / M3508_POWER_CONTROL_DELAY_S);
    //             }
    //         }
    //         else
    //         {
    //             raw_current_limit = M3508_POWER_TO_CURRENT(chassis_power_buffer / M3508_POWER_CONTROL_DELAY_S);
    //         }
    //         total_current_limit = first_order_filter(raw_current_limit, total_current_limit, CURRENT_LIMIT_FILTER_COEFF);
    //     }
    //     // Use supercap
    //     else
    //     {
    //         // turn back integral term
    //         if (chassis_move.motor_speed_pid[0].max_iout == 0)
    //         {
    //             for (uint8_t i = 0; i < 4; i++)
    //             {
    //                 chassis_move.motor_speed_pid[i].max_iout = M3508_MOTOR_SPEED_PID_MAX_IOUT;
    //                 chassis_move.motor_speed_pid[i].Kp = M3508_MOTOR_SPEED_PID_KP;
    //             }
    //         }
    //         total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    //     }
    // }

    // fp32 total_current = 0.0f;
    // //calculate the original motor current set
    // for(uint8_t i = 0; i < 4; i++)
    // {
    //     total_current += fabs(chassis_move.motor_speed_pid[i].out);
    // }
    

    // if(total_current > total_current_limit)
    // {
    //     // only limit current of driver motors, because power usage by steering motors is small and difficult to estimate. Because we control voltage input to steering motor (GM6020), not current
    //     fp32 current_scale = total_current_limit / total_current;
    //     chassis_move.motor_speed_pid[0].out*=current_scale;
    //     chassis_move.motor_speed_pid[1].out*=current_scale;
    //     chassis_move.motor_speed_pid[2].out*=current_scale;
    //     chassis_move.motor_speed_pid[3].out*=current_scale;
    // }
}
