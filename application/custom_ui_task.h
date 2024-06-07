/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       custom_ui_task.c/h
  * @brief      custom  task,
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CUSTOM_UI_TASK_H
#define CUSTOM_UI_TASK_H
// #include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

#define CUSTOM_UI_TIME_MS 10.0f

typedef enum
{
	ARMOR_ZERO,
	ARMOR_ONE,
	ARMOR_TWO,
	ARMOR_THREE,
	NONE,
} armor_damage_info;
/**
 * @brief          client ui drawing
 * @param[in]      pvParameters: null
 * @retval         none
 */

extern void custom_ui_task(void const *argument);
void chassis_direction_draw(float yaw_relative_angle);
void gimbal_pitch_direction_draw(float pitch_relative_angle);
armor_damage_info armor_damage_judge(void);
void armor_damage_draw(float yaw_relative_angle);
void super_cap_status_draw(void);
void static_elements_init(void);
void chassis_mode(void);
void trigger_motor_state(float trigger_rpm);

#endif
