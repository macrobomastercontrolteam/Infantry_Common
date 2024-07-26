/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "global_inc.h"
#include "chassis_task.h"

typedef enum
{
  CHASSIS_ZERO_FORCE,         // chassis will be like no power
  CHASSIS_BASIC_FPV_MODE,     // chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
  CHASSIS_FOLLOW_GIMBAL_MODE, // chassis will follow gimbal, usually in infantry
  CHASSIS_SPINNING_MODE,
  CHASSIS_CV_CONTROL_MODE, // Autonomous chassis control by Computer Vision; when enemy detected spin faster
} chassis_behaviour_e;

/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @retval         none
  */
extern void chassis_behaviour_set_mode(void);

/**
  * @brief          set control set-point. three movement param, according to difference control mode,
  *                 will control corresponding movement.in the function, usually call different control function.
  * @param[out]     vx_set, usually controls vertical speed.
  * @param[out]     vy_set, usually controls horizotal speed.
  * @param[out]     wz_set, usually controls rotation speed.
  * @retval         none
  */
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, fp32 *angle_set);
extern void chassis_behaviour_change_transit(void);

extern chassis_behaviour_e chassis_behaviour_mode;

#endif
