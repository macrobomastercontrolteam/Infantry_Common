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
#ifndef _CUSTOM_UI_TASK_H
#define _CUSTOM_UI_TASK_H

#include "main.h"

extern void custom_ui_task(void const *argument);

typedef struct
{
  uint8_t launcher_flag_byte;
  uint8_t chassis_flag_byte;


//**********gimbal ***********/

  bool_t trigger_state;
  bool_t firc_state;
  bool_t auto_aim_state;

#if (LAUNCHER_TYPE == LAUNCHER_42MM)
  bool_t Heat_Limit_Ignored;

  bool_t Launcher_Loaded;
  bool_t Launcher_Opened;
#endif

//**********chassis ***********/
  bool_t spinning_state;
  bool_t power_saving
  
} ui_info_t;

typedef enum
{
    UI_TRIGGER_STATE_BIT = 1 << 0,
    UI_FRIC_STATE_BIT = 1 << 1,
    UI_AUTO_AIM_STATE_BIT = 1 << 2,
    // CV_MODE_SHOOT_BIT = 1 << 3,
    // CV_MODE_CHASSIS_SPINNING_BIT = 1 << 4,
    UI_IGNORE_HEAT_LIMIT_BIT = 1 << 5,
    UI_LAUNCHER_LOADED_BIT = 1 << 6,
    UI_LAUNCHER_OPENED_BIT = 1 << 7,
} eLauncherStatusBits;

typedef enum
{
    UI_SPINNING_STATE_BIT = 1 << 0,
    UI_POWER_SAVING_BIT = 1 << 1,
    
} eChassisStatusBits;

extern ui_info_t ui_info;

#endif //* _CUSTOM_UI_TASK_H */
