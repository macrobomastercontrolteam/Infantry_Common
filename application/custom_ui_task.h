/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       custom_ui_task.c/h
  * @brief      custom  task,
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
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
  //uint8_t chassis_state;
  uint8_t spinning_state;

  uint8_t trigger_state;
  uint8_t firc_state;
  uint8_t auto_aim_state;

  uint8_t supercap_persentage;
#if (LAUNCHER_TYPE == LAUNCHER_42MM)
  uint8_t Limit_Ignored;

  uint8_t Launcher_Loaded;
  uint8_t Launcher_Opened;
#endif

} ui_info_t;


extern ui_info_t ui_info;

#endif /* _CUSTOM_UI_TASK_H */
