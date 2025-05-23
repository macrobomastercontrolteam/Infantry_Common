/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. rgb led
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef LED_TRIGGER_TASK_H
#define LED_TRIGGER_TASK_H


#include "global_inc.h"

/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void led_RGB_flow_task(void const * argument);

#endif



