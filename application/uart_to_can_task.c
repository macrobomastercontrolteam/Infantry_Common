/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
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

#include "cmsis_os.h"

#include "uart_to_can_task.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "referee.h"

void uart_to_can_task(void const *pvParameters)
{
	uint32_t ulSystemTime = osKernelSysTick();

	while (1)
	{
		CAN_cmd_ref_uart();
		osDelayUntil(&ulSystemTime, UART_TO_CAN_DELAY_TIME_MS);
	}
}
