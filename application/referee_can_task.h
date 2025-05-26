#ifndef REFEREE_CAN_TASK_H
#define REFEREE_CAN_TASK_H
#include "CAN_receive.h"
#include "user_lib.h"

#define UART_TO_CAN_DELAY_TIME_MS 10.0f
#define UART_TO_CAN_DELAY_TIME_S (UART_TO_CAN_DELAY_TIME_MS / 1000.0f)

extern void referee_can_task(void const *pvParameters);

#endif
