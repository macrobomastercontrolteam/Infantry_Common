/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"

#define USART_RX_BUF_LENGTH     512
#define REFEREE_FIFO_BUF_LENGTH 1024

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);

/**
  * @brief          push externally received bytes into the referee FIFO
  * @note           Used when the referee/custom-controller stream is carried over
  *                 a peripheral other than the dedicated referee USART (e.g. USART6
  *                 shared with the VT13 remote). Safe to call from interrupt context.
  * @param[in]      data: pointer to received bytes
  * @param[in]      len: number of bytes
  * @retval         none
  */
extern void referee_push_bytes(uint8_t *data, uint16_t len);
#endif
