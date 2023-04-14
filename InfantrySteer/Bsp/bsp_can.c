/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
 
#include "bsp_can.h"

int16_t feedback_ecd1;
int16_t target_ecd1;

/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_CONTROLLER_RX_ID = 0x112,
  CAN_CHASSIS_GM6020_TX_ID = 0x1FF,

  CAN_6020_M1_ID = 0x205,
  CAN_6020_M2_ID = 0x206,
  CAN_6020_M3_ID = 0x207,
  CAN_6020_M4_ID = 0x208,
} can_msg_id_e;

moto_info_t motor_info[STEER_MOTOR_COUNT];

/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @retval None
  */
void can_user_init(void)
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;                // set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);        // init can filter
  HAL_CAN_Start(&hcan1);                          // start can1
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can1 rx interrupt

  // Receive only message from Type C Board
  can_filter.FilterIdHigh  = CAN_CHASSIS_CONTROLLER_RX_ID << 5; // refer to https://schulz-m.github.io/2017/03/23/stm32-can-id-filter/
  can_filter.FilterMaskIdHigh = 0xFFFF;
  can_filter.FilterMaskIdLow  = 0xFFFF;
  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

float loop_ecd_constrain_test(float Input)
{
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
  if (Input > HALF_ECD_RANGE)
  {
    while (Input > HALF_ECD_RANGE)
    {
      Input -= ECD_RANGE;
    }
  }
  else if (Input < -HALF_ECD_RANGE)
  {
    while (Input < -HALF_ECD_RANGE)
    {
      Input += ECD_RANGE;
    }
  }
  return Input;
}

/**
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  uint8_t             index;

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data

  if(hcan->Instance == CAN1)
  {
    if ((rx_header.StdId >= CAN_6020_M1_ID) && (rx_header.StdId <=  CAN_6020_M4_ID))
    {
      index = rx_header.StdId - CAN_6020_M1_ID;                  // get motor index by can_id
      motor_info[index].feedback_ecd    = ((rx_data[0] << 8) | rx_data[1]);
      // motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
      // motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
      // motor_info[index].temperature    =   rx_data[6];
    }
    feedback_ecd1 = loop_ecd_constrain_test((float)motor_info[0].feedback_ecd - (float)motor_info[0].offset_ecd);
  }
  else if(hcan->Instance == CAN2)
  {
    // CAN_CHASSIS_CONTROLLER_RX_ID is set in filter, so no need to check
    for (index = 0; index < STEER_MOTOR_COUNT; index++)
    {
      motor_info[index].target_ecd = ((rx_data[2*index] << 8) | rx_data[2*index+1]);
    }
    target_ecd1 = loop_ecd_constrain_test(motor_info[0].target_ecd);
  }
}

/**
  * @brief  send motor control message through can bus
  * @param  motor voltage 1,2,3,4 or 5,6,7
  * @retval None
  */
void CAN_cmd_steer_motors(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = (id_range == 0) ? (CAN_CONTROL_ID_BASE) : (CAN_CONTROL_ID_EXTEND);
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  tx_data[0] = v1 >> 8;
  tx_data[1] = v1;
  tx_data[2] = v2 >> 8;
  tx_data[3] = v2;
  tx_data[4] = v3 >> 8;
  tx_data[5] = v3;
  tx_data[6] = v4 >> 8;
  tx_data[7] = v4;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
