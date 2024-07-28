#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "referee_send_task.h"
#include "string.h"
#include "robot_arm_task.h"
#include "CAN_receive.h"

#define FRAME_HEADER_LENGTH 5
#define CMD_ID_LENGTH 2
#define DATA_LENGTH 30
#define FRAME_TAIL_LENGTH 2

#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH)

#define CONTROLLER_CMD_ID 0x0302

/***********          ******************/
#define REF_SEND_TIME_MS 50.0f
#define REF_SEND_TIME_S (REF_SEND_TIME_MS / 1000.0f)

static void Data_Concatenation(uint8_t *data, uint16_t data_lenth);

Controller_t tx_data;
uint8_t data[DATA_LENGTH];

void referee_send_task(void const *argument)
{
    uint32_t ulSystemTime = osKernelSysTick();

    while (1)
    {
        uint8_t bCursor = 0;
        for (int i = 0; i < 7; i++)
        {
            memcpy(&data[bCursor], &motor_measure[i].output_angle, sizeof(motor_measure[i].output_angle));
            bCursor += sizeof(motor_measure[i].output_angle);
        }

        data[bCursor] = (robot_arm.arm_state == ARM_STATE_TEACHING);
        bCursor += 1;

        // Send data
        Data_Concatenation(data, DATA_LENGTH);
        HAL_UART_Transmit(&huart1, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
        osDelayUntil(&ulSystemTime, REF_SEND_TIME_MS); // max delay time for custom controller
    }
}

/**
 * @brief
 * @param data   锟捷段�?�拷    �??
 * @param data_lenth   锟捷段�?�拷
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    tx_data.frame_header.sof = 0xA5;
    tx_data.frame_header.data_length = data_lenth;
    tx_data.frame_header.seq = seq++;
    append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5);

    tx_data.cmd_id = CONTROLLER_CMD_ID;
    memcpy(tx_data.data, data, data_lenth);
    append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}
