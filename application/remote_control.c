/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      remote control process, remote control is transmitted through a protocol
  *            similar to SBUS, using DMA transmission method to save CPU
  *           resources, using serial port idle interrupt to pull up the processing function,
  *          and providing some offline restart DMA, serial port
  *        to ensure the stability of hot swap.
  * @note    This task is started by serial port interrupt, not freeRTOS task
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"
#include "cv_usart_task.h"

//remote control error limit
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
RC_ctrl_t rc_ctrl;
// receive raw data of 18 bytes, 36 bytes length is given to prevent DMA transmission overflow
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
int16_t raw_rc_ch[5];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

// determine whether the remote control data is wrong
uint8_t RC_data_is_error(void)
{
    // use go to statement to facilitate error handling and unify the processing of remote control variable data to zero
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void solve_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void solve_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE) // data received
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_length
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                // record time of data arrival
                detect_hook(DBUS_TOE);
                // sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_length
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                // process remote control data
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                // record time of data arrival
                detect_hook(DBUS_TOE);
                // sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }

}

// absolute function
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

	raw_rc_ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;                              //!< Channel 0
	raw_rc_ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;                       //!< Channel 1
	raw_rc_ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff; //!< Channel 2
	raw_rc_ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;                       //!< Channel 3
	raw_rc_ch[4] = (sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07ff;                            //!< Channel 4
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                                        //!< Switch right
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                                   //!< Switch left
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                                     //!< Mouse X axis
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                                     //!< Mouse Y axis
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                                   //!< Mouse Z axis
	rc_ctrl->mouse.press_l = sbus_buf[12];                                                   //!< Mouse Left Is Press ?
	rc_ctrl->mouse.press_r = sbus_buf[13];                                                   //!< Mouse Right Is Press ?
	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                                     //!< KeyBoard value

	raw_rc_ch[0] -= RC_CH_VALUE_OFFSET;
	raw_rc_ch[1] -= RC_CH_VALUE_OFFSET;
	raw_rc_ch[2] -= RC_CH_VALUE_OFFSET;
	raw_rc_ch[3] -= RC_CH_VALUE_OFFSET;
	raw_rc_ch[4] -= RC_CH_VALUE_OFFSET;

	// @TODO: fix residual spike despite this fix, where raw_rc_ch has no spike but rc_ctrl->rc.ch has.
	// Filtering out bit spikes
	static uint8_t bConsecutiveAbnormalityCount[5] = {0, 0, 0, 0, 0};
	for (uint8_t i = 0; i < 5; i++)
	{
		if ((RC_abs(raw_rc_ch[i] - rc_ctrl->rc.ch[i]) >= 256) && (bConsecutiveAbnormalityCount[i] < 2))
		{
			bConsecutiveAbnormalityCount[i]++;
		}
		else
		{
			rc_ctrl->rc.ch[i] = raw_rc_ch[i];
			bConsecutiveAbnormalityCount[i] = 0;
		}
	}

#if CV_INTERFACE
    CvCmder_DetectAutoAimSwitchEdge((rc_ctrl->key.v & AUTO_AIM_TOGGLE_KEYBOARD) != 0);
#endif
}

// // We don't use this feature. USART1 is used to communicate with CV instead.
// /**
//   * @brief          send sbus data by usart1, called in usart3_IRQHandle
//   * @param[in]      sbus: sbus data, 18 bytes
//   * @retval         none
//   */
// void sbus_to_usart1(uint8_t *sbus)
// {
//     static uint8_t usart_tx_buf[20];
//     static uint8_t i =0;
//     usart_tx_buf[0] = 0xA6;
//     memcpy(usart_tx_buf + 1, sbus, 18);
//     for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
//     {
//         usart_tx_buf[19] += usart_tx_buf[i];
//     }
//     usart1_tx_dma_enable(usart_tx_buf, 20);
// }
