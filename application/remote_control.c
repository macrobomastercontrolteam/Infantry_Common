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
#include "CRC8_CRC16.h"
#include "string.h"

#include "detect_task.h"
#include "cv_usart_task.h"

//remote control error limit
#define RC_CHANNAL_ERROR_VALUE 700
#define RC_CHANNEL_DROPIN_LIMIT ((int16_t)660)

#if (REMOTE_TYPE == REMOTE_USE_VT13)
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
#else
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
#endif


static int16_t RC_abs(int16_t value);
#if (REMOTE_TYPE == REMOTE_USE_VT13)
static int16_t rc_constrain_to_dr16_range(int16_t value);
#endif
#if (REMOTE_TYPE == REMOTE_USE_VT13)
static uint8_t vt13_mode_to_switch(uint8_t mode_raw);
static uint8_t vt13_verify_frame_crc(const uint8_t *frame);
#endif
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
#if (REMOTE_TYPE == REMOTE_USE_DR16)
static void dr16_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#endif
#if (REMOTE_TYPE == REMOTE_USE_VT13)
static uint8_t vt13_to_rc(const uint8_t *rx_buf, uint16_t rx_len, RC_ctrl_t *rc_ctrl);
#endif

//remote control data 
RC_ctrl_t rc_ctrl;
// receive raw data of 18 bytes, 36 bytes length is given to prevent DMA transmission overflow
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
int16_t raw_rc_ch[5];

#if (REMOTE_TYPE == REMOTE_USE_VT13)
// Debug counters - inspect via debugger to localize the failure point.
volatile uint32_t vt13_dbg_irq_total      = 0; // any USART6 IRQ entered
volatile uint32_t vt13_dbg_idle_events    = 0; // IDLE branch taken
volatile uint32_t vt13_dbg_rxne_only      = 0; // RXNE-only entries (IDLE skipped)
volatile uint32_t vt13_dbg_parse_calls    = 0; // vt13_to_rc() invocations
volatile uint32_t vt13_dbg_parse_ok       = 0; // vt13_to_rc() returned 1
volatile uint32_t vt13_dbg_header_seen    = 0; // 0xA9 0x53 found in buffer
volatile uint32_t vt13_dbg_crc_pass       = 0; // CRC verified OK
volatile uint16_t vt13_dbg_last_rx_len    = 0;
volatile uint8_t  vt13_dbg_last_frame[VT13_FRAME_LENGTH];
#endif

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
    rc_ctrl.vt13.trigger = 0;
    rc_ctrl.vt13.customizable_button_left = 0;
    rc_ctrl.vt13.customizable_button_right = 0;
    rc_ctrl.vt13.pause_button = 0;
    rc_ctrl.vt13.dial = 0;
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

#if (REMOTE_TYPE == REMOTE_USE_VT13)
void USART6_IRQHandler(void)
{
    vt13_dbg_irq_total++;
    if(huart6.Instance->SR & UART_FLAG_RXNE) // data received
    {
        vt13_dbg_rxne_only++;
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
        vt13_dbg_idle_events++;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_length
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(vt13_to_rc(sbus_rx_buf[0], this_time_rx_len, &rc_ctrl))
            {
                detect_hook(REMOTE_TOE);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_length
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

#if (REMOTE_TYPE == REMOTE_USE_DR16)
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                // process remote control data
                dr16_to_rc(sbus_rx_buf[1], &rc_ctrl);
                // record time of data arrival
                detect_hook(REMOTE_TOE);
            }
#elif (REMOTE_TYPE == REMOTE_USE_VT13)
            if(vt13_to_rc(sbus_rx_buf[1], this_time_rx_len, &rc_ctrl))
            {
                detect_hook(REMOTE_TOE);
            }
#endif
        }
    }

}
#else

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

#if (REMOTE_TYPE == REMOTE_USE_DR16)
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                dr16_to_rc(sbus_rx_buf[0], &rc_ctrl);
                // record time of data arrival
                detect_hook(REMOTE_TOE);
            }
#endif
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

#if (REMOTE_TYPE == REMOTE_USE_DR16)
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                // process remote control data
                dr16_to_rc(sbus_rx_buf[1], &rc_ctrl);
                // record time of data arrival
                detect_hook(REMOTE_TOE);
            }
#endif
        }
    }

}
#endif

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

#if (REMOTE_TYPE == REMOTE_USE_VT13)
static int16_t rc_constrain_to_dr16_range(int16_t value)
{
    if (value > RC_CHANNEL_DROPIN_LIMIT)
    {
        return RC_CHANNEL_DROPIN_LIMIT;
    }
    if (value < -RC_CHANNEL_DROPIN_LIMIT)
    {
        return -RC_CHANNEL_DROPIN_LIMIT;
    }
    return value;
}
#endif
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
#if (REMOTE_TYPE == REMOTE_USE_DR16)
static void dr16_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
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
    rc_ctrl->vt13.trigger = 0;
    rc_ctrl->vt13.customizable_button_left = 0;
    rc_ctrl->vt13.customizable_button_right = 0;
    rc_ctrl->vt13.pause_button = 0;
    rc_ctrl->vt13.dial = 0;

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
#endif

#if (REMOTE_TYPE == REMOTE_USE_VT13)
static uint8_t vt13_to_rc(const uint8_t *rx_buf, uint16_t rx_len, RC_ctrl_t *rc_ctrl)
{
    if ((rx_buf == NULL) || (rc_ctrl == NULL) || (rx_len < VT13_FRAME_LENGTH))
    {
        return 0;
    }

#if (REMOTE_TYPE == REMOTE_USE_VT13)
    vt13_dbg_parse_calls++;
    vt13_dbg_last_rx_len = rx_len;
#endif

    for (uint16_t i = 0; i <= (rx_len - VT13_FRAME_LENGTH); i++)
    {
        const uint8_t *frame = rx_buf + i;
        if ((frame[0] != 0xA9) || (frame[1] != 0x53))
        {
            continue;
        }
#if (REMOTE_TYPE == REMOTE_USE_VT13)
        vt13_dbg_header_seen++;
        for (uint8_t k = 0; k < VT13_FRAME_LENGTH; k++)
        {
            vt13_dbg_last_frame[k] = frame[k];
        }
#endif
        if (!vt13_verify_frame_crc(frame))
        {
            continue;
        }
#if (REMOTE_TYPE == REMOTE_USE_VT13)
        vt13_dbg_crc_pass++;
#endif

        uint64_t packed = 0;
        for (uint8_t byte_idx = 0; byte_idx < 8; byte_idx++)
        {
            packed |= ((uint64_t)frame[2 + byte_idx]) << (8u * byte_idx);
        }

        // VT13 -> DR16 drop-in channel mapping:
        // ch0~ch3: joysticks, ch4: dial/wheel.
        raw_rc_ch[0] = (int16_t)((packed >> 0) & 0x07FFu);
        raw_rc_ch[1] = (int16_t)((packed >> 11) & 0x07FFu);
        raw_rc_ch[3] = (int16_t)((packed >> 22) & 0x07FFu);
        raw_rc_ch[2] = (int16_t)((packed >> 33) & 0x07FFu);
        // Dial direction is reversed vs. DR16 wheel convention.
        raw_rc_ch[4] = (int16_t)(2u * RC_CH_VALUE_OFFSET - (int16_t)((packed >> 49) & 0x07FFu));

        raw_rc_ch[0] = rc_constrain_to_dr16_range(raw_rc_ch[0] - RC_CH_VALUE_OFFSET);
        raw_rc_ch[1] = rc_constrain_to_dr16_range(raw_rc_ch[1] - RC_CH_VALUE_OFFSET);
        raw_rc_ch[2] = rc_constrain_to_dr16_range(raw_rc_ch[2] - RC_CH_VALUE_OFFSET);
        raw_rc_ch[3] = rc_constrain_to_dr16_range(raw_rc_ch[3] - RC_CH_VALUE_OFFSET);
        raw_rc_ch[4] = rc_constrain_to_dr16_range(raw_rc_ch[4] - RC_CH_VALUE_OFFSET);

        // Filtering out bit spikes
        static uint8_t bConsecutiveAbnormalityCount[5] = {0, 0, 0, 0, 0};
        for (uint8_t ch_idx = 0; ch_idx < 5; ch_idx++)
        {
            if ((RC_abs(raw_rc_ch[ch_idx] - rc_ctrl->rc.ch[ch_idx]) >= 256) && (bConsecutiveAbnormalityCount[ch_idx] < 2))
            {
                bConsecutiveAbnormalityCount[ch_idx]++;
            }
            else
            {
                rc_ctrl->rc.ch[ch_idx] = raw_rc_ch[ch_idx];
                bConsecutiveAbnormalityCount[ch_idx] = 0;
            }
        }

        uint8_t mode_sw = (uint8_t)((packed >> 44) & 0x03u);
        uint8_t mode_sw_norm = vt13_mode_to_switch(mode_sw);
        rc_ctrl->rc.s[0] = mode_sw_norm;
        // VT13 has one mode switch; keep left lever neutral to avoid unexpected DR16-only side effects.
        rc_ctrl->rc.s[1] = RC_SW_MID;
        rc_ctrl->vt13.trigger = (uint8_t)((packed >> 60) & 0x01u);

        // Extract VT13 buttons
        rc_ctrl->vt13.pause_button = (uint8_t)((packed >> 46) & 0x01u);
        rc_ctrl->vt13.customizable_button_left = (uint8_t)((packed >> 47) & 0x01u);
        rc_ctrl->vt13.customizable_button_right = (uint8_t)((packed >> 48) & 0x01u);

        // Extract VT13 dial (11 bits at offset 65, which is bits 49-59 in packed)
        rc_ctrl->vt13.dial = (int16_t)((packed >> 49) & 0x07FFu);
        rc_ctrl->vt13.dial = rc_constrain_to_dr16_range(rc_ctrl->vt13.dial - RC_CH_VALUE_OFFSET);

        rc_ctrl->mouse.x = (int16_t)(frame[10] | (frame[11] << 8));
        rc_ctrl->mouse.y = -((int16_t)(frame[12] | (frame[13] << 8)));  // Negate Y for correct orientation
        rc_ctrl->mouse.z = (int16_t)(frame[14] | (frame[15] << 8));

        uint8_t mouse_bits = frame[16];
        rc_ctrl->mouse.press_l = ((mouse_bits & 0x03u) != 0u);
        rc_ctrl->mouse.press_r = (((mouse_bits >> 2) & 0x03u) != 0u);
        rc_ctrl->key.v = (uint16_t)(frame[17] | (frame[18] << 8));

#if CV_INTERFACE
        CvCmder_DetectAutoAimSwitchEdge((rc_ctrl->key.v & AUTO_AIM_TOGGLE_KEYBOARD) != 0);
#endif
#if (REMOTE_TYPE == REMOTE_USE_VT13)
        vt13_dbg_parse_ok++;
#endif
        return 1;
    }

    return 0;
}

static uint8_t vt13_mode_to_switch(uint8_t mode_raw)
{
    switch (mode_raw)
    {
        case 0u:
            return RC_SW_UP;
        case 1u:
            return RC_SW_MID;
        case 2u:
            return RC_SW_DOWN;
        default:
            return RC_SW_DOWN;
    }
}

// VT13 frame CRC: despite the datasheet stating "CRC-16/CCITT-FALSE", DJI's
// VT13uses reflected CRC-16/MCRF4XX algorithm
static uint8_t vt13_verify_frame_crc(const uint8_t *frame)
{
    return verify_CRC16_check_sum((uint8_t *)frame, VT13_FRAME_LENGTH) ? 1u : 0u;
}
#endif


bool_t key_rising_edge(uint8_t *last, uint8_t current)
{
    bool_t rising = (current && !(*last));
    *last = current;
    return rising;
}

bool_t key_falling_edge(uint8_t *last, uint8_t current)
{
    bool_t rising = (!current && *last);
    *last = current;
    return rising;
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
