/**
 * @file       cv_usart_task.c
 * @brief      Computer vision communication
 * @arthur     2022 MacFalcons Control Team
 * All UART packets from CV should have the same size (DATA_PACKAGE_SIZE). Pad packet payloads with CHAR_UNUSED in the end if necessary.
 */

#pragma push
#pragma anon_unions
// Start of section using anonymous unions

#include "cv_usart_task.h"
#include "cmsis_os.h"

#include "bsp_usart.h"
#include "detect_task.h"
#include "referee.h"
#include "string.h"
#include "usart.h"
#include "referee.h"
#include "gimbal_task.h"
#if DEBUG_CV_WITH_USB
#include "usb_task.h"
#include <stdio.h>
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif /* MIN */
#define DATA_PACKAGE_SIZE 10
#define TVL_INFO_SIZE 2
#define DATA_PACKAGE_HEADLESS_SIZE (DATA_PACKAGE_SIZE - TVL_INFO_SIZE)
#define DATA_PACKAGE_PAYLOAD_SIZE (DATA_PACKAGE_HEADLESS_SIZE - sizeof(uint16_t) - sizeof(uint8_t)) // sizeof(uiTimestamp) and sizeof(bMsgType)
#define CHAR_UNUSED 0xFF
#define SHOOT_TIMEOUT_MS 350
#define CV_TRANDELTA_FILTER_SIZE 4 // TranDelta means Transmission delay

// Test result with pyserial: 0 to 2 millisecond of cv msg receiving interval; Message burst is at max 63 bytes per time, so any number bigger than 63 is fine for Rx buffer size
uint8_t abUsartRxBuf[DATA_PACKAGE_SIZE];
//eMsgTypes CV_CMD_TYPE;
uint8_t CvCmdLength, fQpresses;
uint8_t fIsKeyPressingEdge = 0;
uint16_t shoot_heat_limit, shoot_heat, projectile_allowance_17mm, gold_coins;
#if CV_INTERFACE

typedef enum
{
	MSG_CHECK_STATE = 0x00,
	MSG_CV_CHASSIS_MOVE_STATE = 0x01,
	//MSG_ACK = 0x40,
	MSG_CONTROL_SPINNNG = 0x02,
	MSG_AIM_ERROR = 0x03,
	MSG_SHOOT_CMD = 0x04,
} eMsgTypes;

typedef enum
{
	CV_INFO_GAME_PROGRESS = 0x00,
	CV_INFO_TEAM_COLOR = 0x01,
	CV_INFO_ROBOT_TYPE = 0x02,
} eMsgTypeAckInfo;

//
// typedef enum
// {
// 	CV_INFO_TRANDELTA_BIT = 1 << 0,
// 	CV_INFO_CVSYNCTIME_BIT = 1 << 1,
// 	CV_INFO_REF_STATUS_BIT = 1 << 2,
// 	CV_INFO_GIMBAL_ANGLE_BIT = 1 << 3,
// 	CV_INFO_LAST_BIT = 1 << 4,
// } eInfoBits;
// STATIC_ASSERT(CV_INFO_LAST_BIT <= (1 << 8));

typedef struct
{
	moving_average_type_t TranDeltaFilter;
	fp32 adTranDeltaFilterBuffer[CV_TRANDELTA_FILTER_SIZE];
	int16_t iTranDeltaMA;
	uint16_t uiCtrlSyncTime;
	uint16_t uiCvSyncTime;
} tCvTimestamps;

void CvCmder_Init(void);
void CvCmder_PollForModeChange(void);
static void CvCmder_RxParserTlv(const uint8_t *pData, uint16_t size);
void CvCmder_EchoTxMsgToUsb(void);
//void CvCmder_SendSetModeRequest(void);
//void CvCmder_SendInfoData(eMsgTypes CvCmdBit);
//void CvCmder_UpdateTranDelta(void);
static void CvCmder_SendAck(uint8_t msgType);
#if DEBUG_CV_WITH_USB
uint8_t CvCmder_MockModeChange(void);
#endif

tCvCmdHandler CvCmdHandler;
// don't compare with literal string "ACK", since it contains extra NULL char at the end
const uint16_t abExpectedAckPayload;
uint8_t abExpectedUnusedPayload[DATA_PACKAGE_PAYLOAD_SIZE];
tCvTimestamps CvTimestamps;

#if DEBUG_CV_WITH_USB
uint8_t fIsUserKeyPressingEdge = 0;
char usbMsg[500];
volatile uint16_t uiUsbMsgSize;
#endif

void cv_usart_task(void const *argument)
{
	uint32_t ulSystemTime = osKernelSysTick();
	CvCmder_Init();
	while (1)
	{
		CvCmder_PollForModeChange();
		// shoot mode timeout logic
		if (CvCmder_GetMode(CV_MODE_SHOOT_BIT) && (osKernelSysTick() - CvCmdHandler.ulShootStartTime > SHOOT_TIMEOUT_MS))
		{
			CvCmder_ChangeMode(CV_MODE_SHOOT_BIT, 0);
		}
		osDelayUntil(&ulSystemTime, CV_CONTROL_TIME_MS);
	}
}

void CvCmder_Init(void)
{
	CvTimestamps.TranDeltaFilter.size = CV_TRANDELTA_FILTER_SIZE;
	CvTimestamps.TranDeltaFilter.cursor = 0;
	CvTimestamps.TranDeltaFilter.ring = CvTimestamps.adTranDeltaFilterBuffer;
	CvTimestamps.TranDeltaFilter.sum = 0;
	CvTimestamps.uiCtrlSyncTime = 0;

	//memcpy(CvRxBuffer.tData.abMessageHeader, abExpectedMessageHeader, sizeof(abExpectedMessageHeader));
	//memcpy(CvTxBuffer.tData.abMessageHeader, abExpectedMessageHeader, sizeof(abExpectedMessageHeader));
	memset(abExpectedUnusedPayload, CHAR_UNUSED, sizeof(abExpectedUnusedPayload));

	memset(&CvCmdHandler, 0, sizeof(CvCmdHandler));       // clear status
	CvCmdHandler.cv_rc_ctrl = get_remote_control_point(); // reserved, not used yet

	CvCmdHandler.fCvMode = 0;

	// Get a callback when DMA completes or IDLE
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, abUsartRxBuf, sizeof(abUsartRxBuf));
	// disable half transfer interrupt, because if it coincide with IDLE or DMA interrupt, callback will be called twice
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

	// RXNE is not used
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
}

void CvCmder_toe_solve_lost_fun(void)
{
	//memset(&(CvCmdHandler.CvCmdMsg), 0, sizeof(CvCmdHandler.CvCmdMsg));
	//To do: set all RX parameter to 0 when CV is offline
}
/**
 * @brief if a command is received from remote controller, we keep sending set-mode requests to CV until an ACK is received
 */
void CvCmder_PollForModeChange(void)
{
	// @TODO: Implement check for Auto-move mode and Enemy mode
	// uint8_t fLastEnemyMode = CvCmder_GetMode(CV_MODE_ENEMY_DETECTED_BIT);
	static enum {
		CMDER_STATE_IDLE,
		CMDER_STATE_WAIT_FOR_ACK,
	} eCvCmderState = CMDER_STATE_IDLE;

	switch (eCvCmderState)
	{
		case CMDER_STATE_IDLE:
		{
#if DEBUG_CV_WITH_USB
			if (checkAndResetFlag(&CvCmdHandler.fIsModeChanged) || toe_is_error(CV_TOE) || CvCmder_MockModeChange())
#else
			if (checkAndResetFlag(&CvCmdHandler.fIsModeChanged) || toe_is_error(CV_TOE))
#endif
			{
				//CvCmder_SendSetModeRequest();
				CvCmdHandler.fIsWaitingForAck = 1;
				eCvCmderState = CMDER_STATE_WAIT_FOR_ACK;
			}
			break;
		}
		case CMDER_STATE_WAIT_FOR_ACK:
		{
			if (CvCmdHandler.fIsWaitingForAck)
			{
				//CvCmder_SendSetModeRequest();
				// reset receive interrupt to detect new UART connection, in case CV boots up after control
				HAL_UARTEx_ReceiveToIdle_DMA(&huart1, abUsartRxBuf, sizeof(abUsartRxBuf));
				__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
			}
			else
			{
				// Session established until communication timeout
				eCvCmderState = CMDER_STATE_IDLE;
			}
			break;
		}
		default:
		{
			eCvCmderState = CMDER_STATE_IDLE;
			break;
		}
	}
}


void CvCmder_EchoTxMsgToUsb(void)
{
#if DEBUG_CV_WITH_USB
	// echo to usb
	// watch out of null character at the end
	memcpy(usbMsg, "Sent: ", sizeof("Sent: ") - 1);
	uiUsbMsgSize = sizeof("Sent: ") - 1;
	for (uint8_t i = 0; i < sizeof(CvTxBuffer.abData); i++)
	{
		if (uiUsbMsgSize + (sizeof("0x00,") - 1) > sizeof(usbMsg))
		{
			break;
		}
		uiUsbMsgSize += snprintf(&usbMsg[uiUsbMsgSize], sizeof(usbMsg) - uiUsbMsgSize, "0x%02X,", CvTxBuffer.abData[i]);
	}
	usbMsg[uiUsbMsgSize - 1] = '\n';
	usbMsg[uiUsbMsgSize] = 0;
	usb_printf("%s", usbMsg);
#endif
}

void CvCmder_DetectAutoAimSwitchEdge(uint8_t fIsKeyPressed)
{
	// no need to debounce because keyboard signal is clean
	static uint8_t fLastKeySignal = 0;
	if (fLastKeySignal != fIsKeyPressed)
	{
		if (fIsKeyPressed)
		{
			fQpresses = 1;
			CvCmder_ToggleMode(CV_MODE_AUTO_AIM_BIT);
		}
		fLastKeySignal = fIsKeyPressed;
		fQpresses = 0;
	}
}

static void CvCmder_SendAck(uint8_t msgType)
{
    // For example, Tag = msgType, Length = 1, Value = 0xAA (ACK placeholder)
    uint8_t ackBuf[4];
    ackBuf[0] = msgType; // Tag
          // Length
    ackBuf[2] = 0xFF;    // Value (ACK)
	switch(msgType){
		case MSG_CV_CHASSIS_MOVE_STATE:
		{
			ackBuf[1] = 1;  
			ackBuf[2] = 0xFF;
			break;
		}

		case MSG_CHECK_STATE:
		{
			ackBuf[1] = 2; 
			switch(CvCmdHandler.CvCmdMsg.cv_info_type){
				case CV_INFO_GAME_PROGRESS:{
					ackBuf[2] = 0x00;
					if(is_game_started()){
						ackBuf[3] = 0x00;
					}
					else{
						ackBuf[3] = 0xFF;
					}
					break;
				}
				case CV_INFO_TEAM_COLOR:{
					ackBuf[2] = 0x01;
					if(get_team_color() == 1){
						ackBuf[3] = 0x00;
					}
					else{
						ackBuf[3] = 0xFF;
					}
					break;
				}
				case CV_INFO_ROBOT_TYPE:
					ackBuf[2] = 0x02;
#if SENTRY_2023_MECANUM
					ackBuf[3] = 0x00;
#else
					ackBuf[3] = 0xFF;
#endif
					break;
			}
			break;
		}

		case MSG_CONTROL_SPINNNG:
		{
			if(CV_MODE_CHASSIS_SPINNING_BIT == 1){
				ackBuf[1] = 1; 
				ackBuf[2] = 0xFF;
			}
			else{
				ackBuf[1] = 1; 
				ackBuf[2] = 0x00;
			}
		}

		case MSG_AIM_ERROR:
		{
			ackBuf[1] = 1; 
			ackBuf[2] = 0xFF;
			break;
		}
		
		case MSG_SHOOT_CMD:
		{
#if !DEBUG_CV
			ackBuf[1] = 1;
			if((projectile_allowance_17mm == 0 && gold_coins < 50)){
				ackBuf[2] = 0x00;
			}
			else if(shoot_heat_limit <= shoot_heat-15){
				ackBuf[2] = 0xAA;
			}
			else if((gold_coins > 50)&& (projectile_allowance_17mm == 0)){
				ackBuf[2] = 0xBB;
			}
			else{
				ackBuf[2] = 0xFF  ;
			}
#else
			ackBuf[2] = 0xFF;
#endif
			//TODO: Return the correct response
		}
	}


    // Byte 3 could remain unused or contain a CRC etc. Set to 0xFF or 0 if you like
    //ackBuf[3] = 0xFF;

    HAL_UART_Transmit(&huart1, ackBuf, (uint16_t)ackBuf[1] + 2, 100);
}

static void CvCmder_RxParserTlv(const uint8_t *pData, uint16_t size)
{
    while (size >= 2)	
    {
        uint8_t  tag    = pData[0];
        uint8_t  length = pData[1];
        if (size < 2 + length)
            break; // incomplete packet

        switch (tag)
        {
        	case MSG_CHECK_STATE:
			{
        	    if (length == 1)
        	    {
        	        // pData[2] = state enum
        	        // TODO: handle state
					CvCmdHandler.CvCmdMsg.cv_info_type = pData[2];
					CvCmder_SendAck(MSG_CHECK_STATE);
					detect_hook(CV_TOE);
        	    }
        	    break;
			}
        	case MSG_CV_CHASSIS_MOVE_STATE:
			{
        	    if (length == 8)
        	    {
        	        fp32 xSpeed, ySpeed;
        	        memcpy(&xSpeed, &pData[2], 4);
        	        memcpy(&ySpeed, &pData[6], 4);
					CvCmdHandler.CvCmdMsg.xSpeed = xSpeed;
					CvCmdHandler.CvCmdMsg.ySpeed = ySpeed;
					CvCmder_ChangeMode(CV_MODE_AUTO_MOVE_BIT, 1);
					CvCmder_SendAck(MSG_CV_CHASSIS_MOVE_STATE);
        	    }


        	    break;
			}
        	case MSG_CONTROL_SPINNNG:
			{
        	    if (length == 1)
        	    {
        	        uint8_t spinCmd = pData[2]; // 0x00 or 0xFF
					if(spinCmd == 0xFF)
					{
						CvCmder_ChangeMode(CV_MODE_CHASSIS_SPINNING_BIT, 1);
					}
					else
					{
						CvCmder_ChangeMode(CV_MODE_CHASSIS_SPINNING_BIT, 0);
					}

					CvCmder_SendAck(MSG_CONTROL_SPINNNG);
					detect_hook(CV_TOE);
        	    }

        	    break;
			}
        	case MSG_AIM_ERROR:
			{
        	    if (length == 8)
        	    {
        	        fp32 xError, yError;
        	        memcpy(&xError, &pData[2], 4);
        	        memcpy(&yError, &pData[6], 4);
					CvCmdHandler.CvCmdMsg.xAimError = xError;
					CvCmdHandler.CvCmdMsg.yAimError = yError;
					CvCmder_SendAck(MSG_AIM_ERROR);
					detect_hook(CV_TOE);
        	        // TODO: handle aim error
        	    }
        	    break;
			}
			case MSG_SHOOT_CMD:
			{
				get_shoot_heat0_limit_and_heat(&shoot_heat_limit, &shoot_heat);
				get_remaining_gold_coins(&gold_coins);
				get_projectile_allowance_17mm(&projectile_allowance_17mm);
				if(length == 1){
					uint8_t shootCmd = pData[2];
#if !DEBUG_CV
					if((shootCmd == 0xFF) && (projectile_allowance_17mm > 0) &&  ((shoot_heat-10)< shoot_heat_limit)){
#else
					if((shootCmd == 0xFF)){
#endif
						CvCmder_ChangeMode(CV_MODE_SHOOT_BIT, 1);
					} else {
						CvCmder_ChangeMode(CV_MODE_SHOOT_BIT, 0);
					}
					CvCmder_SendAck(MSG_SHOOT_CMD);
					detect_hook(CV_TOE);
				}

				break;
			}
        	default:
			{
        	    // unknown tag
				CvCmdHandler.CvCmdMsg.xAimError = 0.0f;
				CvCmdHandler.CvCmdMsg.yAimError = 0.0f;
				CvCmdHandler.CvCmdMsg.xSpeed = 0.0f;
				CvCmdHandler.CvCmdMsg.ySpeed = 0.0f;
				CvCmder_ChangeMode(CV_MODE_CHASSIS_SPINNING_BIT, 0);
				CvCmder_ChangeMode(CV_MODE_SHOOT_BIT, 0);
				CvCmder_ChangeMode(CV_MODE_AUTO_MOVE_BIT, 0);
        	    break;
			}

        }
        pData += (2 + length);
        size  -= (2 + length);
    }
}

//Mo mode bit anymore, kep it here but won't be used.
uint8_t CvCmder_GetMode(uint8_t bCvModeBit)
{
	return (CvCmdHandler.fCvMode & bCvModeBit);
}

void CvCmder_ToggleMode(uint8_t bCvModeBit)
{
	CvCmdHandler.fCvMode = (CvCmdHandler.fCvMode & ~bCvModeBit) | (!CvCmder_GetMode(bCvModeBit) ? bCvModeBit : 0);
	CvCmdHandler.fIsModeChanged = 1;
}

/**
 * @brief Change specific mode bit within mode byte. Cannot change multiple bits at once.
 * @param bCvModeBit: type is eModeControlBits
 * @param fFlag: 1 or 0
 */
void CvCmder_ChangeMode(uint8_t bCvModeBit, uint8_t fFlag)
{
	uint8_t fLastMode = CvCmder_GetMode(bCvModeBit);
	if (fLastMode != fFlag)
	{
		CvCmdHandler.fCvMode = (CvCmdHandler.fCvMode & ~bCvModeBit) | (fFlag ? bCvModeBit : 0);
		CvCmdHandler.fIsModeChanged = 1;
	}
}

/**
 * @brief Get pointer to Cv Cmd Handler
 * TODO: use it in gimbal_task and chassis_task
 */
tCvCmdHandler *CvCmder_GetHandler(void)
{
	return &CvCmdHandler;
}

#if DEBUG_CV_WITH_USB
uint8_t CvCmder_CheckAndResetUserKeyEdge(void)
{
	return checkAndResetFlag(&fIsUserKeyPressingEdge);
}

/**
 * @brief mock event of user enabling/disabling CV control mode by pressing button
 */
uint8_t CvCmder_MockModeChange(void)
{
	// polling for key press. We can't use interrupt because some other module interfere with it (maybe IMU DMA)
	GPIO_PinState fRead1;
	GPIO_PinState fRead2;
	fRead1 = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
	// debounce
	osDelay(20);
	fRead2 = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

	if (fRead1 == fRead2)
	{
		static GPIO_PinState LastUserKeyState = GPIO_PIN_SET;
		fIsUserKeyPressingEdge = (fRead1 == GPIO_PIN_RESET) && (LastUserKeyState == GPIO_PIN_SET);
		LastUserKeyState = fRead1;

		if (fIsUserKeyPressingEdge)
		{
			static uint8_t bMockCounter = 0;
			switch (bMockCounter)
			{
				case 0:
				{
					CvCmdHandler.fCvMode = 0;
					break;
				}
				case 1:
				{
					CvCmdHandler.fCvMode = CV_MODE_ENEMY_DETECTED_BIT;
					break;
				}
				case 2:
				{
					CvCmdHandler.fCvMode = CV_MODE_AUTO_MOVE_BIT | CV_MODE_ENEMY_DETECTED_BIT;
					break;
				}
				case 3:
				{
					CvCmdHandler.fCvMode = CV_MODE_AUTO_AIM_BIT | CV_MODE_AUTO_MOVE_BIT | CV_MODE_ENEMY_DETECTED_BIT;
					break;
				}
				case 4:
				{
					CvCmdHandler.fCvMode = CV_MODE_AUTO_AIM_BIT | CV_MODE_AUTO_MOVE_BIT | CV_MODE_ENEMY_DETECTED_BIT | CV_MODE_SHOOT_BIT;
					break;
				}
			}
			bMockCounter = (bMockCounter + 1) % 5;
		}
	}
	return fIsUserKeyPressingEdge;
}
#endif // DEBUG_CV_WITH_USB

#endif // CV_INTERFACE

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
#if CV_INTERFACE
        // Directly parse all incoming data as TLV
        CvCmder_RxParserTlv(abUsartRxBuf, Size);
#endif

        // Restart DMA reception
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, abUsartRxBuf, sizeof(abUsartRxBuf));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

// End of section using anonymous unions
#pragma pop
