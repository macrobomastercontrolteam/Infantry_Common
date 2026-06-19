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
#include "INS_task.h"
#include "CRC8_CRC16.h"
#if DEBUG_CV_WITH_USB
#include "usb_task.h"
#include <stdio.h>
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif /* MIN */
#define DATA_PACKAGE_SIZE 10
#define CV_RX_BUF_SIZE 64 // DMA-to-idle Rx buffer; large enough for a full burst of TLV records (each now carries a trailing CRC8)
#define TVL_INFO_SIZE 2
#define DATA_PACKAGE_HEADLESS_SIZE (DATA_PACKAGE_SIZE - TVL_INFO_SIZE)
#define DATA_PACKAGE_PAYLOAD_SIZE (DATA_PACKAGE_HEADLESS_SIZE - sizeof(uint16_t) - sizeof(uint8_t)) // sizeof(uiTimestamp) and sizeof(bMsgType)
#define CHAR_UNUSED 0xFF
#define SHOOT_TIMEOUT_MS 350
#define CHASSIS_SPIN_TIMEOUT_MS 15000
#define CV_TRANDELTA_FILTER_SIZE 4 // TranDelta means Transmission delay
#define CV_SPEED_FILTER_ALPHA 0.25f

// Test result with pyserial: 0 to 2 millisecond of cv msg receiving interval; Message burst is at max 63 bytes per time, so any number bigger than 63 is fine for Rx buffer size
uint8_t abUsartRxBuf[CV_RX_BUF_SIZE];
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
	MSG_CV_IMU_ACCELE = 0x05,
	MSG_CV_IMU_VELOCITY = 0x06,
	MSG_CV_IMU_POSITION = 0x07,
	MSG_CV_INFO_GIMBAL_ANGLE = 0x08,
} eMsgTypes;

#define CV_NUM_REQ_TYPES (MSG_CV_INFO_GIMBAL_ANGLE + 1) // number of request Tags (0x00..0x08), used to size the debug frequency arrays

typedef enum
{
	CV_INFO_GAME_PROGRESS = 0x00,
	CV_INFO_TEAM_COLOR = 0x01,
	CV_INFO_ROBOT_TYPE = 0x02,
	CV_INFO_ROBOT_HP = 0x03,
	//CV_INFO_PITCH_ANGLE = 0x04,
} eMsgTypeAckInfo;

// The protocol is TV (Tag-Value): the Length byte was removed from the wire. Both sides must derive
// each frame's value size from its Tag. On-wire frame is [Tag][Value...][CRC8]; total frame size =
// 1 (Tag) + value length + 1 (CRC8). CRC8 covers [Tag][Value...] (DJI poly, init 0xFF).

// Value-field byte lengths for CV -> Board request frames, indexed by Tag.
typedef enum
{
	REQ_LEN_CHECK_STATE          = 1,                // InfoType
	REQ_LEN_CV_CHASSIS_MOVE      = 2 * sizeof(fp32), // xSpeed + ySpeed
	REQ_LEN_CONTROL_SPINNING     = 1,                // SpinCmd
	REQ_LEN_AIM_ERROR            = 2 * sizeof(fp32), // xAimError + yAimError
	REQ_LEN_SHOOT_CMD            = 1,                // ShootCmd
	REQ_LEN_CV_IMU_ACCELE        = 0,                // request trigger (no value)
	REQ_LEN_CV_IMU_VELOCITY      = 0,                // request trigger (no value)
	REQ_LEN_CV_IMU_POSITION      = 0,                // request trigger (no value)
	REQ_LEN_CV_INFO_GIMBAL_ANGLE = 0,                // request trigger (no value)
} eCvReqValueLen;

// Value-field byte lengths for Board -> CV response frames, indexed by Tag.
typedef enum
{
	RSP_LEN_CHECK_STATE          = 2,                                   // InfoType + InfoValue
	RSP_LEN_CV_CHASSIS_MOVE      = 1,                                   // Ack
	RSP_LEN_CONTROL_SPINNING     = 1,                                   // Ack
	RSP_LEN_AIM_ERROR            = 1,                                   // Ack
	RSP_LEN_SHOOT_CMD            = 1,                                   // Ack
	RSP_LEN_CV_IMU_ACCELE        = 6 * sizeof(fp32) + sizeof(uint32_t), // 3 accel + 3 gyro + send-delay
	RSP_LEN_CV_IMU_VELOCITY      = 3 * sizeof(fp32) + sizeof(uint32_t), // x,y,z velocity + send-delay
	RSP_LEN_CV_IMU_POSITION      = 3 * sizeof(fp32) + sizeof(uint32_t), // x,y,z position + send-delay
	RSP_LEN_CV_INFO_GIMBAL_ANGLE = 4 * sizeof(fp32),                    // pitch+yaw angle + pitch+yaw rate
} eCvRspValueLen;

#define CV_REQ_LEN_UNKNOWN 0xFFU // returned for an unrecognised request Tag (frame size cannot be derived)

typedef struct
{
	moving_average_type_t TranDeltaFilter;
	fp32 adTranDeltaFilterBuffer[CV_TRANDELTA_FILTER_SIZE];
	int16_t iTranDeltaMA;
	uint16_t uiCtrlSyncTime;
	uint16_t uiCvSyncTime;
} tCvTimestamps;

typedef struct
{
	fp32 xSpeed;
	fp32 ySpeed;
	uint8_t fInitialized;
} tCvSpeedFilter;

void CvCmder_Init(void);
void CvCmder_PollForModeChange(void);
static void CvCmder_RxParserTlv(const uint8_t *pData, uint16_t size);
void CvCmder_EchoTxMsgToUsb(void);
//void CvCmder_SendSetModeRequest(void);
//void CvCmder_SendInfoData(eMsgTypes CvCmdBit);
//void CvCmder_UpdateTranDelta(void);
static void CvCmder_SendAck(uint8_t msgType);
static uint8_t CvCmder_GetReqValueLen(uint8_t msgType);
static uint8_t CvCmder_GetRspValueLen(uint8_t msgType);
#if DEBUG_CV_WITH_USB
uint8_t CvCmder_MockModeChange(void);
#endif

tCvCmdHandler CvCmdHandler;
// don't compare with literal string "ACK", since it contains extra NULL char at the end
const uint16_t abExpectedAckPayload;
uint8_t abExpectedUnusedPayload[DATA_PACKAGE_PAYLOAD_SIZE];
tCvTimestamps CvTimestamps;
tCvSpeedFilter CvSpeedFilter;
volatile uint32_t ulCvRxTimestamp = 0; ///< DWT cycle count captured when a CV request frame is received, used to compute the IMU send delay (microseconds)

// --- Debug: per-request update frequency, indexed by message Tag (0x00..0x08) ---
volatile uint16_t cv_request_count[CV_NUM_REQ_TYPES]; ///< CRC-valid requests received since the last frequency calculation, indexed by Tag
fp32 cv_request_freq_hz[CV_NUM_REQ_TYPES];            ///< debug watch: measured request rate in Hz, indexed by Tag

#if DEBUG_CV_WITH_USB
uint8_t fIsUserKeyPressingEdge = 0;
char usbMsg[500];
volatile uint16_t uiUsbMsgSize;
#endif

void cv_usart_task(void const *argument)
{
	uint32_t ulSystemTime = osKernelSysTick();
	CvCmder_Init();

//enable auto aim mode for Sentry 2023 Mecanum robots
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)	|| (ROBOT_TYPE == SENTRY_2026_OMNI)
	CvCmder_ToggleMode(CV_MODE_AUTO_AIM_BIT);
#endif
	//Init all the flags at the beginning of the task
	CvCmdHandler.CvCmdMsg.xAimError = 0.0f;
	CvCmdHandler.CvCmdMsg.yAimError = 0.0f;
	CvCmdHandler.CvCmdMsg.xSpeed = 0.0f;
	CvCmdHandler.CvCmdMsg.ySpeed = 0.0f;
	CvCmder_ChangeMode(CV_MODE_CHASSIS_SPINNING_BIT, 0);
	CvCmder_ChangeMode(CV_MODE_SHOOT_BIT, 0);
	CvCmder_ChangeMode(CV_MODE_AUTO_MOVE_BIT, 0);

	while (1)
	{
		CvCmder_PollForModeChange();
		// shoot mode timeout logic for automatic robots
#if (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI)
		if (CvCmder_GetMode(CV_MODE_SHOOT_BIT) && (osKernelSysTick() - CvCmdHandler.ulShootStartTime > SHOOT_TIMEOUT_MS))
		{
			CvCmder_ChangeMode(CV_MODE_SHOOT_BIT, 0);
		}
		if (CvCmder_GetMode(CV_MODE_CHASSIS_SPINNING_BIT) && (osKernelSysTick() - CvCmdHandler.ulChassisSpinStartTime > CHASSIS_SPIN_TIMEOUT_MS))
		{
			CvCmder_ChangeMode(CV_MODE_CHASSIS_SPINNING_BIT, 0);
		}
#endif

		// Debug: convert the per-Tag request counts accumulated since the last pass into an update rate (Hz)
		{
			static uint32_t ulLastFreqCalcTime = 0;
			uint32_t ulNow = osKernelSysTick();
			uint32_t ulElapsedMs = ulNow - ulLastFreqCalcTime;
			if (ulElapsedMs > 0)
			{
				for (uint8_t i = 0; i < CV_NUM_REQ_TYPES; i++)
				{
					cv_request_freq_hz[i] = (fp32)cv_request_count[i] * 1000.0f / (fp32)ulElapsedMs;
					cv_request_count[i] = 0;
				}
				ulLastFreqCalcTime = ulNow;
			}
		}

		osDelayUntil(&ulSystemTime, CV_CONTROL_TIME_MS);
	}
}

void CvCmder_Init(void)
{
	// Enable the DWT cycle counter for microsecond-resolution timing of the IMU send delay
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

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
	CvCmdHandler.ulChassisSpinStartTime = 0;
	CvSpeedFilter.fInitialized = 0;
	CvSpeedFilter.xSpeed = 0.0f;
	CvSpeedFilter.ySpeed = 0.0f;

	// Get a callback when DMA completes or IDLE
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, abUsartRxBuf, sizeof(abUsartRxBuf));
	// disable half transfer interrupt, because if it coincide with IDLE or DMA interrupt, callback will be called twice
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

	// RXNE is not used
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
}
void CvCmder_toe_solve_lost_fun(void)
{
	// CV is offline: clear aim/speed commands so stale data cannot keep driving the gimbal
	CvCmdHandler.CvCmdMsg.xAimError = 0.0f;
	CvCmdHandler.CvCmdMsg.yAimError = 0.0f;
	CvCmdHandler.CvCmdMsg.xSpeed = 0.0f;
	CvCmdHandler.CvCmdMsg.ySpeed = 0.0f;
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
			CvCmder_ToggleMode(CV_MODE_ASSIST_BIT);
		}
		fLastKeySignal = fIsKeyPressed;
		fQpresses = 0;
	}
}

// Returns the request value-field length (bytes) for a Tag, or CV_REQ_LEN_UNKNOWN if unrecognised.
static uint8_t CvCmder_GetReqValueLen(uint8_t msgType)
{
	switch (msgType)
	{
		case MSG_CHECK_STATE:           return REQ_LEN_CHECK_STATE;
		case MSG_CV_CHASSIS_MOVE_STATE: return REQ_LEN_CV_CHASSIS_MOVE;
		case MSG_CONTROL_SPINNNG:       return REQ_LEN_CONTROL_SPINNING;
		case MSG_AIM_ERROR:             return REQ_LEN_AIM_ERROR;
		case MSG_SHOOT_CMD:             return REQ_LEN_SHOOT_CMD;
		case MSG_CV_IMU_ACCELE:         return REQ_LEN_CV_IMU_ACCELE;
		case MSG_CV_IMU_VELOCITY:       return REQ_LEN_CV_IMU_VELOCITY;
		case MSG_CV_IMU_POSITION:       return REQ_LEN_CV_IMU_POSITION;
		case MSG_CV_INFO_GIMBAL_ANGLE:  return REQ_LEN_CV_INFO_GIMBAL_ANGLE;
		default:                        return CV_REQ_LEN_UNKNOWN;
	}
}

// Returns the response value-field length (bytes) for a Tag, or 0 if unrecognised.
static uint8_t CvCmder_GetRspValueLen(uint8_t msgType)
{
	switch (msgType)
	{
		case MSG_CHECK_STATE:           return RSP_LEN_CHECK_STATE;
		case MSG_CV_CHASSIS_MOVE_STATE: return RSP_LEN_CV_CHASSIS_MOVE;
		case MSG_CONTROL_SPINNNG:       return RSP_LEN_CONTROL_SPINNING;
		case MSG_AIM_ERROR:             return RSP_LEN_AIM_ERROR;
		case MSG_SHOOT_CMD:             return RSP_LEN_SHOOT_CMD;
		case MSG_CV_IMU_ACCELE:         return RSP_LEN_CV_IMU_ACCELE;
		case MSG_CV_IMU_VELOCITY:       return RSP_LEN_CV_IMU_VELOCITY;
		case MSG_CV_IMU_POSITION:       return RSP_LEN_CV_IMU_POSITION;
		case MSG_CV_INFO_GIMBAL_ANGLE:  return RSP_LEN_CV_INFO_GIMBAL_ANGLE;
		default:                        return 0;
	}
}

static void CvCmder_SendAck(uint8_t msgType)
{
    // TV response frame: [Tag][Value...][CRC8]. The value length is derived from the Tag (no Length
    // byte on the wire). Buffer sized for the largest payload (6 fp32 + delta) plus Tag and CRC8.
    uint8_t ackBuf[40];
    uint8_t fAppendSendDelay = 0;            // data responses carry a trailing uint32_t send-delay (microseconds)
    uint8_t valueLen = CvCmder_GetRspValueLen(msgType); // value byte count for this Tag
    ackBuf[0] = msgType;                     // Tag
    ackBuf[1] = 0xFF;                         // Value (ACK placeholder)
	switch(msgType){
		case MSG_CV_CHASSIS_MOVE_STATE:
		{
			ackBuf[1] = 0xFF;
			break;
		}

		case MSG_CHECK_STATE:
		{
			switch(CvCmdHandler.CvCmdMsg.cv_info_type){
				case CV_INFO_GAME_PROGRESS:{
					ackBuf[1] = 0x00;
					if(is_game_started()){
						ackBuf[2] = 0x00;
					}
					else{
						ackBuf[2] = 0xFF;
					}
					break;
				}
				case CV_INFO_TEAM_COLOR:{
					ackBuf[1] = 0x01;
					if(get_team_color() == 1){
						ackBuf[2] = 0x00;
					}
					else{
						ackBuf[2] = 0xFF;
					}
					break;
				}
				case CV_INFO_ROBOT_TYPE:{
					ackBuf[1] = 0x02;
#if (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI)	
					ackBuf[2] = 0x00;
#else
					ackBuf[2] = 0xFF;
#endif
					break;
				}

				case CV_INFO_ROBOT_HP:
				{
					uint16_t currentHP = get_current_HP();
					ackBuf[1] = 0x03;
					memcpy(&ackBuf[2], &currentHP, 2);
					break;
				}

			}
			break;
		}

		case MSG_CONTROL_SPINNNG:
		{
			if(CvCmder_GetMode(CV_MODE_CHASSIS_SPINNING_BIT)){
				ackBuf[1] = 0xFF;
			}
			else{
				ackBuf[1] = 0x00;
			}
			break;
		}

		case MSG_AIM_ERROR:
		{
			ackBuf[1] = 0xFF;
			break;
		}
		
		case MSG_SHOOT_CMD:
		{
#if !DEBUG_CV
	#if COMPETITION_TYPE == RMUC
			if((projectile_allowance_17mm == 0 && gold_coins < 50)){
				ackBuf[1] = 0x00; //running low on 17mm ammo
			}
			else if(shoot_heat_limit <= shoot_heat-30){
				ackBuf[1] = 0xAA; // shoot heat is low enough to allow shooting
			}
			else if((gold_coins > 50)&& (projectile_allowance_17mm == 0)){
				ackBuf[1] = 0xBB; // running low on 17mm ammo, but have enough gold coins to buy more
			}
			else{
				ackBuf[1] = 0xFF; // shoot
			}
	#else //For RMUL there is no economy system and projectial limit
			if(projectile_allowance_17mm == 0)
			{
				ackBuf[1] = 0x00; //running low on 17mm ammo
			}
			else if (shoot_heat_limit <= shoot_heat - 30)
			{
				ackBuf[1] = 0xAA; // shoot heat is low enough to allow shooting
			}
			else
			{
				ackBuf[1] = 0xFF; // shoot
			}

	#endif
#else
			ackBuf[1] = 0xFF;
#endif
			break;
		}

        case MSG_CV_IMU_ACCELE: //We are sending CV raw data
        {
            fp32 accel_data[3];
			fp32 ang_vel_data[3];
            get_world_accel_raw(accel_data, ang_vel_data); // Get world frame linear acceleration

            memcpy(&ackBuf[1], accel_data, 3 * sizeof(fp32));
            memcpy(&ackBuf[1 + 3 * sizeof(fp32)], ang_vel_data, 3 * sizeof(fp32));
            fAppendSendDelay = 1; // send-delay is written just before transmit
            break;
        }

        case MSG_CV_IMU_VELOCITY:
        {
            fp32 velocity_data[3];
            get_world_velocity(velocity_data); // Get world frame velocity

            memcpy(&ackBuf[1], velocity_data, 3 * sizeof(fp32));
            fAppendSendDelay = 1; // send-delay is written just before transmit
            break;
        }

        case MSG_CV_IMU_POSITION:
        {
            fp32 position_data[3];
            get_world_position(position_data); // Get world frame position

            memcpy(&ackBuf[1], position_data, 3 * sizeof(fp32));
            fAppendSendDelay = 1; // send-delay is written just before transmit
            break;
        }

		case MSG_CV_INFO_GIMBAL_ANGLE:
		{
			fp32 pitch_angle = get_gimbal_absolute_pitch_angle();
			fp32 yaw_angle = get_gimbal_absolute_yaw_angle(); // IMU-based absolute gimbal angles
			fp32 pitch_rate = get_gimbal_pitch_rate();
			fp32 yaw_rate = get_gimbal_yaw_rate(); // gimbal angular rates (rad/s)

			memcpy(&ackBuf[1], &pitch_angle, sizeof(fp32));
			memcpy(&ackBuf[1 + sizeof(fp32)], &yaw_angle, sizeof(fp32));
			memcpy(&ackBuf[1 + 2 * sizeof(fp32)], &pitch_rate, sizeof(fp32));
			memcpy(&ackBuf[1 + 3 * sizeof(fp32)], &yaw_rate, sizeof(fp32));
			break;
		}
	}


    // Write the send-delay (microseconds) as the trailing field of the value region, measured as close
    // as possible to the actual UART transmit so it reflects the true request-to-send latency
    if (fAppendSendDelay)
    {
        uint32_t ulImuSendDelay = (DWT->CYCCNT - ulCvRxTimestamp) / (SystemCoreClock / 1000000U);
        memcpy(&ackBuf[1 + valueLen - sizeof(uint32_t)], &ulImuSendDelay, sizeof(uint32_t));
    }

    // append CRC8 over [Tag][Value...]; the CRC occupies one extra trailing byte
    append_CRC8_check_sum(ackBuf, (uint16_t)valueLen + 1 + 1);

    HAL_UART_Transmit(&huart1, ackBuf, (uint16_t)valueLen + 1 + 1, 100);
}

uint8_t cv_tag_request;
static void CvCmder_RxParserTlv(const uint8_t *pData, uint16_t size)
{
    while (size >= 2)	// smallest TV frame is [Tag][CRC8] (zero-length value)
    {
        uint8_t  tag = pData[0];
		cv_tag_request = tag;
        uint8_t  length = CvCmder_GetReqValueLen(tag); // value length is derived from the Tag, not sent on the wire
        if (length == CV_REQ_LEN_UNKNOWN)
            break; // unknown Tag: without a Length byte the frame size is unknown, so we cannot resync

        uint16_t frameSize = (uint16_t)1 + length + 1; // [Tag] + [Value...] + [CRC8]
        if (size < frameSize)
            break; // incomplete packet (need value bytes plus the trailing CRC8)

        // verify CRC8 over [Tag][Value][CRC8]; drop the (suspect) frame on mismatch
        if (!verify_CRC8_check_sum((unsigned char *)pData, frameSize))
        {
            break;
        }

        // A CRC-valid frame proves the CV link is alive: refresh the CV_TOE timestamp here so the
        // online status no longer depends on the tag matching a specific handler (or its length check)
        detect_hook(CV_TOE);

        // Debug: count each CRC-valid request per Tag so the task loop can derive its update rate (Hz)
        if (tag < CV_NUM_REQ_TYPES)
        {
            cv_request_count[tag]++;
        }

        switch (tag)
        {
        	case MSG_CHECK_STATE:
			{
        	    if (length == 1)
        	    {
        	        // pData[1] = state enum
        	        // TODO: handle state
					CvCmdHandler.CvCmdMsg.cv_info_type = pData[1];
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
		        	memcpy(&xSpeed, &pData[1], 4);
		        	memcpy(&ySpeed, &pData[5], 4);

		        	if (!CvSpeedFilter.fInitialized)
		        	{
		        		CvSpeedFilter.xSpeed = xSpeed;
		        		CvSpeedFilter.ySpeed = ySpeed;
		        		CvSpeedFilter.fInitialized = 1;
		        	}

		        	CvSpeedFilter.xSpeed = xSpeed = first_order_filter(xSpeed, CvSpeedFilter.xSpeed, CV_SPEED_FILTER_ALPHA);
		        	CvSpeedFilter.ySpeed = ySpeed = first_order_filter(ySpeed, CvSpeedFilter.ySpeed, CV_SPEED_FILTER_ALPHA);
#if DEBUG_CV
						CvCmdHandler.CvCmdMsg.xSpeed = xSpeed;
						CvCmdHandler.CvCmdMsg.ySpeed = ySpeed;

#else
					if (is_game_started()){
							CvCmdHandler.CvCmdMsg.xSpeed = xSpeed;
							CvCmdHandler.CvCmdMsg.ySpeed = ySpeed;
					}
					else{
						CvCmdHandler.CvCmdMsg.xSpeed = 0.0f;
						CvCmdHandler.CvCmdMsg.ySpeed = 0.0f;
					}
#endif
					CvCmder_ChangeMode(CV_MODE_AUTO_MOVE_BIT, 1);
					CvCmder_SendAck(MSG_CV_CHASSIS_MOVE_STATE);
					detect_hook(CV_TOE);
        	    }


        	    break;
			}
        	case MSG_CONTROL_SPINNNG:
			{
        	    if (length == 1)
        	    {
        	        uint8_t spinCmd = pData[1]; // 0x00 or 0xFF
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
        	        memcpy(&xError, &pData[1], 4);
        	        memcpy(&yError, &pData[5], 4);
#if DEBUG_CV
					CvCmdHandler.CvCmdMsg.xAimError = xError;
					CvCmdHandler.CvCmdMsg.yAimError = yError;
#else
					if(is_game_started()){
						CvCmdHandler.CvCmdMsg.xAimError = xError;
						CvCmdHandler.CvCmdMsg.yAimError = yError;
					}
					else{
						CvCmdHandler.CvCmdMsg.xAimError = 0.0f;
						CvCmdHandler.CvCmdMsg.yAimError = 0.0f;
					}
#endif
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
//setting shoot flag for automatic robots					
#if (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI)
					uint8_t shootCmd = pData[1];
	#if !DEBUG_CV
					if((shootCmd == 0xFF) && (projectile_allowance_17mm > 0) &&  ((shoot_heat-30)< shoot_heat_limit) && is_game_started()){
	#else
					if((shootCmd == 0xFF)){
	#endif
						CvCmder_ChangeMode(CV_MODE_SHOOT_BIT, 1);
						CvCmder_ChangeMode(CV_MODE_CHASSIS_SPINNING_BIT, 1);
						CvCmdHandler.ulShootStartTime = osKernelSysTick();
						CvCmdHandler.ulChassisSpinStartTime = CvCmdHandler.ulShootStartTime;
					} else {
						CvCmder_ChangeMode(CV_MODE_SHOOT_BIT, 0);
					}
#endif
					CvCmder_SendAck(MSG_SHOOT_CMD);
					detect_hook(CV_TOE);
				}

				break;
			}

			case MSG_CV_INFO_GIMBAL_ANGLE:
			{
				CvCmder_SendAck(MSG_CV_INFO_GIMBAL_ANGLE);
				detect_hook(CV_TOE);
				break;
			}

			case MSG_CV_IMU_ACCELE:
			{
				CvCmder_SendAck(MSG_CV_IMU_ACCELE);
				detect_hook(CV_TOE);
				break;
			}

			case MSG_CV_IMU_VELOCITY:
			{
				CvCmder_SendAck(MSG_CV_IMU_VELOCITY);
				detect_hook(CV_TOE);
				break;
			}

			case MSG_CV_IMU_POSITION:
			{
				CvCmder_SendAck(MSG_CV_IMU_POSITION);
				detect_hook(CV_TOE);
				break;
			}

        	default:
			{
        	    // known tag without a dedicated handler
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
        pData += frameSize;
        size  -= frameSize;
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

#if !CV_INTERFACE
tCvCmdHandler CvCmdHandler;
#endif

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
#if CV_INTERFACE
        // timestamp the moment the CV request frame arrives, used to compute the IMU send delay (microseconds)
        ulCvRxTimestamp = DWT->CYCCNT;
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
