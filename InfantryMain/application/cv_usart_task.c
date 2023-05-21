/**
 * @file       cv_usart_task.c
 * @brief      Computer vision communication
 * @arthur     2022 MacFalcons Control Team
 * All UART packets from CV should have the same size (DATA_PACKAGE_SIZE). Pad packet payloads with CHAR_UNUSED in the end if necessary.
 */

#if defined(CV_INTERFACE)

#include "cv_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"
#include "bsp_usart.h"
#include "string.h"
#if defined(DEBUG_CV)
#include "usb_task.h"
#include <stdio.h>
#endif

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif /* MIN */
#define DATA_PACKAGE_SIZE 15
#define DATA_PACKAGE_PAYLOAD_SIZE (DATA_PACKAGE_SIZE - sizeof(uint8_t) * 3)

typedef enum
{
    MSG_MODE_CONTROL = 0x10,
    MSG_CV_CMD = 0x20,
    MSG_ACK = 0x40,
} eMsgTypes;

typedef enum
{
    CHAR_STX = 0x02,
    CHAR_ETX = 0x03,
    CHAR_UNUSED = 0xFF,
} eCharTypes;

typedef enum
{
    CV_MODE_AUTO_AIM_BIT = 1 << 0,
    CV_MODE_AUTO_MOVE_BIT = 1 << 1,
    CV_MODE_ENEMY_DETECTED_BIT = 1 << 2,
} eModeControlBits;

typedef union
{
    struct
    {
        uint8_t bStx; ///< always CHAR_STX
        uint8_t bMsgType;
        uint8_t abPayload[DATA_PACKAGE_PAYLOAD_SIZE];
        uint8_t bEtx; ///< always CHAR_ETX
    } tData;
    uint8_t abData[DATA_PACKAGE_SIZE];
} tCvMsg;

void CvCmder_Init(void);
void CvCmder_PollForModeChange(void);
void CvCmder_RxParser(uint16_t Size);
void CvCmder_SendSetModeRequest(void);
void CvCmder_ChangeMode(uint8_t bCvModeBit, uint8_t fFlag);
#if defined(DEBUG_CV)
uint8_t CvCmder_MockModeChange(void);
#endif

tCvMsg CvRxBuffer;
tCvMsg CvTxBuffer;
tCvCmdHandler CvCmdHandler;
// don't compare with literal string "ACK", since it contains extra NULL char at the end
const uint8_t abExpectedAckPayload[3] = {'A', 'C', 'K'};
uint8_t abUsartRxBuf[DATA_PACKAGE_PAYLOAD_SIZE * 3];
uint8_t abExpectedUnusedPayload[DATA_PACKAGE_PAYLOAD_SIZE - 1];

#if defined(DEBUG_CV)
volatile uint8_t fIsUserKeyPressingEdge = 0;
char usbMsg[500];
volatile uint16_t uiUsbMsgSize;
#endif

void cv_usart_task(void const *argument)
{
    CvCmder_Init();
    while (1)
    {
        CvCmder_PollForModeChange();
        osDelay(500);
    }
}

void CvCmder_Init(void)
{
    CvTxBuffer.tData.bStx = CHAR_STX;
    CvTxBuffer.tData.bEtx = CHAR_ETX;
    memset(abExpectedUnusedPayload, CHAR_UNUSED, sizeof(abExpectedUnusedPayload));

    memset(&CvCmdHandler, 0, sizeof(CvCmdHandler)); // clear status
    CvCmdHandler.cv_rc_ctrl = get_remote_control_point();

    // Get a callback when DMA completes or IDLE
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, abUsartRxBuf, sizeof(abUsartRxBuf));
    // disable half transfer interrupt, because if it coincide with IDLE or DMA interrupt, callback will be called twice
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    // RXNE is not used
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
}

/**
 * @brief if a command is received from remote controller, we keep sending set-mode requests to CV until an ACK is received
 */
void CvCmder_PollForModeChange(void)
{
    // @TODO: Implement check for Auto-move mode and Enemy mode
    // uint8_t fLastAutoMoveMode = CvCmder_GetMode(CV_MODE_AUTO_MOVE_BIT);
    // uint8_t fLastEnemyMode = CvCmder_GetMode(CV_MODE_ENEMY_DETECTED_BIT);

    uint8_t fIsModeChanged = 0;
    if (CvCmdHandler.fIsAutoAimSwitchEdge != EDGE_NONE)
    {
        CvCmder_ChangeMode(CV_MODE_AUTO_AIM_BIT, (CvCmdHandler.fIsAutoAimSwitchEdge == EDGE_RISING));
        fIsModeChanged = 1;
    }

#if defined(DEBUG_CV)
    if (fIsModeChanged == 0)
    {
        fIsModeChanged = CvCmder_MockModeChange();
    }
#endif

    if (fIsModeChanged)
    {
        CvCmder_SendSetModeRequest();
    }
}

void CvCmder_DetectAutoAimSwitchEdge(uint8_t fIsKeyPressed)
{
    // no need to debounce because keyboard signal is clean
    static uint8_t fLastKeySignal = 0;
    if (fLastKeySignal != fIsKeyPressed)
    {
        if (fIsKeyPressed)
        {
            CvCmdHandler.fIsAutoAimSwitchEdge = CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT) ? EDGE_FALLING : EDGE_RISING;
        }
        fLastKeySignal = fIsKeyPressed;
    }
}

void CvCmder_SendSetModeRequest(void)
{
    CvTxBuffer.tData.bMsgType = MSG_MODE_CONTROL;
    memset(CvTxBuffer.tData.abPayload, CHAR_UNUSED, DATA_PACKAGE_PAYLOAD_SIZE);
    CvTxBuffer.tData.abPayload[0] = CvCmdHandler.fCvMode;
    HAL_UART_Transmit(&huart1, CvTxBuffer.abData, sizeof(CvTxBuffer.abData), 100);

    CvCmdHandler.fCvCmdValid = CvCmdHandler.fCvCmdValid && (CvCmder_GetMode(CV_MODE_AUTO_MOVE_BIT) || CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT));
    CvCmdHandler.fIsWaitingForAck = 1;

#if defined(DEBUG_CV)
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        uint16_t uiCursor = 0;
        uint16_t uiPacketSize;
        while (uiCursor < Size)
        {
            uiPacketSize = MIN(Size - uiCursor, sizeof(CvRxBuffer.abData));
            memcpy(CvRxBuffer.abData, &abUsartRxBuf[uiCursor], uiPacketSize);
            CvCmder_RxParser(uiPacketSize);
            uiCursor += sizeof(CvRxBuffer.abData);
        }

        /* start the DMA again */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, abUsartRxBuf, sizeof(abUsartRxBuf));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void CvCmder_RxParser(uint16_t uiPacketSize)
{
    uint8_t fInvalid = 1;
    if ((uiPacketSize == sizeof(CvRxBuffer.abData)) && (CvRxBuffer.tData.bStx == CHAR_STX) && (CvRxBuffer.tData.bEtx == CHAR_ETX))
    {
        switch (CvRxBuffer.tData.bMsgType)
        {
        case MSG_CV_CMD:
        {
            fInvalid = 0;
            fInvalid |= ((CvCmder_GetMode(CV_MODE_AUTO_MOVE_BIT) == 0) && (CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT) == 0));
            // // Check for ACK is not used, because it may be misaligned in the middle and ignored so that further msgs align
            // // However, ACK is still helpful because it can help synchronize communication in the beginning
            // fInvalid |= CvCmdHandler.fIsWaitingForAck;
            fInvalid |= (memcmp(&CvRxBuffer.tData.abPayload[sizeof(tCvCmdMsg)], abExpectedUnusedPayload, DATA_PACKAGE_PAYLOAD_SIZE - sizeof(tCvCmdMsg)) != 0);
            if (fInvalid == 0)
            {
                memcpy(&(CvCmdHandler.CvCmdMsg), CvRxBuffer.tData.abPayload, sizeof(CvCmdHandler.CvCmdMsg));
                CvCmdHandler.fCvCmdValid = 1;
            }
            else
            {
                CvCmdHandler.fCvCmdValid = 0;
            }
            break;
        }
        case MSG_ACK:
        {
            fInvalid = 0;
            fInvalid |= (memcmp(&CvRxBuffer.tData.abPayload[sizeof(abExpectedAckPayload)], abExpectedUnusedPayload, DATA_PACKAGE_PAYLOAD_SIZE - sizeof(abExpectedAckPayload)) != 0);
            fInvalid |= (memcmp(CvRxBuffer.tData.abPayload, abExpectedAckPayload, sizeof(abExpectedAckPayload)) != 0);
            if (fInvalid == 0)
            {
                CvCmdHandler.fIsWaitingForAck = 0;
                CvCmdHandler.fIsAutoAimSwitchEdge = EDGE_NONE;
            }
            break;
        }
        default:
        {
            // fInvalid = 1; // by default
            break;
        }
        }
    }

    if (fInvalid)
    {
        // reserved
    }

#if defined(DEBUG_CV)
    // echo to usb
    uiUsbMsgSize = snprintf(usbMsg, sizeof(usbMsg), "Received: (%s) ", fInvalid ? "Invalid" : "Valid");

    for (uint16_t i = 0; i < uiPacketSize; i++)
    {
        if (uiUsbMsgSize + (sizeof("0x00,") - 1) > sizeof(usbMsg))
        {
            break;
        }
        uiUsbMsgSize += snprintf(&usbMsg[uiUsbMsgSize], sizeof(usbMsg) - uiUsbMsgSize, "0x%02X,", CvRxBuffer.abData[i]);
    }
    usbMsg[uiUsbMsgSize - 1] = '\n';
    usbMsg[uiUsbMsgSize] = 0;
    usb_printf("%s", usbMsg);
#endif
}

uint8_t CvCmder_GetMode(uint8_t bCvModeBit)
{
    return (CvCmdHandler.fCvMode & bCvModeBit);
}

void CvCmder_ChangeMode(uint8_t bCvModeBit, uint8_t fFlag)
{
    CvCmdHandler.fCvMode = (CvCmdHandler.fCvMode & ~bCvModeBit) | (fFlag ? bCvModeBit : 0);
}

/**
 * @brief Get pointer to Cv Cmd Handler
 * TODO: use it in gimbal_task and chassis_task
 */
tCvCmdHandler *CvCmder_GetHandler(void)
{
    return &CvCmdHandler;
}

#if defined(DEBUG_CV)
uint8_t CvCmder_CheckAndResetUserKeyEdge(void)
{
    uint8_t temp = fIsUserKeyPressingEdge;
    fIsUserKeyPressingEdge = 0;
    return temp;
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
            }
            bMockCounter = (bMockCounter + 1) % 4;
        }
    }
    return fIsUserKeyPressingEdge;
}
#endif // defined(DEBUG_CV)

#endif // defined(CV_INTERFACE)
