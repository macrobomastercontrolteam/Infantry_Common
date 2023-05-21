/**
 * @file       cv_usart_task.h
 * @brief      Computer vision communication
 * @arthur     2022 MacFalcons Control Team
 */
#ifndef CV_USART_TASK_H
#define CV_USART_TASK_H

#if defined(DEBUG_CV) && (!defined(CV_INTERFACE))
#error "DEBUG_CV cannot be defined without CV_INTERFACE"
#endif

#if defined(CV_INTERFACE)

#include "main.h"
#include "struct_typedef.h"
#include "remote_control.h"

typedef struct __attribute__((packed))
{
    uint16_t uiXCoordinate;
    uint16_t uiYCoordinate;
    fp32 xSpeed;
    fp32 ySpeed;
} tCvCmdMsg;

typedef enum
{
    EDGE_NONE,
    EDGE_RISING,
    EDGE_FALLING,
} eEdgeTypes;

/**
 * @brief main handler of communication status and commands received from CV
 */
typedef struct
{
    tCvCmdMsg CvCmdMsg;
    uint8_t fCvCmdValid; ///< whether CvCmdMsg is valid as received from CV; to be used by gimbal_task
    uint8_t fIsWaitingForAck;
    uint8_t fCvMode; ///< contains individual CV control flag bits defined by eModeControlBits
    eEdgeTypes fIsAutoAimSwitchEdge;
    const RC_ctrl_t *cv_rc_ctrl; ///< remote control pointer
} tCvCmdHandler;

void cv_usart_task(void const *argument);
uint8_t CvCmder_GetMode(uint8_t bCvModeBit);
tCvCmdHandler* CvCmder_GetHandler(void);
void CvCmder_DetectAutoAimSwitchEdge(uint8_t fRcCmd);
#if defined(DEBUG_CV)
uint8_t CvCmder_CheckAndResetUserKeyEdge(void);
#endif // defined(DEBUG_CV)

#endif // defined(CV_INTERFACE)

#endif // CV_USART_TASK_H
