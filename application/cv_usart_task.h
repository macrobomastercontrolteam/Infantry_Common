/**
 * @file       cv_usart_task.h
 * @brief      Computer vision communication
 * @arthur     2022 MacFalcons Control Team
 */
#ifndef CV_USART_TASK_H
#define CV_USART_TASK_H

#include "global_inc.h"
#include "main.h"
#include "user_lib.h" // moving_average_type_t, STATIC_ASSERT
#include "remote_control.h"

#if CV_INTERFACE

typedef struct __attribute__((packed))
{
    fp32 xTargetAngle; ///< unit: rad
    fp32 yTargetAngle; ///< unit: rad
    fp32 xSpeed;
    fp32 ySpeed;
} tCvCmdMsg;

typedef enum
{
    CV_MODE_AUTO_AIM_BIT = 1 << 0,
    CV_MODE_AUTO_MOVE_BIT = 1 << 1,
    CV_MODE_ENEMY_DETECTED_BIT = 1 << 2,
    CV_MODE_SHOOT_BIT = 1 << 3,
    CV_MODE_IMU_CALI_BIT = 1 << 4, ///< 1 means IMU calibration is not done yet, and gimbal is powered to held at a fixed center position
    CV_MODE_LAST_BIT = 1 << 5,
} eModeControlBits;
STATIC_ASSERT(CV_MODE_LAST_BIT <= (1 << 8));

/**
 * @brief main handler of communication status and commands received from CV
 */
typedef struct
{
    tCvCmdMsg CvCmdMsg;
    uint8_t fCvCmdValid; ///< to be used by gimbal_task, but not chassis_task. chassis_task should maintain previous speed if cv is offline for a short time
    uint8_t fIsWaitingForAck;
    uint8_t fCvMode; ///< contains individual CV control flag bits defined by eModeControlBits
    uint8_t fIsModeChanged;
    uint32_t ulShootStartTime;
    const RC_ctrl_t *cv_rc_ctrl; ///< remote control pointer
} tCvCmdHandler;

void cv_usart_task(void const *argument);
uint8_t CvCmder_GetMode(uint8_t bCvModeBit);
tCvCmdHandler* CvCmder_GetHandler(void);
void CvCmder_DetectAutoAimSwitchEdge(uint8_t fRcCmd);
#if DEBUG_CV_WITH_USB
uint8_t CvCmder_CheckAndResetUserKeyEdge(void);
#endif // DEBUG_CV_WITH_USB

extern tCvCmdHandler CvCmdHandler;

#endif // CV_INTERFACE

#endif // CV_USART_TASK_H
