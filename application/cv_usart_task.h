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

#define CV_CONTROL_TIME_MS 500.0f

#if CV_INTERFACE

typedef struct __attribute__((packed))
{
    fp32 xAimError; ///< unit: rad
    fp32 yAimError; ///< unit: rad
    fp32 xSpeed;
    fp32 ySpeed;
    uint8_t cv_info_type;
} tCvCmdMsg;

typedef enum
{
    CV_MODE_AUTO_AIM_BIT = 1 << 0,
    CV_MODE_AUTO_MOVE_BIT = 1 << 1,
    CV_MODE_ENEMY_DETECTED_BIT = 1 << 2,
    CV_MODE_SHOOT_BIT = 1 << 3,
    CV_MODE_CHASSIS_SPINNING_BIT = 1 << 4,
    CV_MODE_CHASSIS_ALIGN_TO_IMU_FRONT_BIT = 1 << 5,
    CV_MODE_LAST_BIT = 1 << 6,
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
void CvCmder_ToggleMode(uint8_t bCvModeBit);
void CvCmder_ChangeMode(uint8_t bCvModeBit, uint8_t fFlag);
tCvCmdHandler* CvCmder_GetHandler(void);
void CvCmder_DetectAutoAimSwitchEdge(uint8_t fRcCmd);
void CvCmder_toe_solve_lost_fun(void);
#if DEBUG_CV_WITH_USB
uint8_t CvCmder_CheckAndResetUserKeyEdge(void);
#endif // DEBUG_CV_WITH_USB

extern tCvCmdHandler CvCmdHandler;

#endif // CV_INTERFACE

#endif // CV_USART_TASK_H
