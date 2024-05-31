/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      Remote control processing, the remote control is transmitted through a
  *            protocol similar to SBUS, using DMA transmission method to save CPU
  *           resources, using serial port idle interrupt to pull up the processing function,
  *         and providing some offline restart DMA, serial port
  *       to ensure the stability of hot swap.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "global_inc.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

#define JOYSTICK_RIGHT_HORIZONTAL_CHANNEL 0
#define JOYSTICK_RIGHT_VERTICAL_CHANNEL 1
#define JOYSTICK_LEFT_HORIZONTAL_CHANNEL 2
#define JOYSTICK_LEFT_VERTICAL_CHANNEL 3
#define RC_DIAL_CHANNEL 4
#define RC_RIGHT_LEVER_CHANNEL 0
#define RC_LEFT_LEVER_CHANNEL 1

#define JOYSTICK_HALF_RANGE 660.0f
#define JOYSTICK_FULL_RANGE (JOYSTICK_HALF_RANGE*2.0f)

// Measured max value that my mouse can reach by my "average-human" hand (depends on mouse but not too much)
#define MOUSE_X_MAX_SPEED 7700.0f // positive direction is to the right
#define MOUSE_Y_MAX_SPEED 1400.0f // positive direction is downwards
#define MOUSE_Z_MAX_SPEED 60.0f // positive direction is scrolling forward
// Measured speed value corresponding to move across hand available workspace (abbrev. handspace) for one second
#define MOUSE_X_HANDSPACE_PER_S 1000.0f
#define MOUSE_Y_HANDSPACE_PER_S 700.0f
#define MOUSE_Z_HANDSPACE_PER_S 40.0f
// since mouse speed appears as an arch, calculate about the "effective" speed
#define MOUSE_X_EFFECTIVE_SPEED (MOUSE_X_HANDSPACE_PER_S / 5.0f)
#define MOUSE_Y_EFFECTIVE_SPEED (MOUSE_Y_HANDSPACE_PER_S / 5.0f)

// toggle auto-aim mode
#define AUTO_AIM_TOGGLE_KEYBOARD KEY_PRESSED_OFFSET_G
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void solve_RC_lost(void);
extern void solve_data_error(void);
// extern void sbus_to_usart1(uint8_t *sbus);

extern RC_ctrl_t rc_ctrl;
#endif
