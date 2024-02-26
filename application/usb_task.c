/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb ‰≥ˆ¥ÌŒÛ–≈œ¢
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"


void usb_printf(const char *fmt,...);

static uint8_t usb_buf[400];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;



void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();


    while(1)
    {
#if DEBUG_CV_WITH_USB
    UNUSED(status);
#else
        osDelay(1000);
		usb_printf(
#if (ROBOT_TYPE == ENGINEER_2024_MECANUM)
"******************************\r\n\
voltage percentage:%d%% \r\n\
DBUS:%s\r\n\
chassis drive motor1:%s\r\n\
chassis drive motor2:%s\r\n\
chassis drive motor3:%s\r\n\
chassis drive motor4:%s\r\n\
gimbal control board:%s\r\n\
gyro sensor:%s\r\n\
accel sensor:%s\r\n\
mag sensor:%s\r\n\
referee usart:%s\r\n\
cv usart:%s\r\n\
******************************\r\n",
#else
"******************************\r\n\
voltage percentage:%d%% \r\n\
DBUS:%s\r\n\
chassis drive motor1:%s\r\n\
chassis drive motor2:%s\r\n\
chassis drive motor3:%s\r\n\
chassis drive motor4:%s\r\n\
yaw motor:%s\r\n\
pitch motor:%s\r\n\
trigger motor:%s\r\n\
gyro sensor:%s\r\n\
accel sensor:%s\r\n\
mag sensor:%s\r\n\
referee usart:%s\r\n\
cv usart:%s\r\n\
******************************\r\n",
#endif
			get_battery_percentage(),
			status[error_list_usb_local[DBUS_TOE].error_exist],
			status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
			status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
			status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
			status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
#if (ROBOT_TYPE == ENGINEER_2024_MECANUM)
			status[error_list_usb_local[GIMBAL_CONTROL_BOARD_TOE].error_exist],
#else
			status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
			status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
			status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
#endif
			status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
			status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
			status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
			status[error_list_usb_local[REFEREE_TOE].error_exist],
			status[error_list_usb_local[CV_TOE].error_exist]);
#endif // DEBUG_CV_WITH_USB
    }

}

void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    // Warning: len is the length of the string without null terminator if it were to be fully copied to buffer
    len = vsnprintf((char *)usb_buf, sizeof(usb_buf), fmt, ap);

    va_end(ap);

    if (len > 0)
    {
        CDC_Transmit_FS(usb_buf, MIN(len, sizeof(usb_buf) - 1));
    }
    else
    {
        Error_Handler();
    }
}
