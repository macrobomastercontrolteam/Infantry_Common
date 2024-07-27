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
// #include "voltage_task.h"


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
#if defined(DEBUG_CV)
    UNUSED(status);
#else
        osDelay(1000);
        usb_printf(
"******************************\r\n\
joint motor 0:%s\r\n\
joint motor 1:%s\r\n\
joint motor 2:%s\r\n\
joint motor 3:%s\r\n\
joint motor 4:%s\r\n\
joint motor 5:%s\r\n\
joint motor 6:%s\r\n\
******************************\r\n",
            status[error_list_usb_local[JOINT_0_TOE].error_exist],
            status[error_list_usb_local[JOINT_1_TOE].error_exist],
            status[error_list_usb_local[JOINT_2_TOE].error_exist],
            status[error_list_usb_local[JOINT_3_TOE].error_exist],
            status[error_list_usb_local[JOINT_4_TOE].error_exist],
            status[error_list_usb_local[JOINT_5_TOE].error_exist],
            status[error_list_usb_local[JOINT_6_TOE].error_exist]);
#endif // defined(DEBUG_CV)
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
