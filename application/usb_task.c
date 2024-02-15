/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usbï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï?
  * @brief      usb outputs the error message.usbï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï?
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

#include "gimbal_task.h"

extern gimbal_control_t gimbal_control;

void usb_printf(const char *fmt,...);

static uint8_t usb_buf[400];
// static const char status[2][7] = {"OK", "ERROR!"}; // Unused variable
const error_t *error_list_usb_local;



void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();


    while(1)
    {
    #if DEBUG_CV_WITH_USB
        fp32 yaw_angle_to_print = access_angle(xTaskGetTickCount(),&(gimbal_control.yaw_angle));
        fp32 pitch_angle_to_print = access_angle(xTaskGetTickCount(),&(gimbal_control.pitch_angle));


        usb_printf("yaw: %f, pitch: %f\n",yaw_angle_to_print,pitch_angle_to_print);
    #endif
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
