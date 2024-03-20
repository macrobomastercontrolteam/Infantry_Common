typedef float fp32;

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "C:\Users\MB\OneDrive - McMaster University\Desktop\RoboMaster\Infantry_Common\application\gimbal_task.h"
#include "C:\Users\MB\OneDrive - McMaster University\Desktop\RoboMaster\Infantry_Common\application\usb_task.h"

extern gimbal_control_t gimbal_control;

uint16_t time;
fp32 yaw_angle;
fp32 pitch_angle;

int main() {
    time = 0;
    yaw_angle = 0;
    pitch_angle = 0;
    gimbal_init(&gimbal_control);
    while(1) {
        gimbal_feedback_update(&gimbal_control);
        usb_task("hi");
        time += 1;
        yaw_angle += 2.0f;
        pitch_angle += 3.0f;
    }
}
