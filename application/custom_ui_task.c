/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      Chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================  
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "custom_ui_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "referee.h"
#include "graphic.h"
#include "gimbal_task.h"
#include "shoot.h"
#include "gimbal_behaviour.h"

/**
 * @brief
 *
 * @param[out]
 * @retval
 */

graphic_data_struct_t barrel_dir;
graphic_data_struct_t chassis_dir;
string_data chassis_angle;
string_data gimbal_dir_0;
string_data gimbal_angle;
string_data cap_voltage_data;
string_data cap_power_data;
string_data cap_voltage;
string_data cap_power;
string_data chassis_front_dir;
graphic_data_struct_t armor_0;
graphic_data_struct_t armor_1;
graphic_data_struct_t armor_2;
graphic_data_struct_t armor_3;
graphic_data_struct_t G1;
graphic_data_struct_t G2;
graphic_data_struct_t G3;
graphic_data_struct_t G4;
graphic_data_struct_t G5;
graphic_data_struct_t G6;
string_data trigger_speed;
string_data trigger_speed_data;
string_data robot_status_str;
string_data logo;

float IMU_data;
float trigger_motor_rpm = 0.00f;

    
armor_damage_info armor_damage_judge(void);
void text_message_init(void);
void super_cap_status_draw(void);
void chassis_direction_draw(float yaw_relative_angle);
void gimbal_pitch_direction_draw(float pitch_relative_angle);
void armor_damage_draw(float yaw_relative_angle);
void chassis_mode(void);
void trigger_motor_state(float trigger_rpm);

void custom_ui_task(void const *argument)
{
    uint32_t ulSystemTime = osKernelSysTick();
    for(int i = 0; i <= 30; i++){
       static_elements_init();
    }
    
    while (1)
    {
        trigger_motor_rpm = shoot_control.speed;
        const gimbal_motor_t *yaw_motor_feedback = get_yaw_motor_point();
        const fp32 pitch_angle = gimbal_control.gimbal_pitch_motor.absolute_angle;
        trigger_motor_state(trigger_motor_rpm);
        chassis_direction_draw(yaw_motor_feedback->relative_angle);
        gimbal_pitch_direction_draw(pitch_angle);
        armor_damage_draw(yaw_motor_feedback->relative_angle);
        super_cap_status_draw();
		chassis_mode();
        osDelayUntil(&ulSystemTime, CUSTOM_UI_TIME_MS);
    }
    
}

armor_damage_info armor_damage_judge(void)
{
    uint8_t armor_id = get_armor_hurt();
    if (armor_id == 0) {
        return ARMOR_ZERO;
    }
    else if (armor_id == 1) {
        return ARMOR_ONE;
    }
    else if (armor_id == 2) {
        return ARMOR_TWO;
    }
    else if (armor_id == 3) {
        return ARMOR_THREE;
    }
    return NONE;
}

void static_elements_init(void)
{
    char_draw(&cap_voltage, "capVlotageStr", UI_Graph_ADD, 1, UI_Color_Pink, 20, 4, 3, 1473, 468,"CAPV");
    update_char(&cap_voltage);
    char_draw(&logo, "macfalcons", UI_Graph_ADD, 1, UI_Color_Pink, 20, 11, 3, 850, 60,"MACFALCONS");
    update_char(&logo);
    char_draw(&cap_power, "capPowerStr", UI_Graph_ADD, 0, UI_Color_Pink, 20, 4, 3, 1473, 428,"CAPP");
    update_char(&cap_power);
    float_draw(&cap_voltage_data, "capVoltageData", UI_Graph_ADD, 1, UI_Color_Cyan, 20, 4, 3, 1590, 468,(float)(cap_message_rx.cap_message.cap_voltage/1000.0f));
    update_char(&cap_voltage_data);
    float_draw(&cap_power_data, "capPower", UI_Graph_ADD, 0, UI_Color_Cyan, 20, 3, 1, 1590, 3888,cap_message_rx.cap_message.cap_power);
    update_char(&cap_power_data);
    float_draw(&trigger_speed_data, "triggerSpeedData", UI_Graph_ADD, 7, UI_Color_Cyan, 20, 4, 3, 1590, 508, trigger_motor_rpm);
    update_char(&trigger_speed_data);
    char_draw(&trigger_speed, "triggerSpeed", UI_Graph_ADD, 0, UI_Color_Pink, 20, 6, 3, 1473, 508,"TRIRPM");
    update_char(&trigger_speed);
    char_draw(&robot_status_str, "robot_status_str", UI_Graph_ADD, 8, UI_Color_Pink, 20, 4, 3, 930, 227,"SPIN");
    update_char(&robot_status_str);   
    line_draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Cyan,2,960,330,960,620);
    update_ui(&G1);
	line_draw(&G2,"092",UI_Graph_ADD,9,UI_Color_Cyan,2,880,580,1040,580);
    update_ui(&G2);
	line_draw(&G3,"093",UI_Graph_ADD,9,UI_Color_Cyan,2,800,540,1120,540);
    update_ui(&G3);
	line_draw(&G4,"094",UI_Graph_ADD,9,UI_Color_Cyan,2,880,500,1040,500);
    update_ui(&G4);
	line_draw(&G5,"095",UI_Graph_ADD,9,UI_Color_Cyan,2,900,420,1020,420);
    update_ui(&G5);
	line_draw(&G6,"096",UI_Graph_ADD,9,UI_Color_Cyan,2,920,370,1000,370);
    update_ui(&G6);

}

void chassis_direction_draw(float yaw_relative_angle)
{
    char_draw(&chassis_front_dir, "gimbal_dir_0", UI_Graph_ADD, 3, UI_Color_Orange, 30, 1, 5, 960  - arm_cos_f32(yaw_relative_angle + PI/2) * 100, 560 + arm_sin_f32(yaw_relative_angle + PI/2) * 100, "X");
    update_char(&chassis_front_dir);
    float_draw(&chassis_angle, "chassis_angle_rad", UI_Graph_ADD, 7, UI_Color_Orange, 16, 4, 3, 1200, 600, ((yaw_relative_angle*180.0f)/PI));
    update_char(&chassis_angle);
    float_draw(&chassis_angle, "chassis_angle_rad", UI_Graph_Change, 7, UI_Color_Orange, 16, 4, 3, 1200, 600, ((yaw_relative_angle*180.0f)/PI));
    update_char(&chassis_angle);
    char_draw(&chassis_front_dir, "gimbal_dir_0", UI_Graph_Change, 3, UI_Color_Orange, 30, 1, 5, 960  - arm_cos_f32(yaw_relative_angle + PI/2) * 100, 560 + arm_sin_f32(yaw_relative_angle + PI/2) * 100, "X");
    update_char(&chassis_front_dir);
}

void gimbal_pitch_direction_draw(float pitch_relative_angle)
{
    IMU_data = pitch_relative_angle;
    char_draw(&gimbal_dir_0, "gimbal_dir_0", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, 1, 5, 1000, 540 + (pitch_relative_angle * 300), "<");
    update_char(&gimbal_dir_0);
    float_draw(&gimbal_angle, "pitch_angle_rad", UI_Graph_ADD, 3, UI_Color_Purplish_red, 16, 4, 3, 1200, 540, (pitch_relative_angle));
    update_char(&gimbal_angle);
    char_draw(&gimbal_dir_0, "gimbal_dir_0", UI_Graph_Change, 5, UI_Color_Purplish_red, 20, 1, 5, 1000, 540 + (pitch_relative_angle * 300), "<");
    update_char(&gimbal_dir_0);
    float_draw(&gimbal_angle, "pitch_angle_rad", UI_Graph_Change, 3, UI_Color_Purplish_red, 16, 4, 3, 1200, 540, (-pitch_relative_angle*180.0f/PI));
    update_char(&gimbal_angle);
}

void armor_damage_draw(float yaw_relative_angle)
{
    int i;
    armor_damage_info ARMOR_NUM;
    ARMOR_NUM = armor_damage_judge();
    
    switch (ARMOR_NUM)
    {
    case ARMOR_ZERO:
        for(i = 0; i <= 30; i++){
            circle_draw(&armor_0, "front_armor", UI_Graph_ADD, 2, UI_Color_Purplish_red, 7, 960 + arm_cos_f32(yaw_relative_angle + PI/2) * 110, 560 + arm_sin_f32(yaw_relative_angle + PI/2) * 110, 20);
            update_ui(&armor_0);
        }
        break;
    case ARMOR_ONE:
        for(i = 0; i <= 30; i++){
            circle_draw(&armor_1, "right_armor", UI_Graph_ADD, 2, UI_Color_Purplish_red, 7, 960 + arm_cos_f32(yaw_relative_angle + 2*PI/2) * 110, 560 + arm_sin_f32(yaw_relative_angle + 2*PI/2) * 110, 20);
            update_ui(&armor_1);
        }
        break;
    case ARMOR_TWO:
        for(i = 0; i <= 30; i++){
            circle_draw(&armor_2, "back_armor", UI_Graph_ADD, 2, UI_Color_Purplish_red, 7, 960 + arm_cos_f32(yaw_relative_angle + 3*PI/2) * 110, 560 + arm_sin_f32(yaw_relative_angle + 3*PI/2) * 110, 20);
            update_ui(&armor_2);
        }
        break;
    case ARMOR_THREE:
        for(i = 0; i <= 30; i++){
            circle_draw(&armor_3, "left_armor", UI_Graph_ADD, 2, UI_Color_Purplish_red, 7, 960 + arm_cos_f32(yaw_relative_angle + 4*PI/2) * 110, 560 + arm_sin_f32(yaw_relative_angle + 4*PI/2) * 110, 20);
            update_ui(&armor_3);
        }
        break;
    default:
        osDelay(3);
        circle_draw(&armor_3, "left_armor", UI_Graph_Del, 2, UI_Color_Cyan, 7, 960 + arm_cos_f32(yaw_relative_angle + 4*PI/2) * 110, 560 + arm_sin_f32(yaw_relative_angle + 4*PI/2) * 110, 20);
        update_ui(&armor_3);
        circle_draw(&armor_2, "back_armor", UI_Graph_Del, 2, UI_Color_Cyan, 7, 960 + arm_cos_f32(yaw_relative_angle + 3*PI/2) * 110, 560 + arm_sin_f32(yaw_relative_angle + 3*PI/2) * 110, 20);
        update_ui(&armor_2);
        circle_draw(&armor_1, "right_armor", UI_Graph_Del, 2, UI_Color_Cyan, 7, 960 + arm_cos_f32(yaw_relative_angle + 2*PI/2) * 110, 560 + arm_sin_f32(yaw_relative_angle + 2*PI/2) * 110, 20);
        update_ui(&armor_1);
        circle_draw(&armor_0, "front_armor", UI_Graph_Del, 2, UI_Color_Cyan, 7, 960 + arm_cos_f32(yaw_relative_angle + PI/2) * 110, 560 + arm_sin_f32(yaw_relative_angle + PI/2) * 110, 20);
        update_ui(&armor_0);
        break;
    }
}

void super_cap_status_draw(void)
{
    if(cap_message_rx.cap_message.cap_voltage >= 10000){
        float_draw(&cap_voltage_data, "capVoltageData", UI_Graph_Change, 1, UI_Color_Cyan, 20, 4, 3, 1590, 468,(float)(cap_message_rx.cap_message.cap_voltage/1000.0f));
    }
    else if(cap_message_rx.cap_message.cap_voltage < 10000){
        float_draw(&cap_voltage_data, "capVoltageData", UI_Graph_Change, 0,  UI_Color_Purplish_red, 20, 4, 3, 1590, 468,(float)(cap_message_rx.cap_message.cap_voltage/1000.0f));
    }
    update_char(&cap_voltage_data);

    if(cap_message_rx.cap_message.cap_power < 0){
        float_draw(&cap_power_data, "capPower", UI_Graph_Change, 1, UI_Color_Cyan, 20, 5, 3, 1590, 428,cap_message_rx.cap_message.cap_power);
    }
    else if(cap_message_rx.cap_message.cap_power >= 0){
        float_draw(&cap_power_data, "capPower", UI_Graph_Change, 0, UI_Color_Purplish_red, 20, 5, 3, 1590, 428,cap_message_rx.cap_message.cap_power);
    }
    update_char(&cap_power_data);
}

void trigger_motor_state(float trigger_rpm)
{
    if(trigger_rpm >= 0.5f){
        float_draw(&trigger_speed_data, "triggerSpeedData", UI_Graph_Change, 7, UI_Color_Purplish_red, 20, 4, 3, 1590, 508, -trigger_rpm);
        update_char(&trigger_speed_data);
    }
    else{
        float_draw(&trigger_speed_data, "triggerSpeedData", UI_Graph_Change, 7, UI_Color_Cyan, 20, 4, 3, 1590, 508, -trigger_rpm);
        update_char(&trigger_speed_data);
    }
    
}

void chassis_mode(void) {
    
        switch(chassis_behaviour_mode) {
        case CHASSIS_NO_FOLLOW_YAW:
                char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227,"CNFY");
                update_char(&robot_status_str);
            break;
        case CHASSIS_SPINNING:
                char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227,"SPIN");
                update_char(&robot_status_str);       
            break;
        default:
			char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227,"STOP");
            update_char(&robot_status_str);
            break;
        }

    
}

