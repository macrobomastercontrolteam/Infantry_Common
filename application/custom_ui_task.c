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
#include "AHRS_middleware.h"
#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "global_inc.h"
#include "graphic.h"
#include "referee.h"
#include "shoot.h"

#define CUSTOM_UI_TIME_MS 10.0f

graphic_data_struct_t barrel_dir;
graphic_data_struct_t chassis_dir;
string_data chassis_relative_angle;
string_data pitch_direction_icon;
string_data pitch_angle_text;
// string_data cap_voltage_data;
// string_data cap_power_data;
// string_data cap_voltage_str;
// string_data cap_power;
string_data chassis_front_dir;
graphic_data_struct_t armor_0;
graphic_data_struct_t armor_1;
graphic_data_struct_t armor_2;
graphic_data_struct_t armor_3;
graphic_data_struct_t crosshair_vert;
graphic_data_struct_t crosshair_hori_2;
graphic_data_struct_t crosshair_hori_3;
graphic_data_struct_t crosshair_hori_4;
graphic_data_struct_t crosshair_hori_5;
graphic_data_struct_t crosshair_hori_6;
string_data trigger_speed;
string_data trigger_speed_data;
string_data rand_spin_str;
string_data robot_status_str;
string_data MacRM_logo_str;

void static_elements_init(void);
void chassis_direction_draw(float yaw_relative_angle);
void gimbal_pitch_direction_draw(float pitch_relative_angle);
void armor_damage_draw(float yaw_relative_angle);
void chassis_mode_draw(void);
void trigger_motor_state_draw(float trigger_rpm);

void custom_ui_task(void const *argument)
{
	uint32_t ulSystemTime = osKernelSysTick();
	for (uint8_t i = 0; i <= 30; i++)
	{
		static_elements_init();
	}

	while (1)
	{
		trigger_motor_state_draw(shoot_control.speed);
		chassis_direction_draw(gimbal_control.gimbal_yaw_motor.relative_angle);
		gimbal_pitch_direction_draw(gimbal_control.gimbal_pitch_motor.absolute_angle);
		armor_damage_draw(gimbal_control.gimbal_yaw_motor.relative_angle);
		//super_cap_status_draw();
		chassis_mode_draw();
		osDelayUntil(&ulSystemTime, CUSTOM_UI_TIME_MS);
	}
}

void static_elements_init(void)
{
	// char_draw(&cap_voltage_str, "capVlotageStr", UI_Graph_ADD, 1, UI_Color_Pink, 20, 4, 3, 1473, 468, "CAPV");
	// update_char(&cap_voltage_str);
	char_draw(&MacRM_logo_str, "macfalcons", UI_Graph_ADD, 1, UI_Color_Pink, 20, 11, 3, 850, 60, "MACFALCONS");
	update_char(&MacRM_logo_str);
	// char_draw(&cap_power, "capPowerStr", UI_Graph_ADD, 0, UI_Color_Pink, 20, 4, 3, 1473, 428, "CAPP");
	// update_char(&cap_power);
	// float_draw(&cap_voltage_data, "capVoltageData", UI_Graph_ADD, 1, UI_Color_Cyan, 20, 4, 3, 1590, 468, (float)cap_message_rx.cap_message.cap_milivoltage / 1000.0f);
	// update_char(&cap_voltage_data);
	// float_draw(&cap_power_data, "capPower", UI_Graph_ADD, 0, UI_Color_Cyan, 20, 3, 1, 1590, 3888, cap_message_rx.cap_message.cap_power);
	// update_char(&cap_power_data);
	float_draw(&trigger_speed_data, "triggerSpeedData", UI_Graph_ADD, 7, UI_Color_Cyan, 20, 4, 3, 1590, 508, 0);
	update_char(&trigger_speed_data);
	char_draw(&trigger_speed, "triggerSpeed", UI_Graph_ADD, 0, UI_Color_Pink, 20, 6, 3, 1473, 508, "TRIRPM");
	update_char(&trigger_speed);
	char_draw(&robot_status_str, "robot_status_str", UI_Graph_ADD, 8, UI_Color_Pink, 20, 4, 3, 930, 227, "SPIN");
	update_char(&robot_status_str);
	line_draw(&crosshair_vert, "091", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 960, 330, 960, 620);
	update_ui(&crosshair_vert);
	line_draw(&crosshair_hori_2, "092", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 880, 580, 1040, 580);
	update_ui(&crosshair_hori_2);
	line_draw(&crosshair_hori_3, "093", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 800, 540, 1120, 540);
	update_ui(&crosshair_hori_3);
	line_draw(&crosshair_hori_4, "094", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 880, 500, 1040, 500);
	update_ui(&crosshair_hori_4);
	line_draw(&crosshair_hori_5, "095", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 900, 420, 1020, 420);
	update_ui(&crosshair_hori_5);
	line_draw(&crosshair_hori_6, "096", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 920, 370, 1000, 370);
	update_ui(&crosshair_hori_6);
}

void chassis_direction_draw(float yaw_relative_angle)
{
	char_draw(&chassis_front_dir, "pitch_direction_icon", UI_Graph_ADD, 3, UI_Color_Orange, 30, 1, 5, 960 - AHRS_cosf(yaw_relative_angle + PI / 2) * 100, 560 + AHRS_sinf(yaw_relative_angle + PI / 2) * 100, "X");
	update_char(&chassis_front_dir);
	float_draw(&chassis_relative_angle, "chassis_relative_angle_rad", UI_Graph_ADD, 7, UI_Color_Orange, 16, 4, 3, 1200, 600, ((yaw_relative_angle * 180.0f) / PI));
	update_char(&chassis_relative_angle);
	float_draw(&chassis_relative_angle, "chassis_relative_angle_rad", UI_Graph_Change, 7, UI_Color_Orange, 16, 4, 3, 1200, 600, ((yaw_relative_angle * 180.0f) / PI));
	update_char(&chassis_relative_angle);
	char_draw(&chassis_front_dir, "pitch_direction_icon", UI_Graph_Change, 3, UI_Color_Orange, 30, 1, 5, 960 - AHRS_cosf(yaw_relative_angle + PI / 2) * 100, 560 + AHRS_sinf(yaw_relative_angle + PI / 2) * 100, "X");
	update_char(&chassis_front_dir);
}

void gimbal_pitch_direction_draw(float pitch_relative_angle)
{
	char_draw(&pitch_direction_icon, "pitch_direction_icon", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, 1, 5, 1000, 540 + (pitch_relative_angle * 300), "<");
	update_char(&pitch_direction_icon);
	float_draw(&pitch_angle_text, "pitch_angle_rad", UI_Graph_ADD, 3, UI_Color_Purplish_red, 16, 4, 3, 1200, 540, (pitch_relative_angle));
	update_char(&pitch_angle_text);
	char_draw(&pitch_direction_icon, "pitch_direction_icon", UI_Graph_Change, 5, UI_Color_Purplish_red, 20, 1, 5, 1000, 540 + (pitch_relative_angle * 300), "<");
	update_char(&pitch_direction_icon);
	float_draw(&pitch_angle_text, "pitch_angle_rad", UI_Graph_Change, 3, UI_Color_Purplish_red, 16, 4, 3, 1200, 540, (-pitch_relative_angle * 180.0f / PI));
	update_char(&pitch_angle_text);
}

void armor_damage_draw(float yaw_relative_angle)
{
	uint8_t i;
	switch (get_armor_hurt())
	{
		case ARMOR_ZERO:
		{
			for (i = 0; i <= 30; i++)
			{
				circle_draw(&armor_0, "front_armor", UI_Graph_ADD, 2, UI_Color_Purplish_red, 7, 960 + AHRS_cosf(yaw_relative_angle + PI / 2) * 110, 560 + AHRS_sinf(yaw_relative_angle + PI / 2) * 110, 20);
				update_ui(&armor_0);
			}
			break;
		}
		case ARMOR_ONE:
		{
			for (i = 0; i <= 30; i++)
			{
				circle_draw(&armor_1, "right_armor", UI_Graph_ADD, 2, UI_Color_Purplish_red, 7, 960 + AHRS_cosf(yaw_relative_angle + 2 * PI / 2) * 110, 560 + AHRS_sinf(yaw_relative_angle + 2 * PI / 2) * 110, 20);
				update_ui(&armor_1);
			}
			break;
		}
		case ARMOR_TWO:
		{
			for (i = 0; i <= 30; i++)
			{
				circle_draw(&armor_2, "back_armor", UI_Graph_ADD, 2, UI_Color_Purplish_red, 7, 960 + AHRS_cosf(yaw_relative_angle + 3 * PI / 2) * 110, 560 + AHRS_sinf(yaw_relative_angle + 3 * PI / 2) * 110, 20);
				update_ui(&armor_2);
			}
			break;
		}
		case ARMOR_THREE:
		{
			for (i = 0; i <= 30; i++)
			{
				circle_draw(&armor_3, "left_armor", UI_Graph_ADD, 2, UI_Color_Purplish_red, 7, 960 + AHRS_cosf(yaw_relative_angle + 4 * PI / 2) * 110, 560 + AHRS_sinf(yaw_relative_angle + 4 * PI / 2) * 110, 20);
				update_ui(&armor_3);
			}
			break;
		}
		case ARMOR_NONE:
		default:
		{
			osDelay(3);
			circle_draw(&armor_3, "left_armor", UI_Graph_Del, 2, UI_Color_Cyan, 7, 960 + AHRS_cosf(yaw_relative_angle + 4 * PI / 2) * 110, 560 + AHRS_sinf(yaw_relative_angle + 4 * PI / 2) * 110, 20);
			update_ui(&armor_3);
			circle_draw(&armor_2, "back_armor", UI_Graph_Del, 2, UI_Color_Cyan, 7, 960 + AHRS_cosf(yaw_relative_angle + 3 * PI / 2) * 110, 560 + AHRS_sinf(yaw_relative_angle + 3 * PI / 2) * 110, 20);
			update_ui(&armor_2);
			circle_draw(&armor_1, "right_armor", UI_Graph_Del, 2, UI_Color_Cyan, 7, 960 + AHRS_cosf(yaw_relative_angle + 2 * PI / 2) * 110, 560 + AHRS_sinf(yaw_relative_angle + 2 * PI / 2) * 110, 20);
			update_ui(&armor_1);
			circle_draw(&armor_0, "front_armor", UI_Graph_Del, 2, UI_Color_Cyan, 7, 960 + AHRS_cosf(yaw_relative_angle + PI / 2) * 110, 560 + AHRS_sinf(yaw_relative_angle + PI / 2) * 110, 20);
			update_ui(&armor_0);
			break;
		}
	}
}

// void super_cap_status_draw(void)
// {
	// if (cap_message_rx.cap_message.cap_milivoltage >= (SUPCAP_VOLTAGE_LOWER_USE_THRESHOLD * 1000.0f))
	// {
	// 	float_draw(&cap_voltage_data, "capVoltageData", UI_Graph_Change, 1, UI_Color_Cyan, 20, 4, 3, 1590, 468, (float)cap_message_rx.cap_message.cap_milivoltage / 1000.0f);
	// }
	// else if (cap_message_rx.cap_message.cap_milivoltage < 1000)
	// {
	// 	float_draw(&cap_voltage_data, "capVoltageData", UI_Graph_Change, 0, UI_Color_Purplish_red, 20, 4, 3, 1590, 468, (float)cap_message_rx.cap_message.cap_milivoltage / 1000.0f);
	// }
	// update_char(&cap_voltage_data);

	// if (cap_message_rx.cap_message.cap_power < 0)
	// {
	// 	float_draw(&cap_power_data, "capPower", UI_Graph_Change, 1, UI_Color_Cyan, 20, 5, 3, 1590, 428, cap_message_rx.cap_message.cap_power);
	// }
	// else if (cap_message_rx.cap_message.cap_power >= 0)
	// {
	// 	float_draw(&cap_power_data, "capPower", UI_Graph_Change, 0, UI_Color_Purplish_red, 20, 5, 3, 1590, 428, cap_message_rx.cap_message.cap_power);
	// }
	// update_char(&cap_power_data);
// }

void trigger_motor_state_draw(float trigger_rpm)
{
	if (trigger_rpm >= 0.5f)
	{
		float_draw(&trigger_speed_data, "triggerSpeedData", UI_Graph_Change, 7, UI_Color_Purplish_red, 20, 4, 3, 1590, 508, -trigger_rpm);
		update_char(&trigger_speed_data);
	}
	else
	{
		float_draw(&trigger_speed_data, "triggerSpeedData", UI_Graph_Change, 7, UI_Color_Cyan, 20, 4, 3, 1590, 508, -trigger_rpm);
		update_char(&trigger_speed_data);
	}
}

void chassis_mode_draw(void)
{
	switch (chassis_behaviour_mode)
	{
		case CHASSIS_BASIC_FPV_MODE:
		{
			char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227, "CNFY");
			update_char(&robot_status_str);
			char_draw(&rand_spin_str, "rand_spin_str", UI_Graph_Del, 8, UI_Color_Pink, 20, 4, 3, 930, 187, "RAND");
			update_char(&rand_spin_str);
			break;
		}
		case CHASSIS_SPINNING_MODE:
		{
			if(chassis_move.fRandomSpinOn)
			{
				char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227, "SPIN");
				update_char(&robot_status_str);
				char_draw(&rand_spin_str, "rand_spin_str", UI_Graph_ADD, 8, UI_Color_Pink, 20, 4, 3, 930, 187, "RAND");
				update_char(&rand_spin_str);
			}
			else
			{
				char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227, "SPIN");
				update_char(&robot_status_str);
				char_draw(&rand_spin_str, "rand_spin_str", UI_Graph_Del, 8, UI_Color_Pink, 20, 4, 3, 930, 187, "RAND");
				update_char(&rand_spin_str);
			}
			break;
			
		}
		default:
		{
			
			break;
		}
	}
}
