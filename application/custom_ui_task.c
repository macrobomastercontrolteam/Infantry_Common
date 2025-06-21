/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      Chassis control task,
  *             
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
#include <stdio.h>

#define CUSTOM_UI_TIME_MS 10.0f

ui_info_t ui_info;

graphic_data_struct_t barrel_dir;
graphic_data_struct_t chassis_dir;
string_data chassis_relative_angle;
string_data pitch_direction_icon;
string_data pitch_angle_text;
string_data cap_voltage_data;
string_data cap_power_data;
string_data cap_voltage_str;
string_data cap_power;
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

graphic_data_struct_t CV_Box;

string_data Spin_state;
graphic_data_struct_t Spin_indicator;
string_data Trigger_state;
graphic_data_struct_t Trigger_indicator;
string_data Firc_state;
graphic_data_struct_t Firc_indicator;
string_data AutoAim_state;
graphic_data_struct_t AutoAim_indicator;
string_data Cap_state;
string_data Cap_percentage;
string_data Power_saving_state;
graphic_data_struct_t Power_saving_indicator;
string_data Limit_state;
graphic_data_struct_t Limit_indicator;
string_data Loaded_state;
graphic_data_struct_t Loaded_indicator;
string_data Opened_state;
graphic_data_struct_t Opened_indicator;


uint8_t toggle = 0; 
char number_str[5];

void static_elements_init(void);
void super_cap_status_draw(void);
void chassis_direction_draw(float yaw_relative_angle);
void gimbal_pitch_direction_draw(float pitch_relative_angle);
void armor_damage_draw(float yaw_relative_angle);
void chassis_mode_draw(void);
void trigger_motor_state_draw(float trigger_rpm);

void custom_ui_task(void const *argument)
{
	uint32_t ulSystemTime = osKernelSysTick();
	for (uint8_t i = 0; i <= 50; i++)
	{
		clear_ui_all();
	}

	for (uint8_t i = 0; i <= 50; i++)
	{
		static_elements_init();
	}

	while (1)
	{
		// trigger_motor_state_draw(shoot_control.speed);
		// chassis_direction_draw(gimbal_control.gimbal_yaw_motor.relative_angle);
		// gimbal_pitch_direction_draw(gimbal_control.gimbal_pitch_motor.absolute_angle);
		// armor_damage_draw(gimbal_control.gimbal_yaw_motor.relative_angle);
		// super_cap_status_draw();
		chassis_mode_draw();
		osDelayUntil(&ulSystemTime, CUSTOM_UI_TIME_MS);
	}
}

void static_elements_init(void)
{
	rectangle_draw(&CV_Box, "CV_Box", UI_Graph_ADD, 2, UI_Color_Orange, 3, 590, 770, 1330, 320);
	update_ui(&CV_Box);

	char_draw(&Spin_state, "Spin_label", UI_Graph_ADD, 0, UI_Color_Pink, 15, 4, 3, 1680, 605, "SPIN");
	update_char(&Spin_state);
	circle_draw(&Spin_indicator, "Spin_indicator", UI_Graph_ADD, 2, UI_Color_Pink, 10, 1755, 600, 5);
	update_ui(&Spin_indicator);

	char_draw(&Trigger_state, "TriggerSpeed_label", UI_Graph_ADD, 0, UI_Color_Pink, 15, 6, 3, 1680, 645, "TRIG");
	update_char(&Trigger_state);
	circle_draw(&Trigger_indicator, "TriggerSpeedData", UI_Graph_ADD, 7, UI_Color_Cyan, 10, 1755, 640, 5);
	update_ui(&Trigger_indicator);

	char_draw(&Firc_state, "Firc_label", UI_Graph_ADD, 0, UI_Color_Pink, 15, 4, 3, 1680, 685, "FIRC");
	update_char(&Firc_state);
	circle_draw(&Firc_indicator, "Firc_indicator", UI_Graph_ADD, 2, UI_Color_Pink, 10, 1755, 680, 5);
	update_ui(&Firc_indicator);

	// Auto Aim State
	char_draw(&AutoAim_state, "AutoAim_label", UI_Graph_ADD, 0, UI_Color_Pink, 15, 4, 3, 1680, 725, "AAIM");
	update_char(&AutoAim_state);
	circle_draw(&AutoAim_indicator, "AutoAim_indicator", UI_Graph_ADD, 2, UI_Color_Pink, 10, 1755, 720, 5);
	update_ui(&AutoAim_indicator);

	// Supercap Percentage
	char_draw(&Cap_state, "CAP_state", UI_Graph_ADD, 5, UI_Color_Pink, 15, 5, 3, 1680, 765, "CAP%");
	update_char(&Cap_state);

	sprintf(number_str, "%d", -1);  
	char_draw(&Cap_percentage, "Cap_percentage", UI_Graph_ADD, 4, UI_Color_Main, 15, 5, 3, 1760, 762, number_str);
	update_char(&Cap_percentage);


	// Limit Ignored
	char_draw(&Limit_state, "Limit_label", UI_Graph_ADD, 0, UI_Color_Pink, 15, 4, 3, 1680, 805, "HTLM");
	update_char(&Limit_state);
	circle_draw(&Limit_indicator, "Limit_indicator", UI_Graph_ADD, 2, UI_Color_Pink, 10, 1755, 800, 5);
	update_ui(&Limit_indicator);

	char_draw(&Power_saving_state, "Power_saving", UI_Graph_ADD, 0, UI_Color_Pink, 15, 4, 3, 1680, 845, "PRSV");
	update_char(&Power_saving_state);
	circle_draw(&Power_saving_indicator, "Power_saving_indicator", UI_Graph_ADD, 2, UI_Color_Pink, 10, 1755, 840, 5);
	update_ui(&Power_saving_indicator);

	// Launcher Loaded
#if (LAUNCHER_TYPE == LAUNCHER_42MM)
	char_draw(&Loaded_state, "Loaded_label", UI_Graph_ADD, 0, UI_Color_Pink, 15, 4, 3, 1680, 565, "LOAD");
	update_char(&Loaded_state);
	circle_draw(&Loaded_indicator, "Loaded_indicator", UI_Graph_ADD, 2, UI_Color_Pink, 10, 1755, 560, 5);
	update_ui(&Loaded_indicator);

	// Launcher Opened
	char_draw(&Opened_state, "Opened_label", UI_Graph_ADD, 0, UI_Color_Pink, 15, 4, 3, 1680, 525, "OPEN");
	update_char(&Opened_state);
	circle_draw(&Opened_indicator, "Opened_indicator", UI_Graph_ADD, 2, UI_Color_Pink, 10, 1755, 520, 5);
	update_ui(&Opened_indicator);
#endif
	// line_draw(&crosshair_vert, "091", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 960, 330, 960, 620);
	// update_ui(&crosshair_vert);
	// line_draw(&crosshair_hori_2, "092", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 880, 580, 1040, 580);
	// update_ui(&crosshair_hori_2);
	// line_draw(&crosshair_hori_3, "093", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 800, 540, 1120, 540);
	// update_ui(&crosshair_hori_3);
	// line_draw(&crosshair_hori_4, "094", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 880, 500, 1040, 500);
	// update_ui(&crosshair_hori_4);
	// line_draw(&crosshair_hori_5, "095", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 900, 420, 1020, 420);
	// update_ui(&crosshair_hori_5);
	// line_draw(&crosshair_hori_6, "096", UI_Graph_ADD, 9, UI_Color_Cyan, 2, 920, 370, 1000, 370);
	// update_ui(&crosshair_hori_6);
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

void super_cap_status_draw(void)
{
	if (cap_message_rx.cap_message.cap_milivoltage >= (SUPCAP_VOLTAGE_LOWER_USE_THRESHOLD * 1000.0f))
	{
		float_draw(&cap_voltage_data, "capVoltageData", UI_Graph_Change, 1, UI_Color_Cyan, 20, 4, 3, 1590, 468, (float)cap_message_rx.cap_message.cap_milivoltage / 1000.0f);
	}
	else if (cap_message_rx.cap_message.cap_milivoltage < 1000)
	{
		float_draw(&cap_voltage_data, "capVoltageData", UI_Graph_Change, 0, UI_Color_Purplish_red, 20, 4, 3, 1590, 468, (float)cap_message_rx.cap_message.cap_milivoltage / 1000.0f);
	}
	update_char(&cap_voltage_data);

	if (cap_message_rx.cap_message.cap_power < 0)
	{
		float_draw(&cap_power_data, "capPower", UI_Graph_Change, 1, UI_Color_Cyan, 20, 5, 3, 1590, 428, cap_message_rx.cap_message.cap_power);
	}
	else if (cap_message_rx.cap_message.cap_power >= 0)
	{
		float_draw(&cap_power_data, "capPower", UI_Graph_Change, 0, UI_Color_Purplish_red, 20, 5, 3, 1590, 428, cap_message_rx.cap_message.cap_power);
	}
	update_char(&cap_power_data);
}

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
	sprintf(number_str, "%d", ui_info.supercap_percentage);  
	char_draw(&Cap_percentage, "Cap_percentage", UI_Graph_Change, 4, UI_Color_Main, 15, 5, 3, 1760, 762, number_str);
	update_char(&Cap_percentage);

	//If else statements for indicator color

	if ((ui_info.launcher_flag_byte & UI_AUTO_AIM_STATE_BIT) == 0) 
	{
		circle_draw(&AutoAim_indicator, "AutoAim_indicator", UI_Graph_Change, 2, UI_Color_Purplish_red, 10, 1755, 720, 5);
		update_ui(&AutoAim_indicator);
	} 
	else
	{
		circle_draw(&AutoAim_indicator, "AutoAim_indicator", UI_Graph_Change, 2, UI_Color_Green, 10, 1755, 720, 5);
		update_ui(&AutoAim_indicator);
	}

	if ((ui_info.chassis_flag_byte & UI_SPINNING_STATE_BIT)  == 0) 
	{
		circle_draw(&Spin_indicator, "Spin_indicator", UI_Graph_Change, 2, UI_Color_Purplish_red, 10, 1755, 600, 5);
		update_ui(&Spin_indicator);
	} 
	else 
	{
		circle_draw(&Spin_indicator, "Spin_indicator", UI_Graph_Change, 2, UI_Color_Green, 10, 1755, 600, 5);
		update_ui(&Spin_indicator);
	}

	if ((ui_info.launcher_flag_byte & UI_FRIC_STATE_BIT) == 0)
	{
		circle_draw(&Firc_indicator, "Firc_indicator", UI_Graph_Change, 2, UI_Color_Purplish_red, 10, 1755, 680, 5);
		update_ui(&Firc_indicator);
	}
	else
	{
		circle_draw(&Firc_indicator, "Firc_indicator", UI_Graph_Change, 2, UI_Color_Green, 10, 1755, 680, 5);
		update_ui(&Firc_indicator);
	}

	if ((ui_info.launcher_flag_byte & UI_TRIGGER_STATE_BIT) == 0) 
	{
		circle_draw(&Trigger_indicator, "TriggerSpeedData", UI_Graph_Change, 7, UI_Color_Purplish_red, 10, 1755, 640, 5);
		update_ui(&Trigger_indicator);
	}
	else
	{
		circle_draw(&Trigger_indicator, "TriggerSpeedData", UI_Graph_Change, 7, UI_Color_Green, 10, 1755, 640, 5);
		update_ui(&Trigger_indicator);
	}

	if ((ui_info.launcher_flag_byte & UI_IGNORE_HEAT_LIMIT_BIT) == 0) 
	{
		circle_draw(&Limit_indicator, "Limit_indicator", UI_Graph_Change, 2, UI_Color_Purplish_red, 10, 1755, 800, 5);
		update_ui(&Limit_indicator);	
	} 
	else
	{
		circle_draw(&Limit_indicator, "Limit_indicator", UI_Graph_Change, 2, UI_Color_Green, 10, 1755, 800, 5);
		update_ui(&Limit_indicator);
	}

	if ((ui_info.chassis_flag_byte & UI_POWER_SAVING_BIT) == 0) 
	{
		circle_draw(&Power_saving_indicator, "Power_saving_indicator", UI_Graph_Change, 2, UI_Color_Purplish_red, 10, 1755, 840, 5);
		update_ui(&Power_saving_indicator);
	}
	else
	{
		circle_draw(&Power_saving_indicator, "Power_saving_indicator", UI_Graph_Change, 2, UI_Color_Purplish_red, 10, 1755, 840, 5);
		update_ui(&Power_saving_indicator);
	}

#if (LAUNCHER_TYPE == LAUNCHER_42MM)
	if ((ui_info.launcher_flag_byte & UI_LAUNCHER_LOADED_BIT) == 0)
	{
		circle_draw(&Loaded_indicator, "Loaded_indicator", UI_Graph_Change, 2, UI_Color_Purplish_red, 10, 1755, 560, 5);
		update_ui(&Loaded_indicator);
	}
	else
	{
		circle_draw(&Loaded_indicator, "Loaded_indicator", UI_Graph_Change, 2, UI_Color_Green,  10, 1755, 560, 5);
		update_ui(&Loaded_indicator);
	}

	if ((ui_info.launcher_flag_byte & UI_LAUNCHER_OPENED_BIT) == 0)
	{
		circle_draw(&Opened_indicator, "Opened_indicator", UI_Graph_Change, 2, UI_Color_Purplish_red, 10, 1755, 520, 5);
		update_ui(&Opened_indicator);
	}
	else
	{
		circle_draw(&Opened_indicator, "Opened_indicator", UI_Graph_Change, 2, UI_Color_Green, 10, 1755, 520, 5);
		update_ui(&Opened_indicator);
	}
#endif

	

	// switch (chassis_behaviour_mode)
	// {
	// 	case CHASSIS_BASIC_FPV_MODE:
	// 	{
	// 		char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227, "CNFY");
	// 		update_char(&robot_status_str);
	// 		char_draw(&rand_spin_str, "rand_spin_str", UI_Graph_Del, 8, UI_Color_Pink, 20, 4, 3, 930, 187, "RAND");
	// 		update_char(&rand_spin_str);
	// 		break;
	// 	}
	// 	case CHASSIS_SPINNING_MODE:
	// 	{
	// 		if(chassis_move.fRandomSpinOn)
	// 		{
	// 			char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227, "SPIN");
	// 			update_char(&robot_status_str);
	// 			char_draw(&rand_spin_str, "rand_spin_str", UI_Graph_ADD, 8, UI_Color_Pink, 20, 4, 3, 930, 187, "RAND");
	// 			update_char(&rand_spin_str);
	// 		}
	// 		else
	// 		{
	// 			char_draw(&robot_status_str, "robot_status_str", UI_Graph_Change, 8, UI_Color_Pink, 20, 4, 3, 930, 227, "SPIN");
	// 			update_char(&robot_status_str);
	// 			char_draw(&rand_spin_str, "rand_spin_str", UI_Graph_Del, 8, UI_Color_Pink, 20, 4, 3, 930, 187, "RAND");
	// 			update_char(&rand_spin_str);
	// 		}
	// 		break;
			
	// 	}
	// 	default:
	// 	{
			
	// 		break;
	// 	}
	// }
}


