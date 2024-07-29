// Last updated to Ref Serial Protocol V1.6.3 (2024/05/27)

#include "referee.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "detect_task.h"
#include "shoot.h"

#define REF_TEST_MODE 1

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;         // 0x0001
ext_game_result_t game_result;       // 0x0002
ext_game_robot_HP_t game_robot_HP_t; // 0x0003
// ext_ICRA_buff_debuff_zone_and_lurk_status_t ICRA_buff_debuff_zone_and_lurk_status_t; //0x0005

ext_event_data_t field_event;                              // 0x0101
ext_supply_projectile_action_t supply_projectile_action_t; // 0x0102
// ext_supply_projectile_booking_t supply_projectile_booking_t; //0x0103
ext_referee_warning_t referee_warning_t; // 0x0104

ext_game_robot_state_t robot_state;                // 0x0201
ext_power_heat_data_t power_heat_data_t;           // 0x0202
ext_game_robot_pos_t game_robot_pos_t;             // 0x0203
ext_buff_musk_t buff_musk_t;                       // 0x0204
ext_aerial_robot_energy_t robot_energy_t;          // 0x0205
ext_robot_hurt_t robot_hurt_t;                     // 0x0206
ext_shoot_data_t shoot_data_t;                     // 0x0207
ext_rfid_status_t rfid_status_t;                   // 0x0208
ext_projectile_allowance_t projectile_allowance_t; // 0x0209
// ext_dart_client_cmd_t dart_client_cmd_t;                     //0x020A
ext_ground_robot_position_t ground_robot_position_t; // 0x020B
ext_radar_mark_data_t radar_mark_data_t;             // 0x020C
ext_sentry_info_t sentry_info_t;                     // 0x020D
ext_radar_info_t radar_info_t;                       // 0x020E
// ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_data_t student_interactive_data_t;
// ext_client_custom_character_t client_custom_character_t;
ext_sentry_cmd_t sentry_cmd_t;
ext_radar_cmd_t radar_cmd_t;
// ext_robot_interactive_data_t robot_interactive_data_t;
// ext_map_command_t map_command_t;
// ext_map_robot_data_t map_robot_data_t;
// ext_robot_keyboard_mouse_command_t robot_keyboard_mouse_command_t;
// ext_client_map_command_t client_map_command_t;
ext_custom_client_data_t custom_client_data_t;
ext_map_data_t map_data_t;
ext_custom_info_t custom_info_t;

#if REF_TEST_MODE
uint16_t uiRefRxCmdId = 0;
uint8_t test_mains_power_chassis_output = 0;
uint8_t test_mains_power_shooter_output = 0;
uint8_t test_mains_power_gimbal_output = 0;
#endif

void init_referee_struct_data(void)
{
	memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
	memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

	memset(&game_state, 0, sizeof(ext_game_state_t));
	memset(&game_result, 0, sizeof(ext_game_result_t));
	memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

	memset(&field_event, 0, sizeof(ext_event_data_t));
	memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
	// memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
	memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));

	memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
	memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
	memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
	memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
	// memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
	memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
	memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
	memset(&projectile_allowance_t, 0, sizeof(ext_projectile_allowance_t));
	memset(&radar_mark_data_t, 0, sizeof(ext_radar_mark_data_t));
	memset(&ground_robot_position_t, 0, sizeof(ext_ground_robot_position_t));
	memset(&sentry_info_t, 0, sizeof(ext_sentry_info_t));
	memset(&radar_info_t, 0, sizeof(ext_radar_info_t));
	memset(&sentry_cmd_t, 0, sizeof(ext_sentry_cmd_t));
	memset(&radar_cmd_t, 0, sizeof(ext_radar_cmd_t));
	// memset(&map_command_t, 0, sizeof(ext_map_command_t));
	// memset(&map_robot_data_t, 0, sizeof(ext_map_robot_data_t));
	memset(&custom_info_t, 0, sizeof(ext_custom_info_t));
	memset(&custom_client_data_t, 0, sizeof(ext_custom_client_data_t));

	memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
}

void referee_data_solve(uint8_t *frame)
{
	uint16_t cmd_id = 0;

	uint8_t index = 0;

	memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

	index += sizeof(frame_header_struct_t);

	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);

#if REF_TEST_MODE
    uiRefRxCmdId = cmd_id;
#endif

	switch (cmd_id)
	{
		case GAME_STATE_CMD_ID:
		{
			memcpy(&game_state, frame + index, sizeof(game_state));
			break;
		}
		case GAME_RESULT_CMD_ID:
		{
			memcpy(&game_result, frame + index, sizeof(game_result));
			break;
		}
		case GAME_ROBOT_HP_CMD_ID:
		{
			memcpy(&game_robot_HP_t, frame + index, sizeof(game_robot_HP_t));
			break;
		}
		// case DART_LAUNCH_STATUS_ID:
		// {
		//     memcpy();
		//     break;
		// }
		// case AI_BUFF_DEBUFF_CHALLENGE_ID:
		// {
		//     memcpy(&ICRA_buff_debuff_zone_and_lurk_status_t, frame + index, sizeof(ICRA_buff_debuff_zone_and_lurk_status_t));
		//     break;
		// }
		// case DART_LNCH_OPENING_CNTDWN_ID:
		// {
		//     memcpy(&dart_remaining_time_t, frame + index, sizeof(ICRA_buff_debuff_zone_and_lurk_status_t));
		//     break;
		// }
		case FIELD_EVENTS_CMD_ID:
		{
			memcpy(&field_event, frame + index, sizeof(field_event));
			break;
		}
		case SUPPLY_PROJECTILE_ACTION_CMD_ID:
		{
			memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
			break;
		}
		// case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
		// {
		//     memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
		//     break;
		// }
		case REFEREE_WARNING_CMD_ID:
		{
			memcpy(&referee_warning_t, frame + index, sizeof(referee_warning_t));
			break;
		}

		case ROBOT_STATE_CMD_ID:
		{
			memcpy(&robot_state, frame + index, sizeof(robot_state));
#if REF_TEST_MODE
            test_mains_power_chassis_output = robot_state.power_management_chassis_output;
            test_mains_power_shooter_output = robot_state.power_management_shooter_output;
            test_mains_power_gimbal_output = robot_state.power_management_gimbal_output;
#endif
			break;
		}
		case POWER_HEAT_DATA_CMD_ID:
		{
			memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
			break;
		}
		case ROBOT_POS_CMD_ID:
		{
			memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
			break;
		}
		case BUFF_MUSK_CMD_ID:
		{
			memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
			break;
		}
		// case AERIAL_ROBOT_ENERGY_CMD_ID:
		// {
		//     memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
		//     break;
		// }
		case ROBOT_HURT_CMD_ID:
		{
			memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
			break;
		}
		case SHOOT_DATA_CMD_ID:
		{
			memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
			switch (shoot_data_t.shooter_number)
			{
				case 1:
				case 2:
				{
					shoot_control.launching_frequency[shoot_data_t.shooter_number - 1] = shoot_data_t.launching_frequency;
					shoot_control.bullet_init_speed[shoot_data_t.shooter_number - 1] = shoot_data_t.initial_speed;
					break;
				}
				default:
				{
					break;
				}
			}
			break;
		}
		case ROBOT_RFID_STATUS_ID:
		{
			memcpy(&rfid_status_t, frame + index, sizeof(rfid_status_t));
			break;
		}
		case PROJECTILE_ALLOWANCE_CMD_ID:
		{
			memcpy(&projectile_allowance_t, frame + index, sizeof(projectile_allowance_t));
			break;
		}
		case GROUND_ROBOT_POSITION_CMD_ID:
		{
			memcpy(&ground_robot_position_t, frame + index, sizeof(ground_robot_position_t));
			break;
		}
		case RADAR_MARK_DATA_CMD_ID:
		{
			memcpy(&radar_mark_data_t, frame + index, sizeof(radar_mark_data_t));
			break;
		}
		case SENTRY_INFO_CMD_ID:
		{
			memcpy(&sentry_info_t, frame + index, sizeof(sentry_info_t));
			break;
		}
		case RADAR_INFO_CMD_ID:
		{
			memcpy(&radar_info_t, frame + index, sizeof(radar_info_t));
			break;
		}
		case SENTRY_DECISION_CMD_ID:
		{
			memcpy(&sentry_cmd_t, frame + index, sizeof(sentry_cmd_t));
			break;
		}
		case RADAR_DECISION_CMD_ID:
		{
			memcpy(&sentry_cmd_t, frame + index, sizeof(sentry_cmd_t));
			break;
		}
		case CUSTOM_INFO_CMD_ID:
		{
			memcpy(&custom_info_t, frame + index, sizeof(custom_info_t));
			break;
		}
		case CUSTOM_CLIENT_DATA_CMD_ID:
		{
			memcpy(&custom_client_data_t, frame + index, sizeof(custom_client_data_t));
			break;
		}
		// case DART_ROBOT_INSTRUCTIONS_ID:
		// {
		//     memcpy(&dart_client_cmd_t, frame + index, sizeof(dart_client_cmd_t));
		//     break;
		// }
		case STUDENT_INTERACTIVE_DATA_CMD_ID:
		{
			memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
			break;
		}

		//@TODO: unused for now
		// case CUSTOM_CNTRLR_DATA_INTRFCE_ID:
		// {
		//     memcpy(&robot_interactive_data_t, frame + index, sizeof(robot_interactive_data_t));
		//     break;
		// }
		// case SMALL_MAP_INTERACTION_DATA_ID:
		// {
		//     memcpy(&robot_command_t, frame + index, sizeof(robot_command_t));
		//     break;
		// }
		// case KEYBOARD_MOUSE_INFO_ID:
		// {
		//     memcpy(&robot_keyboard_mouse_command_t, frame + index, sizeof(robot_keyboard_mouse_command_t));
		//     break;
		// }
		// case SMALL_MAP_DATA_RECEIPT_ID:
		// {
		//     memcpy(&client_map_command_t, frame + index, sizeof(client_map_command_t));
		//     break;
		// }
		default:
		{
			break;
		}
	}
}

void get_chassis_power_data(fp32 *power, fp32 *buffer, fp32 *power_limit)
{
	*power = power_heat_data_t.chassis_power;
	*buffer = power_heat_data_t.buffer_energy;
	if (robot_state.chassis_power_limit > 0)
	{
		*power_limit = robot_state.chassis_power_limit;
	}
	else
	{
		*power_limit = 45;
	}
}

uint8_t get_robot_id(void)
{
	return robot_state.robot_id;
}

uint8_t get_team_color(void)
{
	// blue: 0, red: 1
	uint8_t team_color = 0;
	switch (robot_state.robot_id)
	{
		case RED_HERO:
		case RED_ENGINEER:
		case RED_STANDARD_1:
		case RED_STANDARD_2:
		case RED_STANDARD_3:
		case RED_AERIAL:
		case RED_SENTRY:
		{
			team_color = 1;
			break;
		}
		case BLUE_HERO:
		case BLUE_ENGINEER:
		case BLUE_STANDARD_1:
		case BLUE_STANDARD_2:
		case BLUE_STANDARD_3:
		case BLUE_AERIAL:
		case BLUE_SENTRY:
		default:
		{
			team_color = 0;
			break;
		}
	}
	return team_color;
}

void get_shoot_heat0_limit_and_heat(uint16_t *heat_limit, uint16_t *heat0)
{
	*heat_limit = robot_state.shooter_barrel_heat_limit;
	*heat0 = power_heat_data_t.shooter_17mm_1_barrel_heat;
}

void get_shoot_heat1_limit_and_heat(uint16_t *heat_limit, uint16_t *heat1)
{
	*heat_limit = robot_state.shooter_barrel_heat_limit;
	*heat1 = power_heat_data_t.shooter_17mm_2_barrel_heat;
}

uint8_t is_game_started(void)
{
	return ((game_state.game_progress == 4) && (toe_is_error(REFEREE_TOE) == 0));
}

uint8_t get_time_remain(void)
{
	return game_state.stage_remain_time;
}

uint16_t get_current_HP(void)
{
	return robot_state.current_HP;
}

armor_damage_info_t get_armor_hurt(void)
{
	if ((robot_hurt_t.HP_deduction_reason == 0) || (robot_hurt_t.HP_deduction_reason == 5))
	{
		return (armor_damage_info_t)robot_hurt_t.armor_id;
	}
	else
	{
		return ARMOR_NONE;
	}
}
