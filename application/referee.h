// @TODO: This file follows the serial protocol released in 2019, update it to latest protocol

#ifndef _REFEREE_H
#define _REFEREE_H

#include "main.h"

#include "protocol.h"
#include "user_lib.h"

// typedef enum
// {
// 	RED_HERO = 1,
// 	RED_ENGINEER = 2,
// 	RED_STANDARD_1 = 3,
// 	RED_STANDARD_2 = 4,
// 	RED_STANDARD_3 = 5,
// 	RED_AERIAL = 6,
// 	RED_SENTRY = 7,
// 	BLUE_HERO = 11,
// 	BLUE_ENGINEER = 12,
// 	BLUE_STANDARD_1 = 13,
// 	BLUE_STANDARD_2 = 14,
// 	BLUE_STANDARD_3 = 15,
// 	BLUE_AERIAL = 16,
// 	BLUE_SENTRY = 17,
// } robot_id_t;
// typedef enum
// {
// 	PROGRESS_UNSTART = 0,
// 	PROGRESS_PREPARE = 1,
// 	PROGRESS_SELFCHECK = 2,
// 	PROGRESS_5sCOUNTDOWN = 3,
// 	PROGRESS_BATTLE = 4,
// 	PROGRESS_CALCULATING = 5,
// } game_progress_t;

// typedef enum
// {
// 	ARMOR_ZERO,
// 	ARMOR_ONE,
// 	ARMOR_TWO,
// 	ARMOR_THREE,
// 	ARMOR_NONE,
// } armor_damage_info_t;

// typedef __packed struct // 0001
// {
// 	uint8_t game_type : 4;
// 	uint8_t game_progress : 4;
// 	uint16_t stage_remain_time;
// 	uint64_t SyncTimeStamp;
// } ext_game_state_t;

// typedef __packed struct // 0002
// {
// 	uint8_t winner;
// } ext_game_result_t;
// typedef __packed struct // 0003
// {
// 	uint16_t red_1_robot_HP;
// 	uint16_t red_2_robot_HP;
// 	uint16_t red_3_robot_HP;
// 	uint16_t red_4_robot_HP;
// 	uint16_t red_5_robot_HP;
// 	uint16_t red_7_robot_HP;
// 	uint16_t red_outpost_HP;
// 	uint16_t red_base_HP;
// 	uint16_t blue_1_robot_HP;
// 	uint16_t blue_2_robot_HP;
// 	uint16_t blue_3_robot_HP;
// 	uint16_t blue_4_robot_HP;
// 	uint16_t blue_5_robot_HP;
// 	uint16_t blue_7_robot_HP;
// 	uint16_t blue_outpost_HP;
// 	uint16_t blue_base_HP;
// } ext_game_robot_HP_t;

// typedef __packed struct // 0005
// {
//     uint8_t F1_zone_status : 1;
//     uint8_t F1_zone_buff_debuff_status : 3;
//     uint8_t F2_zone_status : 1;
//     uint8_t F2_zone_buff_debuff_status : 3;
//     uint8_t F3_zone_status : 1;
//     uint8_t F3_zone_buff_debuff_status : 3;
//     uint8_t F4_zone_status : 1;
//     uint8_t F4_zone_buff_debuff_status : 3;
//     uint8_t F5_zone_status : 1;
//     uint8_t F5_zone_buff_debuff_status : 3;
//     uint8_t F6_zone_status : 1;
//     uint8_t F6_zone_buff_debuff_status : 3;
//     uint16_t red1_bullet_left;
//     uint16_t red2_bullet_left;
//     uint16_t blue1_bullet_left;
//     uint16_t blue2_bullet_left;
//     uint8_t lurk_mode;
//     uint8_t res;
// } ext_ICRA_buff_debuff_zone_and_lurk_status_t;

// typedef __packed struct // 0x0101
// {
// 	uint32_t event_data;
// } ext_event_data_t;

// typedef __packed struct // 0x0102
// {
// 	uint8_t supply_projectile_id;
// 	uint8_t supply_robot_id;
// 	uint8_t supply_projectile_step;
// 	uint8_t supply_projectile_num;
// } ext_supply_projectile_action_t;

// typedef __packed struct // 0x0103
// {
//     uint8_t supply_projectile_id;
//     uint8_t supply_robot_id;
//     uint8_t supply_num;
// } ext_supply_projectile_booking_t;

// typedef __packed struct // 0x0104
// {
//     uint8_t dart_remaining_time;
// } ext_supply_projectile_booking_t //Current: ext_dart_remaining_time_t;

// typedef __packed struct // 0x0104
// {
// 	uint8_t level;
// 	uint8_t offending_robot_id;
// 	uint8_t count;
// } ext_referee_warning_t;

// typedef __packed struct //0x0105
// {
//  uint8_t dart_remaining_time;
//  uint8_t dart_aim_state;
// }dart_info_t;

// typedef __packed struct // 0x0201
// {
// 	uint8_t robot_id;
// 	uint8_t robot_level;
// 	uint16_t current_HP;
// 	uint16_t maximum_HP;
// 	uint16_t shooter_barrel_cooling_value;
// 	uint16_t shooter_barrel_heat_limit;
// 	uint16_t chassis_power_limit;
// 	uint8_t power_management_gimbal_output : 1;
// 	uint8_t power_management_chassis_output : 1;
// 	uint8_t power_management_shooter_output : 1;
// } ext_game_robot_state_t;

// typedef __packed struct // 0x0202
// {
// 	uint16_t chassis_voltage;
// 	uint16_t chassis_current;
// 	float chassis_power;
// 	uint16_t buffer_energy;
// 	uint16_t shooter_17mm_1_barrel_heat;
// 	uint16_t shooter_17mm_2_barrel_heat;
// 	uint16_t shooter_42mm_barrel_heat;
// } ext_power_heat_data_t;

// typedef __packed struct // 0x0203
// {
// 	float x;
// 	float y;
// 	float angle;
// } ext_game_robot_pos_t;

// typedef __packed struct // 0x0204
// {
// 	uint8_t recovery_buff;
// 	uint8_t cooling_buff;
// 	uint8_t defence_buff;
// 	uint16_t attack_buff;
// } ext_buff_musk_t;

// typedef __packed struct // 0x0205
// {
// 	uint8_t airforce_status;
// 	uint8_t time_remain;
// } ext_aerial_robot_energy_t;

// typedef __packed struct // 0x0206
// {
// 	uint8_t armor_id : 4;
// 	uint8_t HP_deduction_reason : 4;
// } ext_robot_hurt_t;

// typedef __packed struct // 0x0207
// {
// 	// uint8_t bullet_freq;
// 	// float bullet_speed;
// 	uint8_t bullet_type;
// 	uint8_t shooter_number;
// 	uint8_t launching_frequency;
// 	float initial_speed;
// } ext_shoot_data_t;

// typedef __packed struct // 0x0208
// {
// 	uint16_t projectile_allowance_17mm;
// 	uint16_t projectile_allowance_42mm;
// 	uint16_t remaining_gold_coin;
// } ext_projectile_allowance_t;

// typedef __packed struct // 0x0209
// {
// 	uint32_t rfid_status;
// } ext_rfid_status_t;

// typedef __packed struct // 0x020A
// {
//      uint8_t dart_launch_opening_status;
//      uint8_t reserved;
//      uint16_t target_change_time;
//      uint16_t latest_launch_cmd_time;
//     uint8_t dart_launch_opening_status;
//     uint8_t dart_attack_target;
//     uint16_t target_change_time;
//     uint16_t latest_launch_cmd_time;
// } ext_dart_client_cmd_t;

// typedef __packed struct // 0x020B
// {
// 	float hero_x;
// 	float hero_y;
// 	float engineer_x;
// 	float engineer_y;
// 	float standard_3_x;
// 	float standard_3_y;
// 	float standard_4_x;
// 	float standard_4_y;
// 	float standard_5_x;
// 	float standard_5_y;
// } ext_ground_robot_position_t;

// typedef __packed struct // 0x020C
// {
// 	uint8_t mark_hero_progress;
// 	uint8_t mark_engineer_progress;
// 	uint8_t mark_standard_3_progress;
// 	uint8_t mark_standard_4_progress;
// 	uint8_t mark_standard_5_progress;
// 	uint8_t mark_sentry_progress;
// } ext_radar_mark_data_t;

// typedef __packed struct // 0x020D
// {
// 	uint32_t sentry_info;
// } ext_sentry_info_t;

// typedef __packed struct // 0x020E
// {
// 	uint8_t radar_info;
// } ext_radar_info_t;

// typedef __packed struct
// {
// 	uint8_t bullet_remaining_num;
// } ext_bullet_remaining_t;

// typedef __packed struct // 0x0301
// {
// 	// uint16_t send_ID;
// 	// uint16_t receiver_ID;
// 	// uint16_t data_cmd_id;
// 	// uint16_t data_len;
// 	// uint8_t *data;
// 	uint16_t data_cmd_id;
// 	uint16_t sender_id;
// 	uint16_t receiver_id;
// 	// uint8_t user_data[x]; //x <= 112
// } ext_student_interactive_data_t;

// typedef __packed struct //0x0110
// {
//     graphic_data_struct_t grapic_data_struct;
//     uint8_t data[30];
// } ext_client_custom_character_t;

// typedef __packed struct // 0x0120
// {
// 	uint32_t sentry_cmd;
// } ext_sentry_cmd_t;

// typedef __packed struct // 0x0121
// {
// 	uint8_t radar_cmd;
// } ext_radar_cmd_t;

// typedef __pack struct // 0x0303
// {
//     float target_position_x;
//     float target_position_y;
//     uint8_t cmd_keyboard;
//     uint8_t target_robot_id;
//     uint8_t cmd_source;
// } ext_robot_command_t;
//     uint8_t cmd_keyboard;
//     uint8_t target_robot_id;
//     uint8_t cmd_source;
// } ext_map_command_t;

// typedef __pack struct // 0x0304
// {
//     int16_t mouse_x;
//     int16_t mouse_y;
//     int16_t mouse_z;
//     int8 left_button_down;
//     int8 right_button_down;
//     uint16_t keyboard_value;
//     uint16_t reserved;
// } ext_robot_keyboard_mouse_command_t; //Current: remote_control_t;

// typedef __pack struct // 0x0305
// {
//     uint16_t target_robot_id;
//     float target_position_x;
//     float target_position_y;
// } ext_map_robot_data_t;

// typedef __packed struct // 0x0306
// {
// 	uint16_t key_value;
// 	uint16_t x_position : 12;
// 	uint16_t mouse_left : 4;
// 	uint16_t y_position : 12;
// 	uint16_t mouse_right : 4;
// 	uint16_t reserved;
// } ext_custom_client_data_t;

// typedef __packed struct // 0x0307
// {
// 	uint8_t intention;
// 	uint16_t start_position_x;
// 	uint16_t start_position_y;
// 	int8_t delta_x[49];
// 	int8_t delta_y[49];
// 	uint16_t sender_id;
// } ext_map_data_t;

// typedef __packed struct // 0x0308
// {
// 	uint16_t sender_id;
// 	uint16_t receiver_id;
// 	uint16_t user_data[30];
// } ext_custom_info_t;

// typedef __packed struct
// {
// 	float data1;
// 	float data2;
// 	float data3;
// 	uint8_t data4;
// } custom_data_t;

// typedef __packed struct
// {
// 	uint8_t data[64];
// } ext_up_stream_data_t;

// typedef __packed struct
// {
// 	uint8_t data[32];
// } ext_download_stream_data_t;

typedef __packed union
{
    __packed struct
    {
        fp32 demoArmAngle[7];
        uint8_t fIsTeaching;
    } tData;
    uint8_t data[30];
} custom_robot_data_t;
STATIC_ASSERT(sizeof(custom_robot_data_t) <= 30);

extern void init_referee_struct_data(void);
extern void ref_solve_lost_fun(void);
extern void referee_data_solve(uint8_t *frame);

// extern void get_chassis_power_data(fp32 *power, fp32 *buffer, fp32 *power_limit);

// extern uint8_t get_robot_id(void);
// extern uint8_t get_team_color(void);

// extern void get_shoot_heat0_limit_and_heat(uint16_t *heat_limit, uint16_t *heat0);
// extern void get_shoot_heat1_limit_and_heat(uint16_t *heat_limit, uint16_t *heat1);
// uint8_t is_game_started(void);
// uint8_t get_time_remain(void);
// uint16_t get_current_HP(void);
// armor_damage_info_t get_armor_hurt(void);

// extern ext_robot_hurt_t robot_hurt_t;
#endif
