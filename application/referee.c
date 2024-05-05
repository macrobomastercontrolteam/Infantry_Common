// @TODO: This file follows the serial protocol released in 2019, update it to latest protocol

#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "usart.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;                                    //0x0001
ext_game_result_t game_result;                                  //0x0002
ext_game_robot_HP_t game_robot_HP_t;                            //0x0003
// ext_ICRA_buff_debuff_zone_and_lurk_status_t ICRA_buff_debuff_zone_and_lurk_status_t; //0x0005

ext_event_data_t field_event; //0x0101
ext_supply_projectile_action_t supply_projectile_action_t;      //0x0102
// ext_supply_projectile_booking_t supply_projectile_booking_t; //0x0103
ext_referee_warning_t referee_warning_t;                        //0x0104

ext_game_robot_state_t robot_state;                             //0x0201
ext_power_heat_data_t power_heat_data_t;                        //0x0202
ext_game_robot_pos_t game_robot_pos_t;                          //0x0203
ext_buff_musk_t buff_musk_t;                                    //0x0204
ext_aerial_robot_energy_t robot_energy_t;                       //0x0205
ext_robot_hurt_t robot_hurt_t;                                  //0x0206
ext_shoot_data_t shoot_data_t;                                  //0x0207
ext_rfid_status_t rfid_status_t;                                //0x0208
ext_projectile_allowance_t projectile_allowance_t;              //0x0209
// ext_dart_client_cmd_t dart_client_cmd_t;                     //0x020A
ext_ground_robot_position_t ground_robot_position_t;            //0x020B
ext_radar_mark_data_t radar_mark_data_t;                        //0x020C
ext_sentry_info_t sentry_info_t;                                //0x020D
ext_radar_info_t radar_info_t;                                  //0x020E
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

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.buffer_energy;
}

uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat_limit_and_heat(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_barrel_heat_limit;
    *heat0 = power_heat_data_t.shooter_17mm_1_barrel_heat;;
}

// GRAPHICS stuff here cause i'm lazy

// typedef __packed struct
// {
//     uint8_t graphic_name[3];
//     uint32_t operate_tpye : 3;
//     uint32_t graphic_tpye : 3;
//     uint32_t layer : 4;
//     uint32_t color : 4;
//     uint32_t start_angle : 9;
//     uint32_t end_angle : 9;
//     uint32_t width : 10;
//     uint32_t start_x : 11;
//     uint32_t start_y : 11;
//     uint32_t radius : 10;
//     uint32_t end_x : 11;
//     uint32_t end_y : 11;
// } graphic_data_struct_t;

// // Client Drawing Graphics
// typedef __packed struct
// {
//     // Number of graphics to be drawn, i.e., the length of the graphic data array.
//     // However, it is important to carefully check the content ID corresponding to the increase 
//     // in the number of graphics provided by the referee system.
//     graphic_data_struct_t grapic_data_struct[7];  

// } ext_client_custom_graphic_t;

// // Interactive Data Information
// typedef __packed struct
// {
//     uint16_t data_cmd_id;                        // Data segment content ID
//     uint16_t sender_ID;                          // Sender ID
//     uint16_t receiver_ID;                        // Receiver ID
//     ext_client_custom_graphic_t graphic_custom;  // Custom graphic data

// } ext_student_interactive_header_data_t;

// /**
//  * @brief     Pack data to the bottom device
//  * @param[in] sof: frame header
//  * @param[in] cmd_id: command ID of the data
//  * @param[in] *p_data: pointer to the data to be sent
//  * @param[in] len: the data length
//  */
// #define MAX_SIZE 128      // Maximum length of uploaded data
// #define frameheader_len 5 // Frame header length
// #define cmd_len 2         // Command code length
// #define crc_len 2         // CRC16 checksum
// uint8_t seq = 0;

// void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)
// {
//     uint8_t tx_buff[MAX_SIZE];

//     uint16_t frame_length = frameheader_len + cmd_len + len + crc_len; // Data frame length

//     memset(tx_buff, 0, frame_length); // Clear the array for storing data

//     /***** Frame Header Packing *****/
//     tx_buff[0] = sof;                                  // Start byte of the data frame
//     memcpy(&tx_buff[1], (uint8_t *)&len, sizeof(len)); // Length of the data in the data frame
//     tx_buff[3] = seq;                                  // Packet sequence number
//     append_CRC8_check_sum(tx_buff, frameheader_len);   // Frame header CRC8 checksum

//     /***** Command Code Packing *****/
//     memcpy(&tx_buff[frameheader_len], (uint8_t *)&cmd_id, cmd_len);

//     /***** Data Packing *****/
//     memcpy(&tx_buff[frameheader_len + cmd_len], p_data, len);
//     append_CRC16_check_sum(tx_buff, frame_length); // Data frame CRC16 checksum

//     if (seq == 0xff)
//         seq = 0;
//     else
//         seq++;

//     /***** Data Upload *****/
//     // TODO: Figure out how to send data to the ref system
//     // Might need this?
//     // USART_ClearFlag(UART4, USART_FLAG_TC);
//     // for (i = 0; i < frame_length; i++)
//     // {
//     //     // USART_SendData(UART4, tx_buff[i]); // Don't think i need this
//     //     // TODO: Not sure if need to change the huart number from 6 to 4?
//     //     HAL_UART_Transmit(&huart6, tx_buff[i], 1, 10);
//     //     while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET); // Wait for the previous character to be sent
//     // }    

//     // I'm just gonna send it all at once, fuck it wii ball
//     HAL_UART_Transmit(&huart6, tx_buff, frame_length, 10000);

// }

// // ????????1920x1080
// #define SCREEN_WIDTH 1080
// #define SCREEN_LENGTH 1920
// ext_student_interactive_header_data_t custom_grapic_draw; // ???????????
// ext_client_custom_graphic_t custom_graphic;               // ????????

// // Screen resolution is 1920x1080
// #define SCREEN_WIDTH 1080
// #define SCREEN_LENGTH 1920

// // Custom graphic draw header data
// ext_student_interactive_header_data_t custom_grapic_draw;

// // Custom graphic structure
// ext_client_custom_graphic_t custom_graphic;

// // Initialize graphic data variables
// void init_graphic_data() {
//     static long calledTimes = 0;
//     calledTimes++;

//     // Custom graphic draw
//     custom_grapic_draw.data_cmd_id = 0x0104; // Draw seven graphics (Content ID, refer to the referee system manual for queries)

//     // TODO: There are multiple blue standard robot ids
//     // These ones might be wrong
//     custom_grapic_draw.sender_ID = 105;       // Sender ID, corresponding to the robot ID, in this case, the Blue Standard
//     custom_grapic_draw.receiver_ID = 0x0169;  // Receiver ID, operator client ID, in this case, the Blue Standard operator client

//     // Custom graphic data
//     {
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 97;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 97;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0; // Graphic name
//         // The above three bytes represent the graphic name, used for graphic indexing, can be defined as needed
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye = 1; // Graphic operation, 0: empty operation; 1: add; 2: modify; 3: delete;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye = 0; // Graphic type, 0 for a straight line, refer to the user manual for others
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer = 1;        // Graphic layer
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].color = 1;        // Color
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle = 0;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle = 0;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].width = 1;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x = SCREEN_LENGTH / 2;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y = SCREEN_WIDTH / 2;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x = SCREEN_LENGTH / 2;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y = SCREEN_WIDTH / 2 - 300;
//         custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius = 0;
//     }
//     // Here, only graphic 1 is drawn; refer to the above for assigning values to the graphic data array for other graphics
//     referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&custom_grapic_draw, sizeof(custom_grapic_draw));
// }

