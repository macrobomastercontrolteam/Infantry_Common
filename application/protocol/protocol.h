// @TODO: This file follows the serial protocol released in 2019, update it to latest protocol

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "global_inc.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

// Up to date to ref protocol 2021.12.31 version
typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    DART_LAUNCH_STATUS_ID             = 0x0004,
    AI_BUFF_DEBUFF_CHALLENGE_ID       = 0x0005,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    DART_LNCH_OPENING_CNTDWN_ID       = 0x0105,
    ROBOT_STATE_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_MUSK_CMD_ID                  = 0x0204,
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    PROJECTILE_ALLOWANCE_CMD_ID       = 0x0208,
    ROBOT_RFID_STATUS_ID              = 0x0209,
    DART_ROBOT_INSTRUCTIONS_ID        = 0x020A,
    GROUND_ROBOT_POSITION_CMD_ID      = 0x020B,
    RADAR_MARK_DATA_CMD_ID            = 0x020C,
    SENTRY_INFO_CMD_ID                = 0x020D,
    RADAR_INFO_CMD_ID                 = 0x020E,
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
    DIY_controller_DATA_SEND_ID       = 0x0302,
    SMALL_MAP_INTERACTION_DATA_ID     = 0x0303,
    KEYBOARD_MOUSE_INFO_ID            = 0x0304,
    SMALL_MAP_DATA_RECEIPT_ID         = 0x0305,
    CUSTOM_CLIENT_DATA_CMD_ID         = 0x0306,
    CUSTOM_INFO_CMD_ID                = 0x0308,
    SENTRY_DECISION_CMD_ID            = 0x0120,
    RADAR_DECISION_CMD_ID             = 0x0121,
    IDCustomData,
}referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
