/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "stm32f4xx_hal.h"
#include "global_inc.h"

// Warning: redundant safety switch for shoot feature. Turn it on only if you know what you are doing.
#define ENABLE_SHOOT_REDUNDANT_SWITCH 0

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    // GM6020 CAN ID = 0x204 + ID, M2006 and M3508 CAN ID = 0x200 + ID
    /*******Chassis CAN IDs********/
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    CAN_YAW_MOTOR_4310_TX_ID = 0x005,
    CAN_YAW_MOTOR_4310_RX_ID = 0x0FF,
    CAN_POWER_METER_RX_ID = 0x212,

    /********Gimbal CAN IDs********/
    CAN_PIT_MOTOR_ID = 0x206,

    /********Other CAN IDs: Location depends on Model********/
    // By default: On chassis
    // INFANTRY_2023_MECANUM: On gimbal
    // INFANTRY_2024_MECANUM: On chassis
    // INFANTRY_2023_SWERVE: On chassis
    // SENTRY_2023_MECANUM: On chassis
    // INFANTRY_2024_BIPED: On chassis
    CAN_TRIGGER_MOTOR_ID = 0x207,

    CAN_FRICTION_MOTOR_LEFT_ID = 0x205, // friction1
    CAN_FRICTION_MOTOR_RIGHT_ID = 0x208, // friction2
} can_msg_id_e;

typedef enum
{
	MOTOR_INDEX_3508_M1 = 0,
	MOTOR_INDEX_3508_M2,
	MOTOR_INDEX_3508_M3,
	MOTOR_INDEX_3508_M4,
	MOTOR_INDEX_YAW,
	MOTOR_INDEX_PITCH,
	MOTOR_INDEX_TRIGGER,
  MOTOR_INDEX_FRICTION_LEFT,
  MOTOR_INDEX_FRICTION_RIGHT,
	MOTOR_LIST_LENGTH,
} can_motor_id_e;

typedef enum { //also update receiving end after change
   ALL = 1,
  
  ROBOT_ID,

  GAME_INFO,

  BARREL_HEAT_LIMIT_AND_BARREL_1_HEAT,

  PROJECTILE_ALLOWANCE_17MM,
  
  CHASSIS_POWER_INFO,
  
  CHASSIS_POWERMETER_DATA,
} request_ref_info_code_t;


typedef enum
{
	/*******Tx CAN IDs********/
  CAN_3508_OR_2006_LOW_RANGE_TX_ID = 0x200,
  CAN_3508_OR_2006_HIGH_RANGE_TX_ID = 0x1FF,
  CAN_6020_LOW_RANGE_TX_ID = 0x1FF,
  CAN_6020_HIGH_RANGE_TX_ID = 0x2FF,

#if (SUPERCAP_TYPE == MACRM_SUPERCAP)
  SUPCAP_TX_ID = 0x302,
  SUPCAP_RX_ID = 0x301,
#elif (SUPERCAP_TYPE == UBC_SUPERCAP)
  SUPCAP_TX_ID = 0x2C8,
  SUPCAP_RX_ID = 0x2C7,
#elif (SUPERCAP_TYPE == SJTU_SUPERCAP)
  SUPCAP_RX_ID = 0x301,
#endif

  CAN_REF_INFO_PULL_RX_ID = 0x130,
  CAN_REF_INFO_PULL_TX_ID = 0x131,

  CAN_UI_INFO_RX_ID = 0x135,
} can_other_msg_id_e;

typedef struct 
{
    fp32 chassis_current;
    fp32 chassis_voltage;
    fp32 chassis_power;
}power_meter_can_rx_t;

#if (SUPERCAP_TYPE == UBC_SUPERCAP)
typedef struct 
{
    uint16_t power_target;
    uint16_t referee_power;
    uint16_t rsvd1; //Must be 0x2012
    uint16_t rsvd2; //Must be 0x0712
}capcan_rx_t;

/*Message come from capacitor module */
/*Expected message frequency = 100Hz */
typedef struct
{
    uint16_t max_discharge_power;
    uint16_t base_power;
    int16_t cap_energy_percentage;
    uint16_t cap_state;
}capcan_tx_t;

typedef enum{
    CAP_OFF,
    CAP_READY,
    CAP_ON,
    VBUS_OVP,
    VBUS_UVP,
    VBAT_OVP,
}Cap_states_e;
#elif(SUPERCAP_TYPE == MACRM_SUPERCAP)
typedef struct 
{
    uint16_t power_target;
    uint16_t referee_power;
    uint16_t rsvd1; //Must be 0x2012
    uint16_t rsvd2; //Must be 0x0712
}capcan_rx_t;


/*Message come from capacitor module */
/*Expected message frequency = 100Hz */
typedef struct
{
    uint16_t current_chassis_power;
    uint16_t current_battery_power;
    int16_t cap_voltage;
    uint16_t cap_state;
}capcan_tx_t;

typedef enum{
    CAP_OFF,
    CAP_READY,
    CAP_ON,
    VBUS_OVP,
    VBUS_UVP,
    VBAT_OVP,
}Cap_states_e;
#elif (SUPERCAP_TYPE == SJTU_SUPERCAP)
typedef union
{
	uint8_t can_buf[8];
	struct
	{
		// 0: not provide power
		// 1: provide power
		uint8_t cap_state;
		uint8_t reserve;
		uint16_t cap_milivoltage;
		float cap_power;
	} cap_message;
} supcap_t;

extern supcap_t cap_message_rx;
#endif

typedef enum
{
    DM_8006 = 0,
    MA_9015 = 1,
    DM_4310 = 2,
    LAST_MIT_CONTROLLED_MOTOR_TYPE,
} MIT_controlled_motor_type_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t feedback_current;
    uint8_t temperate;
    int16_t last_ecd;
    fp32 output_angle; // rad
    fp32 velocity;     // rad/s
    fp32 torque;       // Nm
} motor_measure_t;

typedef enum{

  LAUNCHER_STATUS_UI,
  CHASSIS_STATUS_UI,

} send_ui_info_code_t;


extern void return_ref_info(uint8_t info_code);
void decode_ui_info(uint8_t *rx_data);

#if (SUPERCAP_TYPE == UBC_SUPERCAP)
void decode_ubc_cap_tx_data(uint8_t *data);
void decode_macrm_cap_tx_data(uint8_t *data);
extern uint16_t get_max_discharge_power(void);
extern uint16_t get_current_chassis_power(void);
extern int16_t get_cap_energy_percentage(void);
extern uint16_t get_cap_state(void);
void CAN_cmd_supercap(void);
void decode_supercap(uint8_t *data);
#endif

#if (SUPERCAP_TYPE == MACRM_SUPERCAP)
void decode_macrm_cap_tx_data(uint8_t *data);
extern uint16_t get_current_chassis_power(void);
extern uint16_t get_current_battery_power(void);
extern int16_t get_cap_voltage(void);
extern uint16_t get_cap_state(void);
extern int16_t get_cap_energy_percentage(void);
void CAN_cmd_supercap(void);
void decode_power_meter(uint8_t *data);
void decode_supercap(uint8_t *data);
#endif

#endif
