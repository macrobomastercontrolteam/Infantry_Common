/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      CAN receive/transmit for the referee->main-controller bridge.
  *             Only the referee-info pass-through, UI info, power meter and
  *             supercap traffic remain; all motor control TX/RX has been removed.
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
#include "CAN_receive.h"

#include "main.h"
#include "string.h"

#include "detect_task.h"
#include "referee.h"
#include "custom_ui_task.h"
#include "user_lib.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

power_meter_can_rx_t power_meter_can_rx_msg;

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

void decode_power_meter(uint8_t *data);
void decode_supercap(uint8_t *data);

#if (SUPERCAP_TYPE == UBC_SUPERCAP)
capcan_rx_t capcan_rx_msg;
capcan_tx_t capcan_tx_msg;
void decode_ubc_cap_tx_data(uint8_t *data);
#elif (SUPERCAP_TYPE == MACRM_SUPERCAP)
capcan_rx_t capcan_rx_msg;
capcan_tx_t capcan_tx_msg;
void decode_macrm_cap_tx_data(uint8_t *data);
#elif (SUPERCAP_TYPE == SJTU_SUPERCAP)
supcap_t cap_message_rx;
#endif

/**
 * @brief          hal CAN fifo call back, receive referee-bridge related data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if (hcan == &GIMBAL_CAN)
	{
		switch (rx_header.StdId)
		{
			case CAN_POWER_METER_RX_ID:
			{
				decode_power_meter(rx_data);
				detect_hook(POWER_METER_TOE);
				break;
			}
			default:
			{
				break;
			}
		}
	}
	else if (hcan == &CHASSIS_CAN)
	{
		switch (rx_header.StdId)
		{
			case SUPCAP_RX_ID:
			{
				decode_supercap(rx_data);
				detect_hook(SUPCAP_TOE);
				break;
			}

			case CAN_UI_INFO_RX_ID: //receive robot status from main board to pass to UI
			{
				decode_ui_info(rx_data);
				detect_hook(MAIN_BOARD_TOE);
				break;
			}
			default:
			{
				break;
			}
		}
	}
}

void return_ref_info(uint8_t info_code)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_REF_INFO_PULL_RX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;

	chassis_can_send_data[0] = info_code;

	switch(info_code)
	{

		case BARREL_HEAT_LIMIT_AND_BARREL_1_HEAT:
		{
			uint16_t heat_limit = 0;
			uint16_t barrel_1_heat = 0;
			uint16_t robot_hp = 0;

			uint8_t game_started = 0;
			uint8_t team_color = 0;
			uint8_t flag_byte = 0;

			game_started = is_game_started();
			team_color = get_team_color();
			robot_hp = get_current_HP();

			get_shoot_heat0_limit_and_heat(&heat_limit, &barrel_1_heat);

			// Pack two flags into one byte
			Set_Bit(&flag_byte, 0, game_started);
			Set_Bit(&flag_byte, 1, team_color);

			memcpy(&chassis_can_send_data[1], &heat_limit, 2);
			memcpy(&chassis_can_send_data[3], &barrel_1_heat, 2);
			memcpy(&chassis_can_send_data[5], &flag_byte, 1);
			memcpy(&chassis_can_send_data[6], &robot_hp, 2);

			break;
		}

		case CHASSIS_POWER_INFO:
		{
			uint16_t power_buffer = 0;
			uint16_t power_limit = 0;
			int16_t power = 0;
			uint8_t module_power_byte = 0;

			get_chassis_power_data(&power_buffer,&power_limit); //pass power buffer and limit from refree system

			power = encode_float_as_int16(power_meter_can_rx_msg.chassis_power); //get power reading from powermeter and encode to 2 byte

			Set_Bit(&module_power_byte,0,robot_state.power_management_chassis_output); //pass module enable/disable cmd from refree system
			Set_Bit(&module_power_byte,1,robot_state.power_management_shooter_output);
			Set_Bit(&module_power_byte,2,robot_state.power_management_gimbal_output);

			memcpy(&chassis_can_send_data[1], &power_buffer, 2);
			memcpy(&chassis_can_send_data[3], &power_limit, 2);
			memcpy(&chassis_can_send_data[5], &power, 2);
			memcpy(&chassis_can_send_data[7], &module_power_byte, 1);
			break;
		}


	}
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void decode_ui_info(uint8_t *rx_data)
{

	ui_info.chassis_flag_byte = rx_data[0];
	ui_info.launcher_flag_byte = rx_data[1];

	//TODO: ADD SUPERCAP PERCENTAGE

}

#if (SUPERCAP_TYPE == MACRM_SUPERCAP)

void decode_macrm_cap_tx_data(uint8_t *data)
{
	capcan_tx_msg.current_chassis_power = (data[1] << 8) | data[0];
	capcan_tx_msg.current_battery_power = (data[3] << 8) | data[2];
	capcan_tx_msg.cap_voltage = (data[5] << 8) | data[4];
	capcan_tx_msg.cap_state = (data[7] << 8) | data[6];
}

uint16_t get_current_chassis_power(void)
{
	return capcan_tx_msg.current_chassis_power;
}
uint16_t get_current_battery_power(void)
{
	return capcan_tx_msg.current_battery_power;
}
int16_t get_cap_voltage(void)
{
	return capcan_tx_msg.cap_voltage;
}
uint16_t get_cap_state(void)
{
	return capcan_tx_msg.cap_state;
}

int16_t get_cap_energy_percentage(void)
{

	return ((int16_t)(capcan_tx_msg.cap_voltage - 10) / ( 23 - 10 ) * 100);
}
#endif

#if (SUPERCAP_TYPE == UBC_SUPERCAP)

void decode_ubc_cap_tx_data(uint8_t *data)
{
	capcan_tx_msg.max_discharge_power = (data[1] << 8) | data[0];
	capcan_tx_msg.base_power = (data[3] << 8) | data[2];
	capcan_tx_msg.cap_energy_percentage = (data[5] << 8) | data[4];
	capcan_tx_msg.cap_state = (data[7] << 8) | data[6];
}

uint16_t get_max_discharge_power(void)
{
	return capcan_tx_msg.max_discharge_power;
}
uint16_t get_current_chassis_power(void)
{
	return capcan_tx_msg.base_power;
}
int16_t get_cap_energy_percentage(void)
{
	return capcan_tx_msg.cap_energy_percentage;
}
uint16_t get_cap_state(void)
{
	return capcan_tx_msg.cap_state;
}
#endif

fp32 temp_power = 0;
void decode_power_meter(uint8_t *data)
{
    power_meter_can_rx_msg.chassis_current = (fp32)((int32_t)((data[3] << 8) | (int32_t)(data[2]))) / 100.0f;
	power_meter_can_rx_msg.chassis_voltage = (fp32)((int32_t)((data[1] << 8) | (int32_t)data[0])) / 100.0f;
	power_meter_can_rx_msg.chassis_power = power_meter_can_rx_msg.chassis_current * power_meter_can_rx_msg.chassis_voltage;
	temp_power = power_meter_can_rx_msg.chassis_power;
}

void decode_supercap(uint8_t *data)
{
#if (SUPERCAP_TYPE == SJTU_SUPERCAP)
	memcpy(cap_message_rx.can_buf, data, sizeof(cap_message_rx.can_buf));
#elif (SUPERCAP_TYPE == MACRM_SUPERCAP)
	decode_macrm_cap_tx_data(data);
#elif (SUPERCAP_TYPE == UBC_SUPERCAP)
	decode_ubc_cap_tx_data(data);
#endif
	detect_hook(SUPCAP_TOE);
}
