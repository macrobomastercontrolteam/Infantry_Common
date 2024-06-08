#include "bsp_can.h"
#include "main.h"
#include "CAN_receive.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    // CAN filter setup refer to https://schulz-m.github.io/2017/03/23/stm32-can-id-filter/
    // Receives only the specified IDs in filter_id
    // To alow a new ID, add it to filter_id, and also add a &filter_mask
    uint32_t filter_id = (CAN_STEER_CONTROLLER_RX_ID | CAN_SWERVE_CONTROLLERE_RX_ID);
    uint32_t filter_mask = 0x1FFFFFFF & (~(CAN_STEER_CONTROLLER_RX_ID ^ CAN_SWERVE_CONTROLLERE_RX_ID));

    // Allows only one ID
    // uint32_t filter_id = CAN_SHRINKED_CONTROLLER_RX_ID;
    // uint32_t filter_mask = 0x1FFFFFFF;

    can_filter_st.FilterIdHigh = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13];
    can_filter_st.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
    can_filter_st.FilterMaskIdLow = 0xFFFF;
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}
