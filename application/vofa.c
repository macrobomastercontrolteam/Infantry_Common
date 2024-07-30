#include "vofa.h"


DataPacket vofa_data_packet = {
    .data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f},
    .tail = {0x00, 0x00, 0x80, 0x7f}};


void vofa_send(void)
{
    
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&vofa_data_packet, 36);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    
}
void vofa_receive(void)
{
}
fp32 vofa_data_get(uint8_t place)
{
    return vofa_data_packet.data[place];
}

void vofa_update(uint8_t place, fp32 data)
{
    vofa_data_packet.data[place] = data;
}

