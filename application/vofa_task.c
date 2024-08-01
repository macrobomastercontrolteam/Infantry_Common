#include "vofa_task.h"

uint8_t RxBuffer[1];
uint16_t RxLine = 0;
uint8_t DataBuff[200];



DataPacket vofa_data_packet = {
    .data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f},
    .tail = {0x00, 0x00, 0x80, 0x7f}};

void vofa_init(void)
{
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)&vofa_data_packet, 36);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);
}

void vofa_send(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)&vofa_data_packet, 36);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if(UartHandle->Instance==USART1)
    {
        HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);
        RxLine++;
        DataBuff[RxLine - 1] = RxBuffer[0];
        if (RxBuffer[0] == 0x21)
        {

            vofa_update(get_place(),get_data());
            memset(DataBuff, 0, sizeof(DataBuff));
            RxLine = 0;
        }
        RxBuffer[0] = 0;
        
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

fp32 get_data(void)
{
    uint8_t data_Start_Num = 0;
    uint8_t data_End_Num = 0;
    uint8_t data_Num = 0; 
    uint8_t minus_Flag = 0; 
    fp32 data_return = 0; 
    uint8_t pos = 0;
    int8_t power = 0;
    for(uint8_t i=0;i<200;i++) 
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; 
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') 
    {
        data_Start_Num += 1;
        minus_Flag = 1; 
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    power = data_Num-6; 
    while(pos < data_Num)
    {
        if(DataBuff[data_Start_Num+pos] == '.')
        {
        }
        else
        {
            data_return += (DataBuff[data_Start_Num+pos]-48)*(pow(10,power));
            power--;
        }
        pos++;
    }
    if(minus_Flag == 1)  data_return = -data_return;
    return data_return;
}

uint8_t get_place(void)
{
    return (DataBuff[1]-48);
}

fp32 vofa_return_data(uint8_t place)
{
    return vofa_data_packet.data[place];
}

void vofa_update(uint8_t place, fp32 data)
{
    vofa_data_packet.data[place] = data;
}


void vofa_task(void const * argument)
{
    vofa_init();

    while(1)
    {
        vofa_send();
    }
}
