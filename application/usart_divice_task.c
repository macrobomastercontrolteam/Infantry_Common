#include "usart_divice_task.h"
#if USART_DIVICE

uint8_t RxBuffer[1];
uint16_t RxLine = 0;
uint8_t TxBuff[4] = {0x80, 0x06, 0x02, 0x78};
uint8_t DiviceCmd[5] = {0};//don't touch?

uint8_t DataBuff[12] = {0};
float_t Distence = 0;

void uart_init(void)
{
  HAL_UART_Transmit_IT(&huart1, (uint8_t *)&TxBuff, 4);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)DataBuff, 12);
}

void uart_send(void)
{
  HAL_UART_Transmit_IT(&huart1, (uint8_t *)&TxBuff, 4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if (UartHandle->Instance == USART1)
  {
    data_update();                                         // Process the received data
    HAL_UART_Receive_IT(&huart1, (uint8_t *)DataBuff, 12); // Restart reception
  }
}

void data_update()
{
  float_t result = 0;
  if (DataBuff[3] == 'E' && DataBuff[4] == 'R' && DataBuff[5] == 'R')
  {
    Distence = 0.122f; // error measurement
  }
  else // caculate the received distence value
  {
    Distence = ((DataBuff[3] - 0x30) * 100 + (DataBuff[4] - 0x30) * 10 + (DataBuff[5] - 0x30) * 1 + (DataBuff[7] - 0x30) * 0.1 + (DataBuff[8] - 0x30) * 0.01 + (DataBuff[9] - 0x30) * 0.001);
  }
}

void usart_divice_task(void const *argument)
{
  uart_init();
  while (1)
  {
    uart_send();
    osDelay(300);
  }
}

#endif
