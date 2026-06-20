#include "bsp_rc.h"
#include "main.h"

#if (REMOTE_TYPE == REMOTE_USE_VT13)
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
#else
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
#endif

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
#if (REMOTE_TYPE == REMOTE_USE_VT13)
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
#else
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
#endif

    //enalbe idle interrupt
    //使能空闲中断
#if (REMOTE_TYPE == REMOTE_USE_VT13)
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
#else
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
#endif

    //disable DMA
    //失效DMA
#if (REMOTE_TYPE == REMOTE_USE_VT13)
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
#else
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
#endif


}
void RC_unable(void)
{
#if (REMOTE_TYPE == REMOTE_USE_VT13)
    __HAL_UART_DISABLE(&huart6);
#else
    __HAL_UART_DISABLE(&huart3);
#endif
}
void RC_restart(uint16_t dma_buf_num)
{
#if (REMOTE_TYPE == REMOTE_USE_VT13)
    __HAL_UART_DISABLE(&huart6);
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    hdma_usart6_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    __HAL_UART_ENABLE(&huart6);
#else
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);
#endif

}







