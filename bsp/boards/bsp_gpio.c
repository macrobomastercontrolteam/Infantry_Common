#include "bsp_gpio.h"
#include "main.h"

void head_pump_control(uint8_t on)
{
    if (on)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
}

void storage_pump_control(uint8_t on)
{
    if (on)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

// void holder_control(uint8_t holder,uint8_t state)
// {
//     switch(holder)
//     {
//         case 0:
//         {
//             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, state);
//             break;
//         }
//         case 1:
//         {
//             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, state);
//             break;
//         }
//         case 2:
//         {
//             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, state);
//             break;
//         }
//         case 3:
//         {
//             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, state);
//             break;
//         }
//         default:
//         {
//             break;
//         }
//     }
    
// }
