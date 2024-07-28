#include "bsp_gpio.h"
#include "main.h"


void pump_on(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}
void pump_off(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void pump_control(uint8_t on)
{
    if (on)
    {
        pump_on();
    }
    else
    {
        pump_off();
    }
}

void holder_control(uint8_t holder,uint8_t state)
{
    switch(holder)
    {
        case 0:
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, state);
            break;
        }
        case 1:
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, state);
            break;
        }
        case 2:
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, state);
            break;
        }
        case 3:
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, state);
            break;
        }
        default:
        {
            break;
        }
    }
    
}
