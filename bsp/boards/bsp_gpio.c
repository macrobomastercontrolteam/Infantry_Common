#include "bsp_gpio.h"
#include "main.h"


bool_t is_launcher_loaded(void)
{
    return (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)); 
}
