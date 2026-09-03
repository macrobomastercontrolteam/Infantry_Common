#include "bsp_gpio.h"
#include "main.h"


bool_t is_launcher_loaded(void)
{
    return !(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7)); //reverse reading because switch is active low
}
