#include "bsp_laser.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
void laser_on(void);
void laser_off(void);

void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}
void laser_enable(uint8_t fEnable)
{
    if (fEnable)
    {
        laser_on();
    }
    else
    {
        laser_off();
    }
}