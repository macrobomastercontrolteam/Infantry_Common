#include "bsp_servo_pwm.h"
#include "main.h"

#define SERVO_MIN_PWM 500
#define SERVO_MAX_PWM 2500
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void servo_pwm_set(uint16_t pwm)
{
    uint16_t set_pwm = pwm;
    if (set_pwm < SERVO_MIN_PWM)
    {
        set_pwm = SERVO_MIN_PWM;
    }
    else if (set_pwm > SERVO_MAX_PWM)
    {
        set_pwm = SERVO_MAX_PWM;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, set_pwm);
}

void servo_open()
{
    servo_pwm_set(2100);
}

void servo_close()
{
    servo_pwm_set(1300);
}
