/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310 magnetic compass middleware, complete IST8310 IIC communication, because set MPU6500 SPI communication
  *            so set to read through mpu6500's IIC_SLV0, write through IIC_SLV4.
  * @note       IST8310 only supports IIC communication
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "global_inc.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)  //IIC address of IST8310
#define IST8310_IIC_READ_MSB (0x80) //SPI read the first bit sent by IST8310 is 1

extern void ist8310_GPIO_init(void); //ist8310 io init
extern void ist8310_com_init(void);  //ist8310 communication init
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void); //reset io set high
extern void ist8310_RST_L(void); //reset io set low, low will cause ist8310 to restart

#endif
