#ifndef BSP_LED_H
#define BSP_LED_H
#include "global_inc.h"

/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
extern void aRGB_led_show(uint32_t aRGB);


#endif
