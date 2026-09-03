#ifndef GIMBAL_INTERFACE_TYPES_H
#define GIMBAL_INTERFACE_TYPES_H

#include "global_inc.h"

typedef struct
{
  fp32 pos;
  fp32 vel;
  fp32 KP;
  fp32 KD;
  fp32 torq;
} MIT_control_variable_t;

typedef struct
{
  MIT_control_variable_t pitch_MIT_variable;
  MIT_control_variable_t pitch_base_MIT_variable;
} MIT_control_motor_t;

#endif
