#ifndef CONTROL_H
#define CONTROL_H

#include "hermes.h"

#define KP (float)0.008
#define KI (float)0.055

#define TARGET_LOW (float)20
#define CTRL_CENTER_U_LOW (float)0.75
#define TARGET_HIGH (float)25
#define CTRL_CENTER_U_HIGH (float)0.8

uint16_t control_throttle(hermes_state *hermes, bool first_time_control_flag);


#endif