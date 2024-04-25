#ifndef RECEIVER_H
#define RECEIVER_H

#include "hermes.h"

/* CONTROL_PIN: For changing control state */
#define CONTROL_PIN     5
/*  RECEIVE_PIN: For changing receive state */
#define RECEIVE_PIN     3


#define CONTROL_THRESHOLD 1700
#define CONTROL_RANGE   50


// If something goes wrong with the throttle output value
#define DEAD_ZONE_THROTTLE 1000 
#define THROTTLE_MIN    1000
#define THROTTLE_MAX    2000


void init_receiver_status();
void get_throttle_value(hermes_state *hermes);
void get_control_status(hermes_state *hermes);

#endif