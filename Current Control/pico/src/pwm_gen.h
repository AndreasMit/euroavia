#ifndef PWM_GEN_H
#define PWM_GEN_H

#include "pico/stdlib.h"

#define PWM_PIN 0

void write_microseconds(uint16_t millis);
void init_pwm();


#endif