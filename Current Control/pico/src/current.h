#ifndef CURRENT_H   
#define CURRENT_H

#include "hermes.h"


#define ADC_INPUT   0
#define FREE_RUNNING true
#define RESISTANCE  0.04


void init_current();
float adc_to_current();
void update_current_blocking(float current, hermes_state *hermes);


#endif