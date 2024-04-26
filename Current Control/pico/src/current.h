#ifndef CURRENT_H   
#define CURRENT_H

#include "hermes.h"


#define ADC_INPUT   0
#define FREE_RUNNING true
#define RESISTANCE  0.04

// #define MA_FILTER
// #define MA_WINDOW_SIZE  10

void init_current();
float adc_to_current();
void update_current_blocking(float current, hermes_state *hermes);

#ifdef MA_FILTER
    float get_average_current(float newValue);
#endif

#endif