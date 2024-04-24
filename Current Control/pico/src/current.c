#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/sync.h"
#include "current.h"
#include "hermes.h"

#ifndef ADC_INPUT
    #error "Make sure ADC input is declared in source code."
#endif

#ifndef FREE_RUNNING
    #error "Make sure free running mode is declared in source code."
#endif

#ifndef RESISTANCE
    #error "Resistance has not been declares. Thus cannot convert voltage to current."
#endif


/*  Initialise ADC for voltage measurement  */
void init_current() {
    adc_init();
    // Make sure GPIO is high-impedance (no pullups)
    adc_gpio_init(26);
    // Select ADC input
    adc_select_input(ADC_INPUT);
    // enable/disable free running mode
    // adc_run(FREE_RUNNING);
}

/*  Gets current from adc   */
float adc_to_current() {
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result = adc_read();
    float current = result * conversion_factor;
    // printf("Current from adc_to_current: %f\n", current);
    return current;
}


void update_current_blocking(float current, hermes_state *hermes) {
    mutex_enter_blocking(&hermes->lock); // Enter critical section
    hermes->current = current;
    mutex_exit(&hermes->lock); // Exit critical section
}