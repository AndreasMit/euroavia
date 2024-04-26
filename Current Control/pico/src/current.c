#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/sync.h"
#include "pico/time.h"
#include "current.h"
#include "hermes.h"

#ifdef DEBUG_MODE
    #include "debug.h"
#endif

#ifndef ADC_INPUT
    #error "Make sure ADC input is declared in source code."
#endif

#ifndef FREE_RUNNING
    #error "Make sure free running mode is declared in source code."
#endif

#ifndef RESISTANCE
    #error "Resistance has not been declares. Thus cannot convert voltage to current."
#endif

#ifdef MA_FILTER
    #ifndef MA_WINDOW_SIZE
        #error "MA_WINDOW_SIZE must be defined when MA_FILTER is defined"
    #endif
#endif



#ifdef MA_FILTER
    static float ma_current_buffer[MA_WINDOW_SIZE] = {0.0};
    static uint16_t ma_buffer_index = 0;
#endif


#ifdef MA_FILTER

    float get_average_current(float newValue) {
        float sum = 0, average = 0;
        // uint16_t samples_size = (ma_buffer_index > MA_WINDOW_SIZE) ? MA_WINDOW_SIZE : (ma_buffer_index+1);
        
        ma_current_buffer[ma_buffer_index++ % MA_WINDOW_SIZE] = newValue;

        for (uint16_t i = 0; i < MA_WINDOW_SIZE; ++i) {
            sum += ma_current_buffer[i];
        }

        average = sum / MA_WINDOW_SIZE;
        return average;
    }


#endif

/*  Initialise ADC for voltage measurement  */
void init_current() {
    adc_init();
    // Make sure GPIO is high-impedance (no pullups)
    adc_gpio_init(26);
    // Select ADC input
    adc_select_input(ADC_INPUT);
}


#ifdef DEBUG_MODE
    extern debug_vars debug_statements;
#endif

/*  Gets current from adc   */
float adc_to_current() {
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result = adc_read();
    float current = result * conversion_factor / RESISTANCE;
    
    #ifdef DEBUG_MODE
        mutex_enter_blocking(&debug_statements.lock);
        debug_statements.current_value_raw = current;
        mutex_exit(&debug_statements.lock);
    #endif
    
    return current;
}

/*  Update current value of hermes state    */
void update_current_blocking(float current, hermes_state *hermes) {
    mutex_enter_blocking(&hermes->lock); // Enter critical section
    hermes->current = current;
    hermes->previous_sample_time = hermes->sample_time;
    hermes->sample_time = to_us_since_boot(get_absolute_time());
    mutex_exit(&hermes->lock); // Exit critical section
}