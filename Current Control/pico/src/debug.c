#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "debug.h"
#include "hermes.h"


#ifdef DEBUG_MODE

void initialise_debug_statements(debug_vars *debug_statements) {
        debug_statements->integral_sum = 0.0;
        mutex_init(&debug_statements->lock);
}

void print_debug_statements(debug_vars *debug_statements) {
        float current_raw = 0, current = 0;
        mutex_enter_blocking(&debug_statements->lock);
        current_raw = debug_statements->current_value_raw;
        current = debug_statements->current;
        mutex_exit(&debug_statements->lock);
        printf("time: %lu, current raw: %f, throttle: %lu, integral sum: %f, control enabled: %d, current: %f\n", debug_statements->time_now, current_raw, debug_statements->throttle, debug_statements->integral_sum, debug_statements->control_enabled, current);
}



#endif