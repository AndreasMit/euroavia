#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hermes.h"
#include "debug.h"


#ifdef DEBUG_MODE

#include "string.h"

void initialise_debug_statements(debug_vars *debug_statements) {
        debug_statements->integral_sum = 0.0;
        debug_statements->control_type = NO_CONTROL;
        debug_statements->current = 0.0;
        debug_statements->current_value_raw = 0.0;
        debug_statements->change_control_state = false;
        mutex_init(&debug_statements->lock);
}

void print_debug_statements(debug_vars *debug_statements) {
        float current_raw = 0, current = 0;
        char buffer[20];
        mutex_enter_blocking(&debug_statements->lock);
        current_raw = debug_statements->current_value_raw;
        current = debug_statements->current;
        mutex_exit(&debug_statements->lock);

        if (debug_statements->control_type == NO_CONTROL)
                strcpy(buffer, "NO CONTROL");
        else if (debug_statements->control_type == LOW_CONTROL)
                strcpy(buffer, "LOW CONTROL");
        else
                strcpy(buffer, "HIGH CONTROL");
        
        printf("time: %lu, current raw: %f, throttle: %lu, integral sum: %f, control type: %s, current: %f, zero integral: %d\n", debug_statements->time_now, current_raw, debug_statements->throttle, debug_statements->integral_sum, buffer, current, debug_statements->change_control_state);
}



#endif