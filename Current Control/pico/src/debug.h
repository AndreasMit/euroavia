#ifndef DEBUG_H
#define DEBUG_H

#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hermes.h"
/* Only for debugging purposes. In a production environment this should not be included */

#ifdef DEBUG_MODE

typedef struct _debug_vars {
    int16_t throttle;
    float current;
    float current_value_raw;
    bool control_enabled;
    int32_t time_now;
    float integral_sum;
    enum Control_Level control_type;
    bool change_control_state;
    mutex_t lock;
} debug_vars;

void initialise_debug_statements(debug_vars *debug_statements);
void print_debug_statements(debug_vars *debug_statements);

#endif

#endif