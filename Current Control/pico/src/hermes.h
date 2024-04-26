#ifndef HERMES_H
#define HERMES_H

#include "pico/stdlib.h"
#include "pico/sync.h"

#define DEBUG_MODE /* Running in debugging mode  (prints and stuff...)  */

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct _hermes_state {
    float current;
    volatile uint16_t throttle;
    volatile bool control_enabled;
    mutex_t lock;   // must be captured before accessing current!!!!
    uint64_t sample_time;
    uint64_t previous_sample_time; 
} hermes_state;

// void initialise_all(hermes_state *hermes);
// void core1_entry();

/*  Acquires lock. Gets value. Releases lock. Returns value  */
float get_current_value(hermes_state *hermes);
void initialise_hermes_state();

#endif