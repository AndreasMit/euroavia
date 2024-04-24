#ifndef HERMES_H
#define HERMES_H

#include "pico/stdlib.h"
#include "pico/sync.h"


typedef struct _hermes_state {
    float current;
    uint16_t throttle;
    bool control_enabled;
    mutex_t lock;   // must be captured before accessing current!!!!
} hermes_state;

// void initialise_all(hermes_state *hermes);
// void core1_entry();

/*  Acquires lock. Gets value. Releases lock. Returns value  */
float get_current_value(hermes_state *hermes);

#endif