#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hermes.h"
#include "current.h"
#include "pico/multicore.h"


extern hermes_state hermes;

/*  Initialise hermes state struct. THIS MUST BE CALLED BEFORE INITIALISING CORE1!  */
void initialise_hermes_state() {
    hermes.current = 0.0;
    hermes.throttle = 0;
    hermes.control_enabled = false;
    hermes.previous_sample_time = 0;
    hermes.sample_time = 0;
    hermes.control_type = NO_CONTROL;
    mutex_init(&hermes.lock);
}


float get_current_value(hermes_state *hermes) {
    float ret;
    mutex_enter_blocking(&hermes->lock); // Enter critical section
    ret = hermes->current;
    mutex_exit(&hermes->lock); // Exit critical section
    return ret;
}