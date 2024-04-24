#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hermes.h"
#include "current.h"
#include "pico/multicore.h"




float get_current_value(hermes_state *hermes) {
    float ret;
    mutex_enter_blocking(&hermes->lock); // Enter critical section
    ret = hermes->current;
    mutex_exit(&hermes->lock); // Exit critical section
    return ret;
}