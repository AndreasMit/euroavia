#include <stdio.h>
#include "pico/stdlib.h"
#include "control.h"
#include "hermes.h"
#include "receiver.h"
#include "pico/sync.h"
#include "pico/time.h"

#ifdef DEBUG_MODE
    #include "debug.h"
    extern debug_vars debug_statements;
#endif

#ifndef TARGET
    #error "Control target undefined!"
#endif

#ifndef KP
    #error "Proportional control term undefined!"
#endif

#ifndef KI
    #error "Integral control term undefined!"
#endif

static float integral_sum = 0;


/*  PI algorithm for throttle control based on current  */
uint16_t control_throttle(hermes_state *hermes, bool first_time_control_flag) {
    float uc=0, error=0, current=0, dt=0, throttle_value_01=0;
    uint16_t throttle = 0;
    /*  Acquire and release mutex as quickly as possible    */
    mutex_enter_blocking(&hermes->lock);
    current = hermes->current;
    dt = (hermes->sample_time - hermes->previous_sample_time) / 1e6; // [s]
    mutex_exit(&hermes->lock);
    throttle = hermes->throttle;
    /*  Released mutex  */
    

    // Control logic below...

    // if this is the first control loop make sure integral sum is 0
    if (first_time_control_flag)
        integral_sum = 0;

    throttle_value_01 = (throttle - (float)THROTTLE_MIN) / (float)(THROTTLE_MAX - THROTTLE_MIN);

    error = TARGET - current;
    integral_sum += error*dt;

    if (current > TARGET || (current < TARGET && throttle_value_01 > 0.6))
        uc = CTRL_CENTER_U + KP * error + KI * integral_sum;
    else {
        integral_sum = 0;
        uc = throttle_value_01;
    }

    if (uc > throttle_value_01)
        uc  = throttle_value_01;

    if (current < TARGET - 5)
        integral_sum = 0;

    uc = constrain(uc, 0, 1);
    throttle = constrain((int)(uc * 1000 + 1000), 1000, 2000);

    #ifdef DEBUG_MODE
        debug_statements.integral_sum = integral_sum;
    #endif

    // printf("time: %lu, dt: %f, current: %f, throttle: %u, integral_sum: %f\n", to_ms_since_boot(get_absolute_time()), dt, current, throttle, integral_sum);
    return throttle;
}