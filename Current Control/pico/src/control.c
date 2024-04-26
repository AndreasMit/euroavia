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

#ifndef TARGET_LOW
    #error "Target LOW undefined!"
#endif

#ifndef TARGET_HIGH
    #error "Target HIGH undefined!"
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

    const float target = (hermes->control_type == HIGH_CONTROL) ? TARGET_HIGH : TARGET_LOW;
    const float ctrl_center = (hermes->control_type == HIGH_CONTROL) ? CTRL_CENTER_U_HIGH : CTRL_CENTER_U_LOW;

    // if this is the first control loop make sure integral sum is 0
    if (first_time_control_flag)
        integral_sum = 0;

    throttle_value_01 = (throttle - (float)THROTTLE_MIN) / (float)(THROTTLE_MAX - THROTTLE_MIN);

    error = target - current;
    integral_sum += error*dt;

    #ifdef DEBUG_MODE
        debug_statements.integral_sum = integral_sum;
    #endif

    if (current > target || (current < target && throttle_value_01 > 0.6))
        uc = ctrl_center + KP * error + KI * integral_sum;
    else {
        integral_sum = 0;
        uc = throttle_value_01;
    }

    if (uc > throttle_value_01)
        uc  = throttle_value_01;

    if (current < target * 0.5)
        integral_sum = 0;

    uc = constrain(uc, 0, 1);
    throttle = constrain((int)(uc * 1000 + 1000), 1000, 2000);


    return throttle;
}