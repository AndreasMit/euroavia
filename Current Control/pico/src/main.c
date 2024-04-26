#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "hermes.h"
#include "receiver.h"
#include "control.h"
#include "pwm_gen.h"
#include "current.h"

#ifdef DEBUG_MODE
    #include "debug.h"
#endif

/*  Global state    */
hermes_state hermes;

#ifdef DEBUG_MODE
    debug_vars debug_statements;
#endif

// uint64_t counter = 0;
// uint64_t counter2 = 0;

// bool print_counter() {
//     printf("Counter: %llu , Counter2: %llu\n", counter, counter2);
//     counter = 0;
//     counter2 = 0;
//     return true;
// }

/*  Second core code.   */
//  1) Get Current
void core1_entry() {
    float current;
    while (1) {
        current = adc_to_current();
        #ifdef MA_FILTER
            current = get_average_current(current);
            #ifdef DEBUG_MODE
                mutex_enter_blocking(&debug_statements.lock);
                debug_statements.current = current;
                mutex_exit(&debug_statements.lock);
            #endif
        #endif
        update_current_blocking(current, &hermes);
        // ++counter;
    }
}


/*  Initialise mission peripherals. Runs on core0   */
void initialise_all() {
    /*  Initialises pico stuff  */
    stdio_init_all();
    
    /*  Initialises ADC for voltage reading */
    init_current();

    /* Initialise hermes_state mutex    */
    initialise_hermes_state();

    /*  Initialise debug statements */
    #ifdef DEBUG_MODE
        initialise_debug_statements(&debug_statements);
    #endif

    /* Initialise interrupts handler    */
    init_receiver_status();

    /*  Reset core1 to initial state    */
    multicore_reset_core1();

    /*  Launch core1 to run core1_entry function    */
    multicore_launch_core1(core1_entry);
    
    /*  Initialise pwm generator    */
    init_pwm();
}

int main() {

    initialise_all();

    // struct repeating_timer timer;
    // add_repeating_timer_ms(1000, print_counter, NULL, &timer);
    
    float current;
    int64_t throttle;
    bool previous_control_enabled = hermes.control_enabled;
    while (1) {
        /*  Update hermes status values BEGIN   */
        // get_control_status(&hermes);
        // get_throttle_value(&hermes);
        current = get_current_value(&hermes); // this must aqcuire lock first.
        /*  Update hermes status values END */
        // printf("CONTROL ENABLED: %d\n", hermes.control_enabled);
        throttle = hermes.throttle;
        if (!hermes.control_enabled) {
            // printf("%f, %u, %d\n", current, throttle, hermes.control_enabled);
            previous_control_enabled = false;
        }
        else {
            throttle = control_throttle(&hermes, !previous_control_enabled);
            previous_control_enabled = true;
        }

        #ifdef DEBUG_MODE
            debug_statements.time_now = to_ms_since_boot(get_absolute_time());
            debug_statements.throttle = throttle;
            debug_statements.control_enabled = hermes.control_enabled;
        #endif

        /*  Write PWM pulse logic here  */
        write_microseconds(throttle);


        // ++counter2;
        #ifdef DEBUG_MODE
            print_debug_statements(&debug_statements);
        #endif
    }
}