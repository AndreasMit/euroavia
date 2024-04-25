#include <stdio.h>
#include "pico/stdlib.h"
#include "hermes.h"
#include "pico/sync.h"
#include "hermes.h"
#include "current.h"
#include "pico/multicore.h"
#include "receiver.h"
#include "control.h"

/*  Global state    */
hermes_state hermes;

uint64_t counter = 0;
uint64_t counter2 = 0;
bool print_counter() {
    printf("Counter: %llu , Counter2: %llu\n", counter, counter2);
    counter = 0;
    counter2 = 0;
    return true;
}

/*  Second core code.   */
//  1) Get Current
void core1_entry() {
    float current;
    while (1) {
        current = adc_to_current();
        #ifdef MA_FILTER
            current = get_average_current(current);
        #endif
        update_current_blocking(current, &hermes);
        ++counter;
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

    /* Initialise interrupts handler    */
    init_receiver_status();

    /*  Reset core1 to initial state    */
    multicore_reset_core1();

    /*  Launch core1 to run core1_entry function    */
    multicore_launch_core1(core1_entry);
    
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
        
        if (!hermes.control_enabled) {
            printf("%f, %u, %d\n", current, hermes.throttle, hermes.control_enabled);
            previous_control_enabled = false;
        }
        else {
            throttle = control_throttle(&hermes, !previous_control_enabled);
            previous_control_enabled = true;
        }

        /*  Write PWM pulse logic here  */

        ++counter2;
    }
}