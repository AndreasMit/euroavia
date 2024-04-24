#include <stdio.h>
#include "pico/stdlib.h"
#include "hermes.h"
#include "pico/sync.h"
#include "hermes.h"
#include "current.h"
#include "pico/multicore.h"

/*  Global state    */
hermes_state hermes;


/*  Second core code.   */
//  1) Get Current
void core1_entry() {
    float current;
    while (1) {
        current = adc_to_current();
        update_current_blocking(current, &hermes);
    }
}


/*  Initialise mission peripherals. Runs on core0   */
void initialise_all() {
    /*  Initialises pico stuff  */
    stdio_init_all();
    
    /*  Initialises ADC for voltage reading */
    init_current();

    /* Initialise hermes_state mutex    */
    mutex_init(&hermes.lock);

    /*  Reset core1 to initial state    */
    multicore_reset_core1();

    /*  Launch core1 to run core1_entry function    */
    multicore_launch_core1(core1_entry);
    
}

int main() {

    // Initialises mission critical peripherals etc.
    initialise_all();
    
    while (1) {
        printf("Current reading...%f\n", get_current_value(&hermes));
    }
}