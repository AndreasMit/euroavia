#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"




void initialise_mission_dependencies() {
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    adc_fifo_setup(true, true, 2, false, false);
    adc_irq_set_enabled(true);
    irq_set_exclusive_handler();
    irq_set_enabled(true);
    adc_run(true);
    adc_set_clkdiv(0);
}

int main() {
    stdio_init_all();
    
    initialise_mission_dependencies();

    
    while (1) {
        printf("Current reading...%f\n", get_current_value(&hermes));
    }
}