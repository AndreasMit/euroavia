#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "receiver.h"
#include "pico/time.h"
#include "hermes.h"


#ifndef CONTROL_PIN
    #error "CONTROL_PIN is not defined!"
#endif

#ifndef RECEIVE_PIN
    #error "RECEIVE_PIN is not defined!"
#endif


extern hermes_state hermes;

volatile absolute_time_t control_pin_start = 0;
volatile absolute_time_t control_pin_end = 0;

volatile absolute_time_t receive_pin_start = 0;
volatile absolute_time_t receive_pin_end = 0;

const uint8_t LED_PIN = 25;

/*  For some reason pico only allows 1 interrupt handler.
So pin will be passed as a param. */
void interrupt_handler(uint8_t gpio, uint32_t events) {

    switch (gpio) {
        case CONTROL_PIN:
            
            if (gpio_get(CONTROL_PIN)) {    // pulse is high
                control_pin_start = to_us_since_boot(get_absolute_time());
            } else {    // pulse is low

                // safety. If there has not been a high pulse first then dont run!
                if (!control_pin_start)
                    return;

                control_pin_end = to_us_since_boot(get_absolute_time());
                // hermes.control_enabled = (control_pin_end - control_pin_start) >= CONTROL_THRESHOLD - CONTROL_RANGE;
                absolute_time_t pulse_duration = control_pin_end - control_pin_start;

                hermes.control_type = (pulse_duration >= LOW_CONTROL_THREASHOLD + CONTROL_RANGE) ? HIGH_CONTROL:
                (pulse_duration <= LOW_CONTROL_THREASHOLD - CONTROL_RANGE) ? NO_CONTROL:
                ((pulse_duration >= LOW_CONTROL_THREASHOLD - CONTROL_RANGE) && (pulse_duration <= LOW_CONTROL_THREASHOLD + CONTROL_RANGE)) ? LOW_CONTROL
                : NO_CONTROL;
                
                gpio_put(LED_PIN, (bool)((control_pin_end - control_pin_start) >= LOW_CONTROL_THREASHOLD - CONTROL_RANGE));
            }

            break;
        case RECEIVE_PIN:
            
            if (gpio_get(RECEIVE_PIN)) {
                receive_pin_start = to_us_since_boot(get_absolute_time());

            } else {
                // safety. If there has been no high pulse interrupt first dont run!
                if (!receive_pin_start)
                    return;

                receive_pin_end = to_us_since_boot(get_absolute_time());

                absolute_time_t throttle_value = receive_pin_end - receive_pin_start;

                /* constrain value from THROTTLE_MIN to THROTTLE_MAX  */
                throttle_value = constrain(throttle_value, THROTTLE_MIN, THROTTLE_MAX);
                hermes.throttle = throttle_value;
            }

            break;
        default:
            break;
    }
}



/*  Initialise gpio interrupts  */
void init_receiver_status() {
    gpio_init(LED_PIN); // on when control is enabled, off when control is disabled.
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_set_irq_enabled_with_callback(RECEIVE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interrupt_handler);
    gpio_set_irq_enabled_with_callback(CONTROL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interrupt_handler);
}
