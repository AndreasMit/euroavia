#include <stdio.h>
#include "hermes.h"
#include "hardware/pwm.h"
#include "pwm_gen.h"
#include "pico/stdlib.h"
#include "receiver.h"

#ifndef PWM_PIN
    #error "No PWM PIN specified!"
#endif



void write_microseconds(uint16_t millis) {
    millis = constrain(millis, THROTTLE_MIN, THROTTLE_MAX);
    pwm_set_gpio_level(PWM_PIN, (millis/20000.f)*39062.f);
}

/*  Initialise pwm hardware generator   */
void init_pwm() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.f);
    pwm_config_set_wrap(&config, 39062.f);
    pwm_init(slice_num, &config, true);

}