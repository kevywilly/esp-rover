//
// Created by Kevin Williams on 11/6/22.
//

#ifndef ESPROVER_SERVO_H
#define ESPROVER_SERVO_H

#include "driver/gpio.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"

#define SERVO_MIN_US 400
#define SERVO_MAX_US 2400

typedef enum {
    SERVO_ORIENTATION_NORMAL = 1,
    SERVO_ORIENTATION_REVERSE = -1
} servo_orientation_t;

typedef struct {
    gpio_num_t pwm_pin;
    mcpwm_unit_t uint;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_generator_t generator;
    mcpwm_io_signals_t signal;
    servo_orientation_t orientation;
} servo_t;

static const double servo_zero = (SERVO_MAX_US + SERVO_MIN_US) / 2.0;
static const double servo_duty_factor = (SERVO_MAX_US - SERVO_MIN_US) / 2.0;

static void servo_init(servo_t *servo) {
    mcpwm_gpio_init(servo->unit, servo->signal, servo->pwm_pin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(servo->unit, servo->timer, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

static void servo_spin(servo_t *servo, float power) {
    power = (power < -1 ? -1 : power > 1 ? 1 : power) * servo->orientation;

    float duty = power == 0 ? 0.0 : (servo_zero + power * servo_duty_factor) / 200.0;  // (100/20000)

    //duty = duty * (CONFIG_ESP_ROVER_MAX_POWER / 100.0);

    // set duty 50, 100, etc.  not .50, 1.0
    mcpwm_set_duty(
            servo->unit,
            servo->timer,
            servo->generator,
            duty
    );
    mcpwm_set_duty_type(servo->unit,
                        servo->timer,
                        servo->generator,
                        MCPWM_DUTY_MODE_0);
}

#endif //ESPROVER_SERVO_H
