//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_STRUCTS_H
#define ESPROVER_STRUCTS_H

#include "driver/gpio.h"

typedef struct {
    int in1;
    int in2;
} motor_direction_t;

typedef struct {
    gpio_num_t in1[4];
    gpio_num_t in2[4];
    gpio_num_t pwm[4];
    //gpio_num_t enca[4];
    float rpm_factor[4];
} drivetrain_config_t;

typedef struct {
    drivetrain_config_t drivetrain;
    gpio_num_t led_pin;
} robot_config_t;

typedef struct {
    float heading;
    float power;
    float turn;  // +/- percentage
} robot_move_t;

typedef struct {
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_generator_t generator;
    mcpwm_io_signals_t signal;
} pwm_params_t;

#endif //ESPROVER_STRUCTS_H
