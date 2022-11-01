//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_STRUCTS_H
#define ESPROVER_STRUCTS_H

#include "driver/gpio.h"

typedef struct {
    int in1;
    int in2;
} MotorDirection;

typedef struct {
    gpio_num_t in1[4];
    gpio_num_t in2[4];
    gpio_num_t pwm[4];
    //gpio_num_t enca[4];
    double rpm_factor[4];
} DrivetrainConfig;

typedef struct {
    DrivetrainConfig drivetrain;
    gpio_num_t led_pin;
} RobotConfig;

typedef struct {
    double heading;
    double power;
    double turn;  // +/- percentage
} DriveCommand;

typedef struct {
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_generator_t generator;
    mcpwm_io_signals_t signal;
} PWMParams;

#endif //ESPROVER_STRUCTS_H
