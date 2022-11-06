//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_STRUCTS_H
#define ESPROVER_STRUCTS_H

#include "driver/gpio.h"
#include "ultrasonic.h"

typedef struct {
    gpio_num_t pwm[4];
    ultrasonic_sensor_t sonars[4];
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
