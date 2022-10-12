//
// Created by Kevin Williams on 9/23/22.
//

#ifndef ESPROVER_GLOBALS_H
#define ESPROVER_GLOBALS_H


#include <esp_http_server.h>
#include "structs.h"

static const char *TAG = "ESP Rover";
static bool auto_mode = true;
static httpd_handle_t server = NULL;
static const float ROTATION_MATRIX[4]  = {1.0, -1.0, -1.0, 1.0};

static uint16_t distances[4] = {8191,8191,8191, 8191};
static uint16_t recommended_heading = 90;

static  const robot_config_t robot = {
        .drivetrain = {
                .in1 = {(gpio_num_t) CONFIG_M1_IN1, (gpio_num_t)CONFIG_M2_IN1, (gpio_num_t)CONFIG_M3_IN1,  (gpio_num_t)CONFIG_M4_IN1},
                .in2 = {(gpio_num_t)CONFIG_M1_IN2, (gpio_num_t)CONFIG_M2_IN2, (gpio_num_t)CONFIG_M3_IN2,  (gpio_num_t)CONFIG_M4_IN2},
                .pwm = {(gpio_num_t)CONFIG_M1_PWM, (gpio_num_t)CONFIG_M2_PWM, (gpio_num_t)CONFIG_M3_PWM,  (gpio_num_t)CONFIG_M4_PWM},
                .enca = {(gpio_num_t)CONFIG_M1_ENCA, (gpio_num_t)CONFIG_M2_ENCA, (gpio_num_t)CONFIG_M3_ENCA,  (gpio_num_t)CONFIG_M4_ENCA},
                .rpm_factor = {CONFIG_POWER_ADJUST1 / 100.0, CONFIG_POWER_ADJUST2 / 100.0, CONFIG_POWER_ADJUST3 / 100.0, CONFIG_POWER_ADJUST4 / 100.0}
        },
        .led_pin = GPIO_NUM_13
};

#endif //ESPROVER_GLOBALS_H
