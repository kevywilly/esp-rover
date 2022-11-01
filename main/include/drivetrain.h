#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include <stdio.h>
#include <math.h>
#include "globals.h"
#include "structs.h"

void drivetrain_init(DrivetrainConfig *drivetrain);

void drivetrain_motor_spin(DrivetrainConfig *drivetrain, int motor_id, double power);

void drivetrain_motor_set_dir(DrivetrainConfig *drivetrain, int motor_id, MotorDirection dir);

void drivetrain_motor_set_power(DrivetrainConfig *drivetrain, int motor_id, double power);

/**
 * @brief Spins a motor based on % power setting
 * 
 * @param drivetrain : drivetrain config
 * @param motor_id:  0..3
 * @param power:  % power (+/-)
 */
void drivetrain_motor_spin(DrivetrainConfig *drivetrain, int motor_id, double power) {

    MotorDirection dir = {0, 0};

    if (power > 0) {
        dir.in1 = 1;
    } else if (power < 0) {
        dir.in2 = 1;
    }

    drivetrain_motor_set_dir(drivetrain, motor_id, dir);
    drivetrain_motor_set_power(drivetrain, motor_id, fabs(power));

}

/**
 * @brief Sets motor spin direction via motor controller in1 and in2
 * 
 * @param drivetrain
 * @param motor: motor id 0..3 
 * @param dir: reverse, forward, stopped
 */
void drivetrain_motor_set_dir(DrivetrainConfig *drivetrain, int motor_id, MotorDirection dir) {
    ESP_LOGW(TAG, "Setting direction {%d, %d} for motor %d", dir.in1, dir.in2, motor_id);

    gpio_set_level(drivetrain->in1[motor_id], dir.in1);
    gpio_set_level(drivetrain->in2[motor_id], dir.in2);
}

/**
 * @brief Sets motor power as a % of max duty cycle
 * 
 * @param drivetrain
 * @param motor_id: motor id 0..3
 * @param power: % of max duty cycle
 */
void drivetrain_motor_set_power(DrivetrainConfig *drivetrain, int motor_id, double power) {
    ESP_LOGW(TAG, "Setting power %.2f%% for motor %d", power * 100, motor_id);

    double duty = power * 99.0;
    duty = duty > 99.0 ? 99.0 : duty;

    mcpwm_set_duty(
            PWM_PARAMS[motor_id].unit,
            PWM_PARAMS[motor_id].timer,
            PWM_PARAMS[motor_id].generator,
            duty * drivetrain->rpm_factor[motor_id]
    );
    mcpwm_set_duty_type(PWM_PARAMS[motor_id].unit,
                        PWM_PARAMS[motor_id].timer,
                        PWM_PARAMS[motor_id].generator,
                        MCPWM_DUTY_MODE_0);
}

/**
 * @brief Configure drivetrain mnotors and encoders
 * 
 * @param drivetrain
 */
void drivetrain_init(DrivetrainConfig *drivetrain) {

    uint64_t in1_in2_bit_mask = 0;
    //uint64_t enca_bitmask = 0;

    for (int i = 0; i < 4; i++) {
        in1_in2_bit_mask = (in1_in2_bit_mask | (1ULL << drivetrain->in1[i]) | (1ULL << drivetrain->in2[i]));
        //enca_bitmask = (enca_bitmask | (1ULL << drivetrain->enca[i]));
    }

    // configure in1 and in2 pins
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = in1_in2_bit_mask;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    /* We are not using encoders
    // configure enc1 and ecb pins
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = enca_bitmask;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
     */

    // PWM see https://docs.espressif.com/projects/esp-idf/en/v4.4.2/esp32/api-reference/peripherals/mcpwm.html?highlight=mcpwm_unit_0

    // configure pwm pins
    for (int i = 0; i < 4; i++) {
        mcpwm_gpio_init(PWM_PARAMS[i].unit, PWM_PARAMS[i].signal, drivetrain->pwm[i]);
    }

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 80;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings

}

#endif