#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include <stdio.h>
#include <math.h>
#include "globals.h"
#include "structs.h"

#define SERVO_MIN_US 400
#define SERVO_MAX_US 2400

static const double servo_zero = (SERVO_MAX_US + SERVO_MIN_US)/2.0;
static const double servo_duty_factor = (SERVO_MAX_US - SERVO_MIN_US)/2.0;

void drivetrain_init(RobotConfig *drivetrain);
void drivetrain_motor_spin(int motor_id, double power);

/**
 * @brief Spins a motor based on % power setting
 * 
 * @param drivetrain : drivetrain config
 * @param motor_id:  0..3
 * @param power:  % power (+/-)
 */
void drivetrain_motor_spin(int motor_id, double power) {

    int orientation = (motor_id == 1 || motor_id == 2) ? 1 : -1;
    power = power < -1 ? -1 : power > 1 ? 1 : power;

    power = power * orientation;

    double duty = power == 0 ? 0.0 : (servo_zero + power*servo_duty_factor)/200.0;  // (100/20000)

    // set duty 50, 100, etc.  not .50, 1.0
    mcpwm_set_duty(
            PWM_PARAMS[motor_id].unit,
            PWM_PARAMS[motor_id].timer,
            PWM_PARAMS[motor_id].generator,
            duty
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
void drivetrain_init(RobotConfig *drivetrain) {

    // PWM see https://docs.espressif.com/projects/esp-idf/en/v4.4.2/esp32/api-reference/peripherals/mcpwm.html?highlight=mcpwm_unit_0

    // configure pwm pins
    for (int i = 0; i < 4; i++) {
        mcpwm_gpio_init(PWM_PARAMS[i].unit, PWM_PARAMS[i].signal, drivetrain->pwm[i]);
    }

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings

}

#endif