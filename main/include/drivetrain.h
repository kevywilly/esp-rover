#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include <stdio.h>
#include <math.h>
#include "globals.h"
#include "structs.h"

#ifndef RADIANS
#define RADIANS 0.0174533
#endif
    
#define MOTOR_FORWARD {1,0}
#define MOTOR_BACKWARD {0,1}
#define MOTOR_STOPPED {0,0}

float drivetrain_motor_get_duty(const drivetrain_config_t *drivetrain, int motor_id);
void drivetrain_motor_spin(const drivetrain_config_t *drivetrain, int motor_id, float power);
void drivetrain_motor_set_dir(const drivetrain_config_t *drivetrain, int motor_id, motor_direction_t dir);
void drivetrain_motor_set_power(const drivetrain_config_t *drivetrain, int motor_id, float power);

/**
 * @brief Spins a motor based on % power setting
 * 
 * @param drivetrain : drivetrain config
 * @param motor_id:  0..3
 * @param power:  % power (+/-)
 */
void drivetrain_motor_spin(const drivetrain_config_t *drivetrain, int motor_id, float power) {
    
    motor_direction_t dir = {0,0};

    if(power > 0) {
        dir.in1 = 1;
    }
    else if(power < 0) {
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
void drivetrain_motor_set_dir(const drivetrain_config_t *drivetrain, int motor_id, motor_direction_t dir) {
    ESP_LOGW(TAG, "Setting direction {%d, %d} for motor %d", dir.in1, dir.in2, motor_id);

    gpio_set_level(drivetrain->in1[motor_id], dir.in1);
    gpio_set_level(drivetrain->in2[motor_id], dir.in2);
}

float drivetrain_motor_get_duty(const drivetrain_config_t *drivetrain, int motor_id) {
    float duty = 0.0;
    switch(motor_id) {
        case 0:
            duty = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
            break;
        case 1:
            duty = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
            //mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
            break;
        case 2:
            duty =  mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A);
            //mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A);
            break;
        case 3:
            duty =  mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B);
            //mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B);
            break;
    }
    ESP_LOGW(TAG, "Motor %d duty = %.2f%%", motor_id, duty);
    return duty;
}

/**
 * @brief Sets motor power as a % of max duty cycle
 * 
 * @param drivetrain
 * @param motor_id: motor id 0..3
 * @param power: % of max duty cycle
 */
void drivetrain_motor_set_power(const drivetrain_config_t *drivetrain, int motor_id, float power) {
    ESP_LOGW(TAG, "Setting power %.2f%% for motor %d", power*100, motor_id);

    double duty = power * 99.0;
    duty = duty > 99.0 ? 99.0 : duty;


    switch(motor_id) {
        case 0:
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty * drivetrain->rpm_factor[0]);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
            //mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
            break;
        case 1:
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, duty * drivetrain->rpm_factor[1]);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM_DUTY_MODE_0);
            //mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
            break;
        case 2:
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, duty * drivetrain->rpm_factor[2]);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
            //mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A);
            break;
        case 3:
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B, duty * drivetrain->rpm_factor[3]);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
            //mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B);
            break;
    }
}

/**
 * @brief Configure drivetrain mnotors and encoders
 * 
 * @param drivetrain
 */
void drivetrain_init(const drivetrain_config_t * drivetrain) {

    uint64_t in1_in2_bit_mask = 0;
    uint64_t enca_bitmask = 0;

    for(int i=0; i < 4; i++) {
        in1_in2_bit_mask = (in1_in2_bit_mask | (1ULL << drivetrain->in1[i]) | (1ULL << drivetrain->in2[i]));
        enca_bitmask = (enca_bitmask | (1ULL << drivetrain->enca[i]));
    }

    // configure in1 and in2 pins
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = in1_in2_bit_mask;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // configure enc1 and ecb pins
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = enca_bitmask;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    

    // PWM see https://docs.espressif.com/projects/esp-idf/en/v4.4.2/esp32/api-reference/peripherals/mcpwm.html?highlight=mcpwm_unit_0

    // configure pwm pins
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, drivetrain->pwm[0]);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, drivetrain->pwm[1]);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, drivetrain->pwm[2]);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, drivetrain->pwm[3]);

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