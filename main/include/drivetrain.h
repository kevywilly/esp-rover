#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include <stdio.h>
#include <math.h>

#ifndef RADIANS
#define RADIANS 0.0174533
#endif
    
#define MOTOR_FORWARD {1,0}
#define MOTOR_BACKWARD {0,1}
#define MOTOR_STOPPED {0,0}

typedef struct {
    int in1;
    int in2;
} motor_direction_t;

typedef struct {
    gpio_num_t in1[4];
    gpio_num_t in2[4];
    gpio_num_t pwm[4];
    gpio_num_t enca[4];
    gpio_num_t encb[4];
} drivetrain_config_t;

float drivetrain_motor_get_duty(const drivetrain_config_t *dt_conf, int motor_id);
void drivetrain_motor_spin(const drivetrain_config_t *dt_conf, int motor_id, float power);
void drivetrain_motor_set_dir(const drivetrain_config_t *dt_conf, int motor_id, motor_direction_t dir);
void drivetrain_motor_set_power(const drivetrain_config_t *dt_conf, int motor_id, float power);

/**
 * @brief Spins a motor based on % power setting
 * 
 * @param dt_conf : drivetrain config
 * @param motor_id:  0..3
 * @param power:  % power (+/-)
 */
void drivetrain_motor_spin(const drivetrain_config_t *dt_conf, int motor_id, float power) {
    
    motor_direction_t dir = {0,0};

    if(power > 0) {
        dir.in1 = 1;
    }
    else if(power < 0) {
        dir.in2 = 1;
    }
    
    drivetrain_motor_set_dir(dt_conf, motor_id, dir);
    drivetrain_motor_set_power(dt_conf, motor_id, fabs(power));

}

/**
 * @brief Sets motor spin direction via motor controller in1 and in2
 * 
 * @param dt_conf 
 * @param motor: motor id 0..3 
 * @param dir: reverse, forward, stopped
 */
void drivetrain_motor_set_dir(const drivetrain_config_t *dt_conf, int motor_id, motor_direction_t dir) {
    ESP_LOGW(TAG, "Setting direction {%d, %d} for motor %d", dir.in1, dir.in2, motor_id);

    gpio_set_level(dt_conf->in1[motor_id], dir.in1);
    gpio_set_level(dt_conf->in2[motor_id], dir.in2);
}

float drivetrain_motor_get_duty(const drivetrain_config_t *dt_conf, int motor_id) {
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
 * @param dt_conf 
 * @param motor_id: motor id 0..3
 * @param power: % of max duty cycle
 */
void drivetrain_motor_set_power(const drivetrain_config_t *dt_conf, int motor_id, float power) {
    ESP_LOGW(TAG, "Setting power %.2f%% for motor %d", power*100, motor_id);
    switch(motor_id) {
        case 0:
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, power*100);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
            //mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
            break;
        case 1:
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, power*100);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM_DUTY_MODE_0);
            //mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
            break;
        case 2:
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, power*100);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
            //mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A);
            break;
        case 3:
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B, power*100);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
            //mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B);
            break;
    }
}

/**
 * @brief Configure drivetrain mnotors and encoders
 * 
 * @param dt_conf 
 */
void drivetrain_config(const drivetrain_config_t * dt_conf) {

    uint64_t in1_in2_bit_mask = 0;
    uint64_t enca_encb_bit_mask = 0;

    for(int i=0; i < 4; i++) {
        in1_in2_bit_mask = (in1_in2_bit_mask | (1ULL << dt_conf->in1[i]) | (1ULL << dt_conf->in2[i]));
        enca_encb_bit_mask = (enca_encb_bit_mask | (1ULL << dt_conf->enca[i]) | (1ULL << dt_conf->encb[i]));
    }

    // configure in1 and in2 pins
    gpio_config_t io_conf_in1_in2 = {};
    io_conf_in1_in2.intr_type = GPIO_INTR_DISABLE;
    io_conf_in1_in2.mode = GPIO_MODE_OUTPUT;
    io_conf_in1_in2.pin_bit_mask = in1_in2_bit_mask;
    io_conf_in1_in2.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf_in1_in2.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_in1_in2);

    // configure enc1 and ecb pins
    gpio_config_t io_conf_enca_encb = {};
    io_conf_enca_encb.intr_type = GPIO_INTR_POSEDGE;
    io_conf_enca_encb.mode = GPIO_MODE_INPUT;
    io_conf_enca_encb.pin_bit_mask = enca_encb_bit_mask;
    io_conf_enca_encb.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf_enca_encb.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_enca_encb);
    

    // PWM see https://docs.espressif.com/projects/esp-idf/en/v4.4.2/esp32/api-reference/peripherals/mcpwm.html?highlight=mcpwm_unit_0

    // configure pwm pins
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, dt_conf->pwm[0]);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, dt_conf->pwm[1]);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, dt_conf->pwm[2]);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, dt_conf->pwm[3]);

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