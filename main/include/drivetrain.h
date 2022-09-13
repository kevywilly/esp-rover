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

#ifndef TAG
#define TAG "Robot"
#endif

namespace rover {
    
    typedef enum {
        MOTOR_REVERSE=-1,
        MOTOR_STOP=0,
        MOTOR_FORWARD=1,
    } motor_direction_t;

    typedef struct {
        public:
            gpio_num_t in1[4];
            gpio_num_t in2[4];
            gpio_num_t pwm[4];
            gpio_num_t enca[4];
            gpio_num_t encb[4];
    } drivetrain_config_t;

    void drivetrain_motor_spin(const drivetrain_config_t *dt_conf, int motor_id, float power);
    void drivetrain_motor_set_dir(const drivetrain_config_t *dt_conf, int motor_id, motor_direction_t dir);
    void drivetrain_motor_set_power(const drivetrain_config_t *dt_conf, int motor_id, float power);
    void drivetrain_drive(const drivetrain_config_t *dt_conf, float heading, float power);
    void drivetrain_stop(const drivetrain_config_t);

    /**
     * @brief 
     * 
     * @param dt_conf 
     * @param motor_id 
     * @param power - % power (+/-)
     */
    void drivetrain_motor_spin(const drivetrain_config_t *dt_conf, int motor_id, float power) {
        motor_direction_t dir = (power < 0 ? MOTOR_REVERSE : (power > 0 ? MOTOR_FORWARD : MOTOR_STOP));
        
        drivetrain_motor_set_dir(dt_conf, motor_id, dir);
        drivetrain_motor_set_power(dt_conf, motor_id, abs(power));
    }

    void drivetrain_motor_set_dir(const drivetrain_config_t *dt_conf, int motor_id, motor_direction_t dir) {
        ESP_LOGW(TAG, "Setting direction %d for motor %d", dir, motor_id);
        if(dir == MOTOR_STOP) {
            gpio_set_level(dt_conf->in1[motor_id], 0);
            gpio_set_level(dt_conf->in2[motor_id], 0);
        } else {
            gpio_set_level(dt_conf->in1[motor_id], (dir == MOTOR_FORWARD) ? 1 : 0);
            gpio_set_level(dt_conf->in2[motor_id], (dir == MOTOR_FORWARD) ? 0 : 1);
        }
        
    }

    void drivetrain_motor_set_power(const drivetrain_config_t *dt_conf, int motor_id, float power) {
        ESP_LOGW(TAG, "Setting power %.2f%% for motor %d", power*100, motor_id);
        switch(motor_id) {
            case 0:
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, power);
                break;
            case 1:
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, power);
                break;
            case 2:
                mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, power);
                break;
            case 3:
                mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B, power);
                break;
        }
        
    }

    void drivetrain_config(const drivetrain_config_t * dt_conf) {

        uint64_t input_bit_mask = 0;
        uint64_t output_bit_mask = 0;

        for(int i=0; i < 4; i++) {
            output_bit_mask = (output_bit_mask | (1ULL << dt_conf->in1[i]) | (1ULL << dt_conf->in2[i]));
            input_bit_mask = (input_bit_mask | (1ULL << dt_conf->enca[i]) | (1ULL << dt_conf->encb[i]));
        }

        // configure output pins
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = output_bit_mask;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        // configure input pins
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        io_conf.pin_bit_mask = input_bit_mask;
        io_conf.mode = GPIO_MODE_INPUT;
        gpio_config(&io_conf);
        

        // PWM see https://docs.espressif.com/projects/esp-idf/en/v4.4.2/esp32/api-reference/peripherals/mcpwm.html?highlight=mcpwm_unit_0

        // init pwm pins
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

        /*

             
            
        
            
            uint32_t angle, count;

            while (1) {
                for (count = 0; count < 90; count++) {
                    printf("Angle of rotation: %d\n", count);
                    angle = servo_per_degree_init(count);
                    //printf("pulse width: %dus\n", angle);
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
                    vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
                }
            }
        */
    }


}
#endif