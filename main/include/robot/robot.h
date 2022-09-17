#ifndef ROBOT_H
#define ROBOT_H



#include "esp_log.h"
#include "driver/gpio.h"
#include "include/robot/drivetrain.h"

typedef enum {
    ROTATE_CCW = -1,
    ROTATE_CW = 1,
} rotation_dir_t;


typedef struct {
    drivetrain_config_t drivetrain;
    gpio_num_t led_pin;
} robot_config_t;


static const robot_config_t robot = {
        .drivetrain = {
                .in1 = {CONFIG_M1_IN1, CONFIG_M2_IN1, CONFIG_M3_IN1,  CONFIG_M4_IN1},
                .in2 = {CONFIG_M1_IN2, CONFIG_M2_IN2, CONFIG_M3_IN2,  CONFIG_M4_IN2},
                .pwm = {CONFIG_M1_PWM, CONFIG_M2_PWM, CONFIG_M3_PWM,  CONFIG_M4_PWM},
                .enca = {CONFIG_M1_ENCA, CONFIG_M2_ENCA, CONFIG_M3_ENCA,  CONFIG_M4_ENCA},
                .encb = {CONFIG_M1_ENCB, CONFIG_M2_ENCB, CONFIG_M3_ENCB,  CONFIG_M4_ENCB},
                .rpm_factor = {CONFIG_POWER_ADJUST1 / 100.0, CONFIG_POWER_ADJUST2 / 100.0, CONFIG_POWER_ADJUST3 / 100.0, CONFIG_POWER_ADJUST4 / 100.0}
        }
};

static const float CW_ROTATION_MATRIX[4]  = {1.0, -1.0, -1.0, 1.0};
static const float CCW_ROTATION_MATRIX[4] = {-1.0, 1.0, 1.0, -1.0};


void robot_config(const robot_config_t *robot) {
    drivetrain_config(&robot->drivetrain);
}

void robot_drive(const robot_config_t *robot, float heading, float power) {
    ESP_LOGW(TAG, "Drive heading: %.2f%% for power: %.2f%%", heading, power);
    float radians = RADIANS * (90 - heading);

    float v1 = sin(radians + 0.25 * M_PI) * power;
    float v2 = sin(radians - 0.25 * M_PI) * power;
    
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(&robot->drivetrain, i, ((i + 1) % 2 > 0) ? v1 : v2);

}

void robot_rotate(const robot_config_t *robot, rotation_dir_t dir, float power) {
    ESP_LOGW(TAG, "Rotate: %d for power: %.2f%%", dir, power*100);
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(
            &robot->drivetrain,
            i, 
            power * ((dir == ROTATE_CW) ? CW_ROTATION_MATRIX[i] : CCW_ROTATION_MATRIX[i])
        );

}

void robot_stop(const robot_config_t *robot) {
    ESP_LOGW(TAG, "Stop Robot Motion");
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(&robot->drivetrain, i, 0);

}


#endif