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

typedef struct {
    double heading;
    double power;
}drive_request_t;

typedef struct {
    rotation_dir_t dir;
    double power;
}rotation_request_t;

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

void robot_drive(const robot_config_t *robot, drive_request_t request) {
    ESP_LOGW(TAG, "Drive heading: %.2f%% for power: %.2f%%", request.heading, request.power);
    double radians = RADIANS*request.heading;

    double v1 = sin(radians + 0.25 * M_PI);// * request.power;
    double v2 = sin(radians - 0.25 * M_PI);// * request.power;
    double dir1 = v1 < 0 ? -1 : 1;
    double dir2 = v2 < 0 ? -1 : 1;
    double diff_from_1 = fabs(v1) > fabs(v2) ? 1-fabs(v1) : 1 - fabs(v2);

    v1 += dir1*diff_from_1*request.power;
    v2 += dir2*diff_from_1*request.power;
    
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(&robot->drivetrain, i, ((i + 1) % 2 > 0) ? v1 : v2);

}

void robot_rotate(const robot_config_t *robot, rotation_request_t request) {
    ESP_LOGW(TAG, "Rotate: %d for power: %.2f%%", request.dir, request.power);
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(
            &robot->drivetrain,
            i, 
            request.power * ((request.dir == ROTATE_CW) ? CW_ROTATION_MATRIX[i] : CCW_ROTATION_MATRIX[i])
        );

}

void robot_stop(const robot_config_t *robot) {
    ESP_LOGW(TAG, "Stop Robot Motion");
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(&robot->drivetrain, i, 0);

}


#endif