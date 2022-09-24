#ifndef ROBOT_H
#define ROBOT_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "include/robot/drivetrain.h"
#include "globals.h"


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
    double turn;  // +/- peercentage
} robot_move_t;


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

double _abs_max(double * values, int size) {
    double max = 0;
    for(int i=0; i < size; i++) {
        if(fabs(values[i]) > max) {
            max = fabs(values[i]);
        }
    }
    return max;
}
void robot_move(const robot_config_t *robot, robot_move_t request) {
    ESP_LOGW(TAG, "<Move heading: %2.f power: %2.f %%>", request.heading, request.power*100);
    double max_val;
    int i;
    double adj;

    double radians = RADIANS*request.heading;

    double v1 = sin(radians + 0.25 * M_PI);// * request.power;
    double v2 = sin(radians - 0.25 * M_PI);// * request.power;

    double values[4] = {v1,v2,v1,v2};


    // level up factors so that max value of power factor based on heading is is 1
    max_val = _abs_max(values, 4);
    adj = max_val > 0 && max_val < 1 ? 1.0/max_val : 1.0;

    for(i=0; i < 4; i++) {
        values[i] = values[i] * adj * request.power + CW_ROTATION_MATRIX[i] * request.turn;;
    }

    // level down so that no values are > 1.0 for power
    max_val = _abs_max(values, 4);
    adj = max_val > 1.0 ? 1.0/max_val : 1.0;

    for(i=0; i < 4 ; i++)
        drivetrain_motor_spin(&robot->drivetrain, i, values[i]*adj);
}
#endif