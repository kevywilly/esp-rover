#ifndef ROBOT_H
#define ROBOT_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "drivetrain.h"
#include "globals.h"
#include "ultrasonic.h"

void robot_init() {
    drivetrain_init(&Robot);
    for(int i=0; i < 4; i++) {
        //ultrasonic_init(&Robot.sonars[i]);
    }
}

double abs_max(double *values, int size) {
    double max = 0;
    for (int i = 0; i < size; i++) {
        if (fabs(values[i]) > max) {
            max = fabs(values[i]);
        }
    }
    return max;
}

void robot_move(RobotConfig *robot, DriveCommand request) {
    ESP_LOGW(TAG, "<Move heading: %2.f power: %2.f %%>", request.heading, request.power * 100);
    double max_val;
    int i;
    double adj;

    double radians = RADIANS * request.heading;

    double v1 = sin(radians + 0.25 * M_PI);// * request.power;
    double v2 = sin(radians - 0.25 * M_PI);// * request.power;

    double values[4] = {v1, v2, v1, v2};


    // level up factors so that max value of power factor based on heading is is 1
    max_val = abs_max(values, 4);
    adj = max_val > 0 && max_val < 1 ? 1.0 / max_val : 1.0;

    for (i = 0; i < 4; i++) {
        values[i] = values[i] * adj * request.power + ROTATION_MATRIX[i] * request.turn;
    }

    // level down so that no values are > 1.0 for power
    max_val = abs_max(values, 4);
    adj = max_val > 1.0 ? 1.0 / max_val : 1.0;

    for (i = 0; i < 4; i++)
        drivetrain_motor_spin(i, values[i] * adj);
}

#endif