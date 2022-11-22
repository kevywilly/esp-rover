//
// Created by Kevin Williams on 11/21/22.
//

#ifndef ESPROVER_DRIVE_H
#define ESPROVER_DRIVE_H


#include "servo.hpp"
#include <esp_log.h>
#include "freertos/freertos.h"
#include "freertos/queue.h"
#include "math.h"
#include "globals.h"

typedef struct {
    double power;
    double heading;
    double turn;  // +/- percentage
} drive_command_t;


#define MOTOR_COUNT 4

class AppDrive {
public:
    Servo * motors;
    QueueHandle_t queueMotion;

    AppDrive(const QueueHandle_t queueMotion = nullptr);

    void run();
    void apply(drive_command_t cmd);

    static float abs_max(float *values, int size) {
        double max = 0;
        for (int i = 0; i < size; i++) {
            if (fabs(values[i]) > max) {
                max = fabs(values[i]);
            }
        }
        return max;
    }

};


#endif //ESPROVER_DRIVE_H
