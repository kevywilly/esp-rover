//
// Created by Kevin Williams on 11/21/22.
//

#pragma once


#include "servo.h"
#include <esp_log.h>
#include "freertos/freertos.h"
#include "freertos/queue.h"
#include "math.h"

typedef struct {
    double power;
    double heading;
    double turn;  // +/- percentage
} drive_command_t;

class AppDrive {
private:
    drive_command_t * cmd = nullptr;
public:
    Servo * motors;
    QueueHandle_t xQueue_In;

    AppDrive(const QueueHandle_t queueDrive = nullptr);

    void run();

    void apply_command(drive_command_t cmd);

    drive_command_t getCmd() const;

};
