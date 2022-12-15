//
// Created by Kevin Williams on 11/21/22.
//

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "servo.h"

typedef struct {
    float yaw;
    float pitch;
} look_cmd_t;

class AppLook {
public:
    QueueHandle_t xQueue_In;

    AppLook(const QueueHandle_t queue_in);

    void run();

    void setYaw(float yaw);

    void setPitch(float pitch);

    void apply(look_cmd_t cmd);

    float getYaw() const;

    float getPitch() const;

    look_cmd_t  getCmd() const {
        return *_cmd;
    }

    Servo * _yawServo;
    Servo * _pitchServo;

private:
    look_cmd_t * _cmd = nullptr;
    float _yaw = 0;
    float _pitch = 0;
};

