//
// Created by Kevin Williams on 11/21/22.
//

#ifndef ESPROVER_APP_LOOK_H
#define ESPROVER_APP_LOOK_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "servo.hpp"

typedef struct {
    int16_t yaw;
    int16_t pitch;
} look_cmd_t;

class AppLook {
public:
    QueueHandle_t xQueue_In;

    AppLook(const QueueHandle_t queue_in);

    void run();

    void setYaw(int16_t yaw);

    void setPitch(int16_t pitch);

    void command(look_cmd_t cmd);

    int16_t getYaw() const;

    int16_t getPitch() const;

    Servo * _yawServo;
    Servo * _pitchServo;

private:
    look_cmd_t _cmd;
    int16_t _yaw = 0;
    int16_t _pitch = 0;
};


#endif //ESPROVER_APP_LOOK_H
