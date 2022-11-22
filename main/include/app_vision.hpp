//
// Created by Kevin Williams on 11/21/22.
//

#ifndef ESPROVER_HEAD_H
#define ESPROVER_HEAD_H

#include "servo.hpp"

class AppVision {
public:
    AppVision();

    void run();

    void setYaw(int16_t yaw);

    void setPitch(int16_t pitch);

    int16_t getYaw() const;

    int16_t getPitch() const;

private:
    Servo * _yawServo;
    Servo * _pitchServo;
    int16_t _yaw = 0;
    int16_t _pitch = 0;
};


#endif //ESPROVER_HEAD_H
