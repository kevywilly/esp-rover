//
// Created by Kevin Williams on 11/21/22.
//

#include "app_vision.hpp"

AppVision::AppVision() {
    _yawServo = new Servo((gpio_num_t) 16, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM0A, SERVO_ORIENTATION_NORMAL, 500, 2500, -90, 90);
    _pitchServo = new Servo((gpio_num_t) 17, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM0A, SERVO_ORIENTATION_NORMAL, 500, 2500, -90, 90);
}

void AppVision::run() {
    _pitchServo->init();
    _yawServo->init();

    setYaw(0);
    setPitch(0);
}

void AppVision::setYaw(int16_t yaw) {
    //if(yaw < _servoYaw)
    _yaw = _yawServo->setAngle(yaw);
}

void AppVision::setPitch(int16_t pitch) {
    _pitch = _pitchServo->setAngle(pitch);
}

int16_t AppVision::getYaw() const {
    return _yaw;
}

int16_t AppVision::getPitch() const {
    return _pitch;
}
