//
// Created by Kevin Williams on 11/20/22.
//

#ifndef ESPROVER_SERVO_H
#define ESPROVER_SERVO_H

#include "driver/gpio.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"

#define SERVO_MIN_US 450
#define SERVO_MAX_US 2450

typedef enum {
    SERVO_ORIENTATION_NORMAL = 1,
    SERVO_ORIENTATION_REVERSE = -1
} servo_orientation_t;

class Servo {

private:
    gpio_num_t pwm_pin;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_generator_t generator;
    mcpwm_io_signals_t signal;
    servo_orientation_t orientation;
    uint16_t _minMicros;
    uint16_t _maxMicros;
    float _zero;
    float _dutyFactor;
    float _angleFactor;
    uint16_t _minAngle;
    uint16_t _maxAngle;


public:
    Servo(const gpio_num_t &pwmPin, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t generator,
          mcpwm_io_signals_t signal, servo_orientation_t orientation, uint16_t minUs, uint16_t maxUs, int16_t minAngle, int16_t maxAngle);

    void init();
    void deInit();
    void setDuty(float duty);
    void spin(float power);
    int16_t setAngle(int16_t angle);

    servo_orientation_t get_orientation() {
        return orientation;
    }

    virtual ~Servo();
};


#endif //ESPROVER_SERVO2_H
