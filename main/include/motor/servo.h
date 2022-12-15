//
// Created by Kevin Williams on 11/20/22.
//

#pragma once

#include "driver/gpio.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

typedef enum {
    SERVO_ORIENTATION_NORMAL = 1,
    SERVO_ORIENTATION_REVERSE = -1
} servo_orientation_t;

static const uint32_t frequency = 50;
static const uint32_t microseconds_per_cycle = 1000000 / frequency;

class Servo {

private:
    gpio_num_t pwm_pin;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_generator_t generator;
    mcpwm_io_signals_t signal;
    servo_orientation_t orientation;
    uint32_t _minMicros;
    uint32_t _maxMicros;
    float _minAngle;
    float _maxAngle;
    float zero;
    float range;

public:
    Servo(const gpio_num_t &pwmPin, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t generator,
          mcpwm_io_signals_t signal, servo_orientation_t orientation, uint32_t minUs, uint32_t maxUs, float minAngle, float maxAngle);

    void init();
    void deInit();
    void setDuty(float duty);

    void setMicroseconds(uint32_t micros);

    float setPower(float power);

    float setAngle(float angle);

    servo_orientation_t get_orientation() {
        return orientation;
    }

    virtual ~Servo();
};

