//
// Created by Kevin Williams on 11/20/22.
//

#ifndef ESPROVER_SERVO_H
#define ESPROVER_SERVO_H

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
    uint16_t _minMicros;
    uint16_t _maxMicros;
    int16_t _minAngle;
    int16_t _maxAngle;

public:
    Servo(const gpio_num_t &pwmPin, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t generator,
          mcpwm_io_signals_t signal, servo_orientation_t orientation, uint16_t minUs, uint16_t maxUs, int16_t minAngle, int16_t maxAngle);

    void init();
    void deInit();
    void setDuty(float duty);

    inline void setMicroseconds(int16_t micros) {
        setDuty(100*micros/microseconds_per_cycle);
    }

    float setPower(float power);

    int16_t setAngle(int16_t angle);

    servo_orientation_t get_orientation() {
        return orientation;
    }

    virtual ~Servo();
};


#endif //ESPROVER_SERVO2_H
