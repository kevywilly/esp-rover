//
// Created by Kevin Williams on 11/20/22.
//

#include "servo.hpp"


Servo::Servo(const gpio_num_t &pwmPin, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t generator,
             mcpwm_io_signals_t signal, servo_orientation_t orientation, uint16_t minUs, uint16_t maxUs, int16_t minAngle, int16_t maxAngle) : pwm_pin(
        pwmPin), unit(unit), timer(timer), generator(generator), signal(signal), orientation(orientation),
                                                                                                                                               _minMicros(minUs),
                                                                                                                                               _maxMicros(maxUs),
                                                                                                                                               _minAngle(minAngle),
                                                                                                                                               _maxAngle(maxAngle){
}

Servo::~Servo() {
    deInit();
}

void Servo::init() {
    mcpwm_gpio_init(unit, signal, pwm_pin);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(unit, timer, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

void Servo::deInit() {
    setDuty(0);
}

// Note - duty is in the form 50.0 for 50%
void Servo::setDuty(float duty) {
    mcpwm_set_duty(unit,timer,generator,duty);
    mcpwm_set_duty_type(unit,timer,generator,MCPWM_DUTY_MODE_0);
}

int16_t Servo::setAngle(int16_t angle) {
    angle = (angle < _minAngle ? _minAngle : (angle > _maxAngle ? _maxAngle : angle)) * orientation;
    setMicroseconds(_minMicros + ((angle - _minAngle)*(_maxMicros-_minMicros))/(_maxAngle - _minAngle));
    return angle;
}

float Servo::setPower(float power) {
    if(power == 0) {
        setDuty(0);
        return power;
    }
    power = (power < -1 ? -1 : power > 1 ? 1 : power) * orientation;
    setMicroseconds(power * 0.5 * (_maxMicros - _minMicros) + (_minMicros+_maxMicros/2));
    return power;
}

/*
 * power = (power < -1 ? -1 : power > 1 ? 1 : power) * orientation;
    float duty = power == 0 ? 0.0 : (_zero + power * _dutyFactor) / 200.0;  // (100/20000)
    setDuty(duty);



    _zero = 0.5*(_maxMicros + _minMicros);
    _dutyFactor = (0.5) * (_maxMicros - _minMicros);
    _angleFactor = (_maxMicros - _minMicros) / (_maxAngle - _minAngle);
 */