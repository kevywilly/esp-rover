//
// Created by Kevin Williams on 11/21/22.
//

#ifndef ESPROVER_PROXIMITY_HPP
#define ESPROVER_PROXIMITY_HPP

#include "stdio.h"
#include "tof_sensor.hpp"

#define P_OK 400

#define FRONT_LEFT 0
#define FRONT_MIDDLE 1
#define FRONT_RIGHT 2
#define RIGHT 3
#define LEFT 4

class Proximity {
public:

    TOFSensor * sensors;
    int numSensors;

    uint16_t frontLeft;
    uint16_t frontMiddle;
    uint16_t frontRight;
    uint16_t right;
    uint16_t left;
    uint16_t front;
    uint16_t sides;

    Proximity(TOFSensor *sensors, int numSensors) : sensors(sensors), numSensors(numSensors) {}


    void analyze() {

        frontLeft = sensors[FRONT_LEFT].distance;
        frontRight = sensors[FRONT_RIGHT].distance;
        frontMiddle = sensors[FRONT_MIDDLE].distance;
        left = sensors[LEFT].distance;
        right = sensors[RIGHT].distance;
        front = frontLeft < frontRight ? frontLeft : frontRight;
        front = front < frontMiddle ? front : frontMiddle;
        sides = left < right ? left : right;

    }

    inline bool frontClear() {
        return front > P_OK;
    }

    inline bool sidesClear() {
        return sides > P_OK;
    }

    inline bool clear() {
        return frontClear() && sidesClear();
    }

    inline float headLeftOrRight() {
        return left > right ? 180 : 0;
    }

    inline float spinLeftOrRight() {
        return frontLeft > frontRight ? 1 : -1;
    }

};
#endif //ESPROVER_PROXIMITY_HPP
