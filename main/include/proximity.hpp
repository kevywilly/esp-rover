//
// Created by Kevin Williams on 11/21/22.
//

#ifndef ESPROVER_PROXIMITY_HPP
#define ESPROVER_PROXIMITY_HPP

#include "stdio.h"

#define P_OK 400

#define FRONT_LEFT 0
#define FRONT_MIDDLE 1
#define FRONT_RIGHT 2
#define RIGHT 3
#define LEFT 4

class Proximity {
public:

    uint16_t * readings;
    int numReadings;

    uint16_t frontLeft;
    uint16_t frontMiddle;
    uint16_t frontRight;
    uint16_t right;
    uint16_t left;
    uint16_t front;
    uint16_t sides;

    Proximity();

    void analyze() {
        /*
        frontLeft = readings[FRONT_LEFT];
        frontRight = readings[FRONT_RIGHT];
        frontMiddle = readings[FRONT_MIDDLE];
        left = readings[FRONT_LEFT];
        right = readings[FRONT_RIGHT];
        front = frontLeft < frontRight ? frontLeft : frontRight;
        front = front < frontMiddle ? front : frontMiddle;
        sides = left < right ? left : right;
         */
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
