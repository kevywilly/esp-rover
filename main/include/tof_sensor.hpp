//
// Created by Kevin Williams on 11/22/22.
//

#ifndef ESPROVER_TOF_SENSOR_H
#define ESPROVER_TOF_SENSOR_H

#include "vl53l0x.hpp"

typedef struct {
    uint16_t angle;
    uint16_t distance;
} tof_reading_t;

class TOFSensor {
public:
    VL53L0X * device;
    uint16_t angle;
    uint16_t distance;

    TOFSensor(VL53L0X *device, uint16_t angle) : device(device), angle(angle), distance(0) {}

    uint16_t ping() {
        distance = device->readRangeSingleMillimeters();
        return distance;
    }

};
#endif //ESPROVER_TOF_SENSOR_H
