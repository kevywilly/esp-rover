//
// Created by Kevin Williams on 11/21/22.
//
#include "proximity.hpp"

Proximity::Proximity(){
    numReadings = 5;
    readings = new uint16_t[numReadings]();
    for(int i=0; i < numReadings; i++) {
        readings[i] = 8191;
    }
    analyze();
}

