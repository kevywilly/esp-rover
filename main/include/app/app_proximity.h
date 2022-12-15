//
// Created by Kevin Williams on 12/14/22.
//

#ifndef ESPROVER_APP_PROXIMITY_H
#define ESPROVER_APP_PROXIMITY_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esplidar.h"
#include "proximity.h"

class AppProximity {
public:
    AppProximity(QueueHandle_t xQueueIn, ProximityFrameHandle_t frame);
    QueueHandle_t xQueue_In;
    ProximityFrameHandle_t pFrame;
    void run();

private:


};


#endif //ESPROVER_APP_PROXIMITY_H
