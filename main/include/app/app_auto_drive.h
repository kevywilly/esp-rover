//
// Created by Kevin Williams on 11/21/22.
//

#pragma once

#include <esp_log.h>
#include "vl53l0x.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_drive.h"
#include "tofsensor.h"
#include "tofarray.h"
#include "proximity.h"

class AppAutoDrive {


public:

    AppAutoDrive(QueueHandle_t xQueueIn, QueueHandle_t xQueueDrive, ProximityFrame *pFrame);

    QueueHandle_t xQueue_In;
    QueueHandle_t xQueue_Drive;
    ProximityFrame * pFrame;
    TOFArray * tofArray;
    void run();

    bool isActive() {
        return active;
    }

    void sendCmd(drive_command_t cmd);

    drive_command_t getCmd();

    bool checkIsActive();

private:
    bool active = false;


    void stop();
    void start();


};

