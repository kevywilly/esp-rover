//
// Created by Kevin Williams on 11/21/22.
//

#ifndef ESPROVER_APP_PROXIMITY_H
#define ESPROVER_APP_PROXIMITY_H

#include <esp_log.h>
#include "vl53l0x.hpp"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_drive.hpp"
#include "tof_sensor.hpp"
#include "tof_array.hpp"

class AppAutoDrive {
private:
    bool active = false;
    QueueHandle_t queueDrive;

    void stop();
    void start();

public:
    QueueHandle_t xQueue_In;
    TOFArray * tofArray;
    explicit AppAutoDrive(QueueHandle_t queueMotion = nullptr, QueueHandle_t queueAutoDrive = nullptr);
    void run();

    bool isActive() {
        return active;
    }

    void sendCmd(drive_command_t cmd);

    drive_command_t getCmd();

    bool checkIsActive();




};


#endif //ESPROVER_APP_PROXIMITY_H
