//
// Created by Kevin Williams on 11/21/22.
//

#include <thread>
#include "vl53l0x.hpp"
#include "app_auto_drive.hpp"
#include "app_drive.hpp"
#include "tof_array.hpp"

static const char TAG[] = "AppAutodrive";

AppAutoDrive::AppAutoDrive(QueueHandle_t queueMotion, QueueHandle_t queueAutoDrive) : queueDrive(
        queueMotion), xQueue_In(queueAutoDrive) {
}

void AppAutoDrive::sendCmd(drive_command_t cmd) {
    xQueueSend(queueDrive, &cmd, 10);
}

void AppAutoDrive::start() {
    sendCmd({0,0,0});
    tofArray->restart();
    usleep(1000);
    active = true;
    ESP_LOGI(TAG, "Autodrive is ACTIVE");
}

void AppAutoDrive::stop() {
    sendCmd({.power = 0, .heading = 90, .turn = 0});
    active = false;
    ESP_LOGI(TAG, "Autodrive is INACTIVE");
}

bool AppAutoDrive::checkIsActive() {
    uint8_t buf = 0;
    if (xQueueReceive(xQueue_In, (void *) &buf, 0) == pdTRUE) {
        active ? stop() : start();
    }
    return active;
}

drive_command_t AppAutoDrive::getCmd() {

    if(tofArray->isClear(FRONT)) {
        if(tofArray->isClear(SIDES)) {
            // go forward
            return {.power = 0.5, .heading = 90, .turn = 0};
        } else if(tofArray->isClear(LFT)) {
            // slide left
            return {.power = 0.3, .heading = 180, .turn = 0};
        } else if(tofArray->isClear(RGT)) {
            // slide right
            return {.power = 0.3, .heading = 0, .turn = 0};
        } else {
            // go forward slowly
            return {.power = 0.3, .heading = 90, .turn = 0};
        }
    } else {
        if(tofArray->isClear(FL)) {
            // spin left
            return {.power = 0, .heading = 90, .turn = 0.3};
        } else  {
            // spin right
            return {.power = 0, .heading = 90, .turn = -0.3};
        }
    }
}
static void task(AppAutoDrive *self) {

    TOFArray * tofArray = self->tofArray;
    uint8_t * proximity = &tofArray->proximity;
    TOFSensor * tofSensors = tofArray->sensors;

    while(1) {
        self->checkIsActive();

        tofArray->readAllAvg();
        //ESP_LOGI(TAG, "\nProximity: 0b%d%d%d%d%d", bit(n,4), bit(n,3), bit(n,2), bit(n,1), bit(n,0));
        ESP_LOGI(TAG, "\n");
        ESP_LOGI(TAG, "Proximity: %d", *proximity);
        for (int i = 0; i < tofArray->size; i++) {
            ESP_LOGI(TAG, "Reading %d : %d", tofSensors[i].angle, tofSensors[i].distance);
        }

        ESP_LOGI(TAG, "FrontClear: %d", tofArray->isClear(*proximity, (FRONT)));
        ESP_LOGI(TAG, "SidesClear: %d", tofArray->isClear(*proximity, (SIDES)));

        drive_command_t cmd = self->getCmd();
        ESP_LOGI(TAG, "CMD: p: %f, h: %f, t: %f", cmd.power, cmd.heading, cmd.turn);
        if(self->isActive()) {
            self->sendCmd(cmd);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void AppAutoDrive::run() {
    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 5 * 1024, this, 5, NULL, 1);
}

