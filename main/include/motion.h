//
// Created by Kevin Williams on 10/15/22.
//

#ifndef ESPROVER_MOTION_H
#define ESPROVER_MOTION_H
#include "globals.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "robot.h"

static void logDriveCommand(DriveCommand * c) {
    ESP_LOGI(TAG, "Got Drive: p=%f h=%f turn=%f", c->power, c->heading, c->turn);
}

static void motionTask(void * args) {

    DriveCommand driveCmd = {0,0,0};

    while(1) {
        if (xQueueReceive(drive_queue, (void *)&driveCmd, 0) == pdTRUE) {
            logDriveCommand(&driveCmd);
        }
        robot_move(&Robot, driveCmd);
        vTaskDelay(10);
    }


}
#endif //ESPROVER_MOTION_H
