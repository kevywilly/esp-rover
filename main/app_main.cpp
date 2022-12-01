/**
 * @file main.cpp
 * @author Kevin Williams (kevywilly@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// https://github.com/espressif/esp-idf/tree/d6d8324ad9b957f85ecb323ec0db12c4d03a93c4/examples/protocols/http_server/simple
// https://seamonsters-2605.github.io/archive/mecanum/
// for turning [-1,1]sin(angle+1/4π) * magnitude + turn

#include <protocol_examples_common.h>
#include "app_httpd.hpp"
#include "app_look.hpp"
#include "robot.hpp"
#include "driver/gpio.h"

static const char * TAG = "ESPRover";
static const char * mqtt = "esp_rover_mqtt_user:eR.Wqsjfib5.XLV";
static const int drive_queue_len = 10;
static const int auto_drive_queue_len = 2;
static const int neck_queue_len = 10;
#define TASK_CORE 1

static QueueHandle_t xQueueDriveFrame;
static QueueHandle_t xQueueAutoDriveFrame;
static QueueHandle_t xQueueLookFrame;

extern "C" void app_main(void) {

    xQueueDriveFrame = xQueueCreate(drive_queue_len, sizeof(drive_command_t));
    xQueueAutoDriveFrame = xQueueCreate(auto_drive_queue_len, sizeof(bool));
    xQueueLookFrame = xQueueCreate(neck_queue_len, sizeof(look_cmd_t));

    AppLook * appLook = new AppLook(xQueueLookFrame);
    AppDrive * drive = new AppDrive(xQueueDriveFrame);
    AppAutoDrive * autoDrive = new AppAutoDrive(xQueueDriveFrame, xQueueAutoDriveFrame);
    Robot * robot = new Robot(drive, autoDrive, appLook);

    robot->run();

    start_webserver(robot);

}
