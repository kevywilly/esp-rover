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
#include "app_httpd.h"
#include "app_look.h"
#include "app_proximity.h"
#include "robot.h"
#include "espserial.h"
#include "proximity.h"

static const char * TAG = "ESPRover";
//static const char * mqtt = "esp_rover_mqtt_user:eR.Wqsjfib5.XLV";
static const int drive_queue_len = 10;
static const int auto_drive_queue_len = 2;
static const int neck_queue_len = 10;
static const int proximity_queue_len = 2;

static QueueHandle_t xQueueDriveFrame;
static QueueHandle_t xQueueAutoDriveFrame;
static QueueHandle_t xQueueLookFrame;
static QueueHandle_t xQueueProximityFrame;

extern "C" void app_main(void) {


    xQueueDriveFrame = xQueueCreate(drive_queue_len, sizeof(drive_command_t));
    xQueueAutoDriveFrame = xQueueCreate(auto_drive_queue_len, sizeof(bool));
    xQueueLookFrame = xQueueCreate(neck_queue_len, sizeof(look_cmd_t));
    xQueueProximityFrame = xQueueCreate(proximity_queue_len, sizeof(uint8_t));

    ProximityFrameHandle_t pFrame = new ProximityFrame(230, 275, 70, 70);
    AppProximity * appProximity = nullptr; //new AppProximity(xQueueProximityFrame, pFrame);
    AppLook * appLook = nullptr;
    AppDrive * drive = new AppDrive(xQueueDriveFrame);



    AppAutoDrive * autoDrive = new AppAutoDrive(xQueueAutoDriveFrame, xQueueDriveFrame, pFrame);

    Robot * robot = new Robot(drive, autoDrive, appLook, appProximity);

    //robot->appProximity->run();
    robot->appDrive->run();
    robot->appAutoDrive->run();
    //robot->appLook->run();

    start_webserver(robot);


}
