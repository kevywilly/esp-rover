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
#include "globals.h"
#include "webserver.h"
#include "app_vision.hpp"
#include "app_auto_drive.hpp"
#include "app_drive.hpp"

static const int drive_queue_len = 5;
static const int auto_mode_queue_len = 2;

#define TASK_CORE 1


extern "C" void app_main(void) {

    xQueueDriveFrame = xQueueCreate(drive_queue_len, sizeof(drive_command_t));
    xQueueAutoDriveFrame = xQueueCreate(auto_mode_queue_len, sizeof(bool));

    AppVision * vision = new AppVision();
    AppDrive * appDrive = new AppDrive(xQueueDriveFrame);
    AppAutoDrive * proximity = new AppAutoDrive(xQueueDriveFrame, xQueueAutoDriveFrame);

    vision->run();
    appDrive->run();
    proximity->run();


    auto server = start_webserver();

}
