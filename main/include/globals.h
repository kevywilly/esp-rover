//
// Created by Kevin Williams on 9/23/22.
//

#ifndef ESPROVER_GLOBALS_H
#define ESPROVER_GLOBALS_H

#include <esp_http_server.h>
#include "driver/gpio.h"

#ifndef RADIANS
#define RADIANS 0.0174533
#endif


//static httpd_handle_t server = NULL;

static QueueHandle_t xQueueDriveFrame;
static QueueHandle_t xQueueAutoDriveFrame;


#endif //ESPROVER_GLOBALS_H
