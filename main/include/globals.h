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

static const char *TAG = "ESP Rover";

//static httpd_handle_t server = NULL;
static const double ROTATION_MATRIX[4] = {-1.0, 1.0, 1.0, -1.0};
static const int drive_queue_len = 5;
static const int auto_mode_queue_len = 2;
static QueueHandle_t drive_queue;
static QueueHandle_t auto_mode_queue;


#endif //ESPROVER_GLOBALS_H
