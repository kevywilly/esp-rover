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

/*
extern "C" {
    void app_main(void);
}
*/
void init_network(httpd_handle_t * server) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
}

extern "C" void app_main(void) {

    static httpd_handle_t server = NULL;

    init_network(&server);

    xQueueDriveFrame = xQueueCreate(drive_queue_len, sizeof(drive_command_t));
    xQueueAutoDriveFrame = xQueueCreate(auto_mode_queue_len, sizeof(bool));

    AppVision * vision = new AppVision();
    AppDrive * appDrive = new AppDrive(xQueueDriveFrame);
    AppAutoDrive * proximity = new AppAutoDrive(xQueueDriveFrame, xQueueAutoDriveFrame);

    vision->run();
    appDrive->run();
    proximity->run();


    //xTaskCreatePinnedToCore(motion_task, "motion task", 4096, NULL, 2, NULL, TASK_CORE);

    start_webserver();

}
