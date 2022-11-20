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
#include "autodrive.h"
#include "webserver.h"
#include "motion.h"

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

    drive_queue = xQueueCreate(drive_queue_len, sizeof(drive_command_t));
    auto_mode_queue = xQueueCreate(auto_mode_queue_len, sizeof(bool));

    xTaskCreatePinnedToCore(motion_task, "motion task", 4096, NULL, 2, NULL, TASK_CORE);
    xTaskCreatePinnedToCore(autodrive_task, "autodrive", 4096, NULL, 1, NULL, TASK_CORE);

    start_webserver();

}
