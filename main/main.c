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



#include "robot/robot.h"
#include "http/webserver.h"
#include "globals.h"

#define foo CONFIG_POWER_PARAM_NAME


void vRobotTask(void * args) {
    robot_move(&robot, (robot_move_t){.power = 1.0, .heading = 90.0, .turn = 0.0});
    vTaskDelay(pdMS_TO_TICKS(2000));
    robot_move(&robot, (robot_move_t){.power = 1.0, .heading = 90.0, .turn = 0.2});
    vTaskDelay(pdMS_TO_TICKS(2000));
    robot_move(&robot, (robot_move_t){.power = 0, .heading = 0, .turn = 0.2});
    vTaskDelay(portMAX_DELAY);

}
void app_main(void)
{
    static httpd_handle_t server = NULL;

    robot_config(&robot);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

    //xTaskCreate(vRobotTask, "run esp rover", 4096, NULL, 10, NULL);
    server = start_webserver();
}