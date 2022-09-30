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
#include "driver/uart.h"

#define ROVER_UART_PORT_NUM 2
#define ROVER_UART_BAUD_RATE 115200
#define ROVER_RXD 16
#define ROVER_TXD 17
#define BUF_SIZE 1024
#define TOF_BYTE_COUNT 8

void xTofTask(void * args) {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
            .baud_rate = ROVER_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            //.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ROVER_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ROVER_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ROVER_UART_PORT_NUM, ROVER_TXD, ROVER_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    //uart_disable_tx_intr(ECHO_UART_PORT_NUM);
    // Configure a temporary buffer for the incoming data



    while (1) {
        // Read data from the UART
        uint8_t data[TOF_BYTE_COUNT];

        size_t sizet = 0;

        while(sizet < TOF_BYTE_COUNT) {
            uart_get_buffered_data_len(ROVER_UART_PORT_NUM, &sizet);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        int len = uart_read_bytes(ROVER_UART_PORT_NUM, data, TOF_BYTE_COUNT, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        //uart_write_bytes(ROVER_UART_PORT_NUM, (const char *) data, len);

        if (len > 0) {
            ESP_LOGI(TAG, "Got %d bytes", len);
            uint16_t distances[4];
            distances[0] = (data[0] << 8) + data[1];
            distances[1] = (data[2] << 8) + data[3];
            distances[2] = (data[4] << 8) + data[5];
            distances[3] = (data[6] << 8) + data[7];

            ESP_LOGI(TAG, "Measurement: %d, %d, %d, %d", distances[0], distances[1], distances[2], distances[3]);
        }

    }

}
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

    xTaskCreate(xTofTask, "read tof", 2048, NULL, 2, NULL);
    //xTaskCreate(vRobotTask, "run esp rover", 4096, NULL, 10, NULL);
    server = start_webserver();
}