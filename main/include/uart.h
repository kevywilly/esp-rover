//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_UART_H
#define ESPROVER_UART_H

#include "driver/uart.h"

#define ROVER_UART_PORT_NUM 2
#define ROVER_UART_BAUD_RATE 115200
#define ROVER_RXD 16
#define ROVER_TXD 17
#define BUF_SIZE 1024

static void uart_init() {
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

}

static int uart_read_with_blocking(uint8_t *buffer, size_t len) {

    size_t buffered_len = 0;

    while (buffered_len < len) {
        uart_get_buffered_data_len(ROVER_UART_PORT_NUM, &buffered_len);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return uart_read_bytes(ROVER_UART_PORT_NUM, buffer, len, 20 / portTICK_PERIOD_MS);

}

#endif //ESPROVER_UART_H
