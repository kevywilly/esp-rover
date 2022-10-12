//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_TOF_H
#define ESPROVER_TOF_H

#include "globals.h"
#include "uart.h"


#define TOF_DATA_LEN 10
#define TOF_START_BYTE 0x10

void vTOFTask(void * args) {

    while (true) {
        // Read data from the UART
        uint8_t start[0];
        uint8_t data[TOF_DATA_LEN] = {0,0,0,0,0,0,0,0,0,0};

        uart_read_with_blocking(start, 1);

        if(start[0] == TOF_START_BYTE) {
            int len = uart_read_with_blocking(data, TOF_DATA_LEN);
            //ESP_LOGI(TAG, "Read Measurement # %d:%d:%d", data[0], data[1], data[2]);
            if(len == TOF_DATA_LEN && data[0] < 4) {
                uint8_t * p = data;
                uint8_t b1,b2;
                for(int i=0; i < 4; i++) {
                    b1 = *p++; b2 = *p++;
                    distances[i] = (b1 << 8) + b2;
                    ESP_LOGI(TAG, "<TOF id: %d distance: %d", i, distances[i]);
                }
                b1 = *p++; b2 = *p;
                recommended_heading = (b1 << 8) + b2;

                ESP_LOGI(TAG, "<TOF heading: %d> ", recommended_heading);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

}

#endif //ESPROVER_TOF_H
