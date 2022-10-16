//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_TOF_H
#define ESPROVER_TOF_H

#include "globals.h"
#include "uart.h"
#include "freertos/task.h"


#define TOF_DATA_LEN 8
#define TOF_START_BYTE 0x10
#define TOF_THRESHOLD1 300.0
#define TOF_THRESHOLD2 1000.0

static DriveCommand tof_recommend(uint16_t * distances) {

    DriveCommand cmd;

    uint16_t r = distances[0];
    uint16_t f = distances[1];
    uint16_t l = distances[2];
    uint16_t b = distances[3];

    float right = 0;
    float forward = 0;
    float left = 0;
    float back = 0;
    float spin = 0;
    float heading = 0;
    float power = 0;
    float g0 = 0;

    // forward motion
    if(f > TOF_THRESHOLD1) {
        g0++;
        forward = f / TOF_THRESHOLD2;
    }

    // left / right motion
    if((r > TOF_THRESHOLD1) && (r > l)) {
        right = r/TOF_THRESHOLD2;
        left = 0;
        g0++;
    } else if(l > TOF_THRESHOLD1 && l > r) {
        left = l/TOF_THRESHOLD2;
        right = 0;
        g0++;
    }

    if(forward == 0 && right == 0 && left == 0) {
        if(r > l) {
            spin = 0.5;
        } else {
            spin = -0.5;
        }
    } else {
        float sum = forward + right + left;
        heading = (forward*90 + right*0 + left*180)/sum;
        power = g0 > 0 ? sum/g0 : 0;
        power = power > 1.0 ? 1.0 : power;
    }

    return (DriveCommand){.power = power, .heading = heading, .turn = spin};


}

static bool cmdEqual(DriveCommand c1, DriveCommand c2) {
    if(fabs(c1.power - c2.power) > 0.1)
        return false;
    if(fabs(c1.heading - c2.heading) > 0.1)
        return false;
    if(fabs(c1.turn - c2.turn) > 0.1)
        return false;
    return true;
}

void vTOFTask(void *args) {

    uint16_t distances[4] = {0,0,0,0};
    bool auto_mode = false;
    uint8_t mode_request = 0;
    DriveCommand cmd = (DriveCommand){.heading = 0, .power = 0, .turn = 0};

    while (true) {
        if(xQueueReceive(auto_mode_queue, (void *)&mode_request, 0) == pdTRUE) {
            auto_mode = !auto_mode;
            ESP_LOGI(TAG, "Got auto mode: %d", auto_mode);
        }

            // Read data from the UART
        uint8_t start[1] = {0};
        uint8_t data[TOF_DATA_LEN];
        memset(data, 0, TOF_DATA_LEN);

        uart_read_with_blocking(start, 1);

        if (start[0] == TOF_START_BYTE) {
            if (uart_read_with_blocking(data, TOF_DATA_LEN) == TOF_DATA_LEN) {
                uint8_t *p = data;
                uint8_t b1, b2;
                for (int i = 0; i < 4; i++) {
                    b1 = *p++;
                    b2 = *p++;
                    distances[i] = (b1 << 8) + b2;
                    //ESP_LOGI(TAG, "<TOF id: %d distance: %d>", i, distances[i]);
                }
            }
        }

        DriveCommand newCmd = tof_recommend(distances);

        if(!cmdEqual(newCmd, cmd)) {
            cmd = newCmd;
            ESP_LOGI(TAG, "<rec move power = %f heading m= %f turn = %f>",
                     cmd.power,
                     cmd.heading,
                     cmd.turn
            );

            if (auto_mode) {
                xQueueSend(drive_queue, &cmd, 10);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

#endif //ESPROVER_TOF_H
