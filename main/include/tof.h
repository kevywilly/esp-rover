//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_TOF_H
#define ESPROVER_TOF_H

#include "globals.h"
#include "uart.h"


#define TOF_DATA_LEN 8
#define TOF_START_BYTE 0x10
#define TOF_THRESHOLD1 300.0
#define TOF_THRESHOLD2 1000.0

static robot_move_t tof_recommend(uint16_t * distances) {
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

    if(f > TOF_THRESHOLD1) {
        g0++;
        forward = f / TOF_THRESHOLD2;
    }

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

    /*
    printf("f %d : r %d : l %d : b %d\n", f, r, l, b);
    printf("forward %f : right %f : left %f : back %f : turn %f\n", forward, right, left, back, spin);
    printf("power %f heading %f turn %f\n\n", power, heading, spin);
    */

    return (robot_move_t){.power = power, .heading = heading, .turn = spin};

}
void vTOFTask(void *args) {

    while (true) {
        // Read data from the UART
        uint8_t start[0];
        uint8_t data[TOF_DATA_LEN] = {0, 0, 0, 0, 0, 0, 0, 0};

        uart_read_with_blocking(start, 1);

        if (start[0] == TOF_START_BYTE) {
            int len = uart_read_with_blocking(data, TOF_DATA_LEN);
            if (len == TOF_DATA_LEN && data[0] < 4) {
                uint8_t *p = data;
                uint8_t b1, b2;
                for (int i = 0; i < 4; i++) {
                    b1 = *p++;
                    b2 = *p++;
                    tof_distances[i] = (b1 << 8) + b2;
                    //ESP_LOGI(TAG, "<TOF id: %d distance: %d>", i, tof_distances[i]);
                }
            }
        }

       tof_recommended_move = tof_recommend(tof_distances);
       ESP_LOGI(TAG, "<rec move power = %f heading m= %f turn = %f>",
                 tof_recommended_move.power,
                 tof_recommended_move.heading,
                 tof_recommended_move.turn
                 );

        
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

#endif //ESPROVER_TOF_H
