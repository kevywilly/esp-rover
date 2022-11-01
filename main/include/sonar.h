//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_SONAR_H
#define ESPROVER_SONAR_H

#include "globals.h"
#include "uart.h"
#include "freertos/task.h"
#include <math.h>

#define SONAR_DATA_LEN 8
#define SONAR_START_BYTE 0x10

#define SONAR_DIAG_RISK_THRESHOLD 3.5
#define SONAR_RISK_THRESHOLD 3.0
#define SONAR_MAX_RISK 4.5
#define SONAR_MIN_RISK_DISTANCE 90.0

typedef struct {
    uint16_t right;
    uint16_t front;
    uint16_t left;
    uint16_t back;
}  SonarDistances;

typedef struct {
    double right;
    double front;
    double left;
    double back;
} SonarRiskLevels;


static double sonar_calc_risk_level(int16_t distance) {
    return distance >= SONAR_MIN_RISK_DISTANCE ? 0.0 : SONAR_MAX_RISK * (1 - distance / SONAR_MIN_RISK_DISTANCE);
}

static SonarRiskLevels sonar_calc_risk_levels(SonarDistances d) {
    return (SonarRiskLevels) {
            sonar_calc_risk_level(d.right),
            sonar_calc_risk_level(d.front),
            sonar_calc_risk_level(d.left),
            sonar_calc_risk_level(d.back)
    };
}


static double sonar_calc_power(double risk) {
    return (1.0 - (0.20 * risk));
}

static void sonar_recommend_spin(DriveCommand *dc, SonarRiskLevels risk) {
    dc->heading = 0;
    dc->power = 0;
    dc->turn = (risk.left < risk.right) ? -(sonar_calc_power(risk.left)) : sonar_calc_power(risk.right);
}

static void sonar_recommend_avoid(DriveCommand *dc, SonarRiskLevels risk) {
    if (risk.right >= 3 || risk.left >= 3) {
        sonar_recommend_spin(dc, risk);
    } else {
        dc->heading = (risk.left < risk.right) ? 180 : 0;
        dc->power = (risk.left < risk.right) ? sonar_calc_power(risk.left) : sonar_calc_power(risk.right);
    }
}

static void sonar_recommend_forward(DriveCommand *dc, SonarRiskLevels risk) {
    if (risk.left < SONAR_DIAG_RISK_THRESHOLD && risk.right < SONAR_DIAG_RISK_THRESHOLD)
        return;
    double risk2 = risk.left < risk.right ? risk.left : risk.right;
    double heading2 = (risk.left < risk.right ? 180 : 0);
    double power2 = sonar_calc_power(risk2);
    double w2 = (6 - risk2);
    double w1 = (6 - risk.front);
    double divisor = (12 - risk2 - risk.front);
    dc->heading = (dc->heading * w1 + heading2 * w2) / divisor;
    dc->power = (dc->power * w1 + power2 * w2) / divisor;
    dc->turn = 0;
}

static DriveCommand sonar_recommend(SonarDistances d) {
    SonarRiskLevels risk = sonar_calc_risk_levels(d);

    DriveCommand dc = {0, 0, 0};
    if (risk.front >= SONAR_RISK_THRESHOLD) {
        sonar_recommend_avoid(&dc, risk);
    } else {
        dc.heading = 90;
        dc.power = sonar_calc_power(risk.front);
        sonar_recommend_forward(&dc, risk);
    }

    return dc;
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

void sonar_task(void *args) {

    SonarDistances distances = {0, 0, 0, 0};

    bool auto_mode = false;
    uint8_t mode_request = 0;
    DriveCommand cmd = (DriveCommand){.heading = 0, .power = 0, .turn = 0};

    uint8_t start_byte = SONAR_START_BYTE;
    uint8_t data[SONAR_DATA_LEN];

    while (true) {
        if(xQueueReceive(auto_mode_queue, (void *)&mode_request, 0) == pdTRUE) {
            auto_mode = !auto_mode;
            ESP_LOGI(TAG, "Got auto mode: %d", auto_mode);
        }

        uart_flush(ROVER_UART_PORT_NUM);
        if(uart_write_bytes(ROVER_UART_PORT_NUM,&start_byte, 1) == 1) {
            if (uart_read_with_blocking(data, SONAR_DATA_LEN) == SONAR_DATA_LEN) {
                distances.right = ((data[0] << 8) + data[1]);
                distances.front = ((data[2] << 8) + data[3]);
                distances.left = ((data[4] << 8) + data[5]);
                distances.back = ((data[6] << 8) + data[7]);

                ESP_LOGI(TAG, "<TOF r=%d f=%d l=%d b=%d>", distances.right, distances.front, distances.left, distances.back);

                DriveCommand newCmd = sonar_recommend(distances);

                ESP_LOGI(TAG, "<rec move power = %f heading m= %f turn = %f>",
                         newCmd.power,
                         newCmd.heading,
                         newCmd.turn
                );

                if(!cmdEqual(newCmd, cmd)) {
                    cmd = newCmd;
                    if (auto_mode) {
                        xQueueSend(drive_queue, &cmd, 10);
                    }
                }

            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif //ESPROVER_SONAR_H
