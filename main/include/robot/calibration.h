//
// Created by Kevin Williams on 9/17/22.
//

#ifndef ESPROVER_CALIBRATION_H
#define ESPROVER_CALIBRATION_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define CALIBRATION_PERIOD 1000

static int encoder1 = 0;
static int encoder2 = 0;
static int encoder3 = 0;
static int encoder4 = 0;

static int rpms[4] = {0,0,0,0};

static float min_rpm = 0;

static float rpm_factors[4] = {1.0, 1.0, 1.0, 1.0};

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    int motor_num = (int) arg;

    switch(motor_num) {
        case 0:
            encoder1++;
            break;
        case 1:
            encoder2++;
            break;
        case 2:
            encoder3++;
            break;
        case 3:
            encoder4++;
            break;

    }
}

static void clear_encoders() {
    encoder1 = 0;
    encoder2 = 0;
    encoder3 = 0;
    encoder4 = 0;
}
static void calc_rpm() {

    rpms[0] = encoder1 * 60 * 1000.0 / CALIBRATION_PERIOD;
    rpms[1] = encoder2 * 60 * 1000.0 / CALIBRATION_PERIOD;
    rpms[2] = encoder3 * 60 * 1000.0 / CALIBRATION_PERIOD;
    rpms[3] = encoder4 * 60 * 1000.0 / CALIBRATION_PERIOD;

    clear_encoders();

    min_rpm = rpms[0];
    int i;
    for(i=1; i<4; i++) {
        if(rpms[i] < min_rpm) {
            min_rpm = rpms[i];
        }
    }
    for(i=0; i<4; i++) {
        if(min_rpm > 0) {
            rpm_factors[i] = min_rpm/rpms[i];
        } else {
            rpm_factors[i] = 1.0;
        }
    }

    printf("min_rpm: %f\n", min_rpm);
    for(int i=0; i< 4; i++) {
        printf("rpm-%d: %d, adj_factor: %f\n", i, rpms[i], rpm_factors[i]);
    }

}

static char * calibrate_motors(const robot_config_t *robot) {
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    int count = 0;
    for(count=0; count < 4; count++) {
        gpio_isr_handler_add(robot->drivetrain.enca[count], gpio_isr_handler, (void*) count);
    }

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    for(count = 0; count < 4; count++) {
        drivetrain_motor_spin(&robot->drivetrain, count, 1.0);
    }

    for(count=0;count<5;count++){
        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_PERIOD));
        calc_rpm();
    }
    robot_move(robot, (robot_move_t){0,0,0});

    for(count=0; count < 4; count++) {
        gpio_isr_handler_remove(robot->drivetrain.enca[count]);
    }

    return "Calibration complete.";

}
#endif //ESPROVER_CALIBRATION_H
