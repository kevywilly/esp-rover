//
// Created by Kevin Williams on 10/15/22.
//

#ifndef ESPROVER_MOTION_H
#define ESPROVER_MOTION_H

#include <math.h>
#include "globals.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "servo.h"

typedef struct {
    double heading;
    double power;
    double turn;  // +/- percentage
} drive_command_t;

static void log_drive_command(drive_command_t *c) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "Got Drive: p=%f h=%f turn=%f", c->power, c->heading, c->turn);
#endif
}

#define MOTION_NUM_SERVOS 4

static servo_t servos[MOTION_NUM_SERVOS] = {
        {.pwm_pin = CONFIG_M0_PWM, .unit = MCPWM_UNIT_0, .timer = MCPWM_TIMER_0, .generator = MCPWM_GEN_A, .signal=MCPWM0A, .orientation = SERVO_ORIENTATION_NORMAL},
        {.pwm_pin = CONFIG_M1_PWM, .unit = MCPWM_UNIT_0, .timer = MCPWM_TIMER_0, .generator = MCPWM_GEN_B, .signal=MCPWM0B, .orientation = SERVO_ORIENTATION_REVERSE},
        {.pwm_pin = CONFIG_M2_PWM, .unit = MCPWM_UNIT_1, .timer = MCPWM_TIMER_1, .generator = MCPWM_GEN_A, .signal=MCPWM1A, .orientation = SERVO_ORIENTATION_REVERSE},
        {.pwm_pin = CONFIG_M3_PWM, .unit = MCPWM_UNIT_1, .timer = MCPWM_TIMER_1, .generator = MCPWM_GEN_B, .signal=MCPWM1B, .orientation = SERVO_ORIENTATION_NORMAL}
};

double abs_max(float *values, int size) {
    double max = 0;
    for (int i = 0; i < size; i++) {
        if (fabs(values[i]) > max) {
            max = fabs(values[i]);
        }
    }
    return max;
}

static void motion_init() {
    for (int i = 0; i < MOTION_NUM_SERVOS; i++) {
        servo_init(&servos[i]);
    }
}

void motion_apply2(drive_command_t request) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "<Move heading: %2.f power: %2.f %% turn: %2.f %%>", request.heading, request.power * 100,
             request.turn * 100);
#endif

    float radians = RADIANS * request.heading;

    float v1 = sin(radians + 0.25 * M_PI);// * request.power;
    float v2 = sin(radians - 0.25 * M_PI);// * request.power;


    float values[4] = {v1, v2, v1, v2};


    if(request.turn != 0) {

    }

    for (int i = 0; i < 4; i++) {
        servo_spin(&servos[i], values[i]*request.power);
    }
}
void motion_apply(drive_command_t request) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "<Move heading: %2.f power: %2.f %% turn: %2.f %%>", request.heading, request.power * 100,
             request.turn * 100);
#endif
    float max_val;
    int i;
    float adj;

    float radians = RADIANS * request.heading;

    float v1 = sin(radians + 0.25 * M_PI);// * request.power;
    float v2 = sin(radians - 0.25 * M_PI);// * request.power;

    float values[4] = {v1, v2, v1, v2};


    // level up factors so that max value of power factor based on heading is is 1
    max_val = abs_max(values, 4);
    adj = max_val > 0 && max_val < 1 ? 1.0 / max_val : 1.0;

    for (i = 0; i < 4; i++) {
        values[i] = values[i] * adj * request.power + ROTATION_MATRIX[i] * request.turn;
    }

    // level down so that no values are > 1.0 for power
    max_val = abs_max(values, 4);
    adj = max_val > 1.0 ? 1.0 / max_val : 1.0;

    for (i = 0; i < 4; i++) {
        servo_spin(&servos[i], values[i] * adj);
    }
}

static void motion_task(void *args) {

    motion_init();

    drive_command_t driveCmd = {0, 0, 0};

    while (1) {
        if (xQueueReceive(drive_queue, (void *) &driveCmd, 0) == pdTRUE) {
            log_drive_command(&driveCmd);
            motion_apply(driveCmd);
        }

        vTaskDelay(10);
    }

    vTaskDelete(NULL);
}

#endif //ESPROVER_MOTION_H
