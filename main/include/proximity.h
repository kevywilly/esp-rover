//
// Created by Kevin Williams on 11/19/22.
//

#ifndef ESPROVER_PROXIMITY_H
#define ESPROVER_PROXIMITY_H

#include "tof.h"

#define PROXIMITY_OK 400
#define P_FORWARD 90
#define P_LEFT 180
#define P_RIGHT 0

typedef struct {
    uint16_t front;
    uint16_t front_left;
    uint16_t front_right;
    uint16_t right;
    uint16_t left;
    uint16_t sides;
} proximity_t;
#endif //ESPROVER_PROXIMITY_H

static void proximity_print(proximity_t p) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "Distances (%d, %d, %d, %d)", p.front_left, p.front_right, p.right, p.left);
#endif
}

proximity_t proximity_read() {
    uint16_t d[4];
    tof_read_all(d);

    proximity_t p;
    p.front_left = d[0];
    p.front_right = d[1];
    p.right = d[2];
    p.left = d[3];
    p.front = p.front_left < p.front_right ? p.front_left : p.front_right;
    p.sides = p.left < p.right ? p.left : p.right;
#ifdef CONFIG_ESP_ROVER_DEBUG
    proximity_print(p);
#endif
    return p;
}
void proximity_apply(float power, float heading, float turn) {
    drive_command_t cmd = {power, heading, turn};
    xQueueSend(drive_queue, &cmd, 10);
}

bool proximity_front_clear(proximity_t * p) {
    return p->front > PROXIMITY_OK;
}

bool proximity_found_obstacle(proximity_t * p) {
    return (p->front < PROXIMITY_OK || p->sides < PROXIMITY_OK);
}

bool proximity_sides_clear(proximity_t * p) {
    return p->sides > PROXIMITY_OK;
}

bool proximity_left_clear(proximity_t * p) {
    return p->left > PROXIMITY_OK;
}

bool proximity_right_clear(proximity_t * p) {
    return p->right > PROXIMITY_OK;
}

esp_err_t proximity_wait_until(bool (*fn)(proximity_t *)) {
    proximity_t p = proximity_read();
    uint8_t mode_request = 0;
    while(!fn(&p)) {
        p = proximity_read();
        usleep(20*1000);

        if (xQueueReceive(auto_mode_queue, (void *) &mode_request, 0) == pdTRUE) {
            proximity_apply(0,0,0);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

esp_err_t proximity_auto() {

    proximity_t p = proximity_read();
    drive_command_t cmd = {0.0, P_FORWARD, 0};
    esp_err_t status = ESP_OK;

    if(p.front > PROXIMITY_OK) {
        if(p.sides > PROXIMITY_OK) {
            proximity_apply(0.4, P_FORWARD, 0.0);
            status = proximity_wait_until(proximity_found_obstacle);
        } else {
            proximity_apply(0.4, p.left > p.right ? P_LEFT : P_RIGHT, 0.0);
        }
    } else {
        proximity_apply(0.0, 90, p.left > p.right ? 0.5 : -0.5);
        status = proximity_wait_until(proximity_front_clear);
    }
    return status;
}