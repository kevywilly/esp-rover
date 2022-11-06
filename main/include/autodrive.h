//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_AUTODRIVE_H
#define ESPROVER_AUTODRIVE_H

#include "globals.h"
#include "freertos/task.h"
#include "tof.h"
#include <math.h>


#define MAX_OF(A, B) (A > B ? B : A)
#define MIN_OF(A, B) (B < A ? B : A)


#define MEDIAL_BLOCKED 500
#define MEDIAL_CLEAR 1000

#define LATERAL_BLOCKED  200
#define LATERAL_CLEAR 300

typedef struct {
    uint16_t front;
    uint16_t front_left;
    uint16_t front_right;
    uint16_t left;
    uint16_t right;
    uint16_t lateral;

    bool front_clear;
    bool front_blocked;
    bool front_left_clear;
    bool front_left_blocked;
    bool front_right_clear;
    bool front_right_blocked;
    bool left_clear;
    bool left_blocked;
    bool right_clear;
    bool right_blocked;
    bool lateral_clear;
    bool lateral_blocked;
    DriveCommand cmd;
    bool avoiding;
    bool force;
    char * result;
    
    
} obstacle_t;

#define H_FORWARD 90
#define H_RIGHT 0
#define H_LEFT 180
#define H_BACKWARD 270
#define T_RIGHT -1
#define T_NONE 0
#define T_LEFT 1


static void get_obstacles(obstacle_t * o) {
    uint16_t d[NUM_TOFS];
    tof_read_all(d);

    o->front_left = d[0];
    o->front_right = d[1];
    o->right = d[2];
    o->left = d[3];

    o->front = MIN_OF(o->front_left, o->front_right);
    o->lateral = MIN_OF(o->left, o->right);

    o->front_clear = o->front > MEDIAL_CLEAR;
    o->front_left_clear = o->front_left > MEDIAL_CLEAR;
    o->front_right_clear = o->front_right > MEDIAL_CLEAR;
    o->left_clear = o->left > LATERAL_CLEAR;
    o->right_clear = o->right > LATERAL_CLEAR;

    o->front_blocked = o->front <= MEDIAL_BLOCKED;
    o->front_left_blocked = o->front_left <= MEDIAL_BLOCKED;
    o->front_right_blocked = o->front_right <= MEDIAL_BLOCKED;
    o->left_blocked = o->left <= LATERAL_BLOCKED;
    o->right_blocked = o->right <= LATERAL_BLOCKED;
    o->lateral_blocked = o->lateral <= LATERAL_BLOCKED;
    o->lateral_clear = o->lateral > LATERAL_CLEAR;

    o->cmd.turn = T_NONE;
    o->cmd.power = 0.0;
    o->cmd.heading = H_FORWARD;

    o->force = false;

    o->avoiding = false;

    if(o->front_blocked) {
        double turn_dir;
        if(o->front_right_blocked){
            if(o->right_blocked || (o->left > o->right)) {
                turn_dir = T_LEFT;
            } else {
                turn_dir = T_RIGHT;
            }
        } else {
            if(o->left_blocked || (o->right > o->left)) {
                turn_dir = T_RIGHT;
            } else {
                turn_dir = T_LEFT;
            }
        }
        o->cmd = (DriveCommand) {H_BACKWARD, 0.0, turn_dir};
        o->avoiding = true;
    }

    if(o->avoiding) {
        return;
    }

    if(o->right_blocked || o->left_blocked) {
        if(!o->left_blocked) {
            o->cmd = (DriveCommand) {(H_LEFT + H_FORWARD)/2, 0.8, 0.0};
            return;
        } else if(!o->right_blocked) {
            o->cmd = (DriveCommand) {(H_RIGHT + H_FORWARD)/2, 0.8, 0.0};
            return;
        } else {
            o->cmd = (DriveCommand) {H_FORWARD, 0.8, 0.0};
        }
    }

    o->result = "ALL CLEAR";
    o->cmd = (DriveCommand) {H_FORWARD, 1.0, T_NONE};

    
}

static void printObstacles(obstacle_t o) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "Distances (%d, %d, %d, %d)", o.front_left, o.front_right, o.right, o.left);
    ESP_LOGI(TAG, "AvoidMode? %s", o.avoiding ? "Yes" : "No");
    ESP_LOGI(TAG, "Command: (%f, %f, %f)", o.cmd.heading, o.cmd.power, o.cmd.turn);
#endif
}

void autodrive_stop() {
    DriveCommand cmd = (DriveCommand) {.heading=90, .power = 0, .turn = 0.0};
    xQueueSend(drive_queue, &cmd, 10);

}
void autodrive_task(void *args) {

    tof_init_all();
    uint8_t mode_request = 0;
    bool auto_mode = false;
    obstacle_t o;

    while(1) {
        if (xQueueReceive(auto_mode_queue, (void *) &mode_request, 0) == pdTRUE) {
            auto_mode = !auto_mode;
            autodrive_stop();
        }
        get_obstacles(&o);
        printObstacles(o);

        if(auto_mode) {
            xQueueSend(drive_queue, &o.cmd, 10);
        }

        vTaskDelay(20);
    }

    vTaskDelete(NULL);
}

#endif //ESPROVER_AUTODRIVE_H
