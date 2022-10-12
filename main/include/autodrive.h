//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_AUTODRIVE_H
#define ESPROVER_AUTODRIVE_H

#include "robot.h"

#define FORWARD 90.0
#define RIGHT 0.0
#define REVERSE 270.0
#define LEFT 180.0
#define TOO_CLOSE 150

robot_move_t auto_get_best_direction() {
    robot_move_t move = {
            .heading = 90,
            .power = 0.5,
            .turn = 0.0
    };

    if(distances[0] > TOO_CLOSE)
        return move;

    if(distances[1] > distances[3]) {
        if(distances[1] > TOO_CLOSE) {
            move.turn = 1.0;
        } else {
            move.heading = 270;
            move.turn = 0.5;
        }
    } else {
        if(distances[3] > TOO_CLOSE) {
            move.turn = -1.0;
        } else {
            move.heading = 270;
            move.turn = -0.5;
        }
    }

    return move;
}
void vAutodriveTask(void * args) {
    while(true) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if(!auto_mode)
            continue;

        robot_move(&robot, auto_get_best_direction());
    }
}

#endif //ESPROVER_AUTODRIVE_H
