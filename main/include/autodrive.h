//
// Created by Kevin Williams on 10/6/22.
//

#ifndef ESPROVER_AUTODRIVE_H
#define ESPROVER_AUTODRIVE_H

#include "globals.h"
#include "freertos/task.h"
#include "tof.h"
#include <math.h>

#define SONAR_FRONT_RISK_THRESHOLD 3.0
#define SONAR_SIDE_RISK_THRESHOLD 3.5
#define SONAR_RISK_THRESHOLD 2.5
#define SONAR_MAX_RISK 3
#define SONAR_MIN_RISK_DISTANCE 100.0

#define RISK_HIGH_DISTANCE 24
#define RISK_MED_DISTANCE 48
#define RISK_LOW_DISTANCE 72

#define RISK_HIGH 3
#define RISK_MED 2
#define RISK_LOW 1
#define RISK_NONE 0

#define RIGHT 0
#define FORWARD 90
#define LEFT 180
#define BACKWARD 270

#define SPIN_LEFT -1
#define SPIN_RIGHT 1

#define IS_HIGH(R) (R == RISK_HIGH)
#define IS_MED(R) (R == RISK_MED)
#define IS_LOW(R) (R == RISK_LOW)

#define BOTH_HIGH(A,B) (IS_HIGH(A) && IS_HIGH(B))
#define BOTH_MED(A,B) (IS_MED(A) && IS_MED(B))
#define BOTH_LOW(A,B) (IS_LOW(A) && IS_LOW(B))

#define EITHER_HIGH(A,B) (IS_HIGH(A) || IS_HIGH(B))
#define EITHER_MED(A,B) (IS_MED(A) || IS_MED(B))
#define EITHER_LOW(A,B) (IS_LOW(A) || IS_LOW(B))


#define MAX_OF(A, B) (A > B ? B : A)
#define MIN_OF(A, B) (B < A ? B : A)

#define RISK_POWER_ADJ(R) ((1.0 - R*0.2/SONAR_MAX_RISK))

#define RISK_LEVEL(d) (d <= RISK_HIGH_DISTANCE ? RISK_HIGH : (d <= RISK_MED_DISTANCE ? RISK_MED : (d <= RISK_LOW_DISTANCE ? RISK_LOW : RISK_NONE)))

static const int16_t front_right_rules[4] = {0, 0, 45, 90};
static const int16_t front_left_rules[4]  = {0, 0, -45, -90};
static const int16_t right_rules[4] = {0, 0, 27, 45};
static const int16_t left_rules[4] = {0, 0, -27, -45};

typedef struct {
    uint32_t front_left;
    uint32_t front_right;
    uint32_t right;
    uint32_t left;
} sonar_distances_t;

typedef sonar_distances_t sonar_risks_t;

static sonar_risks_t sonar_calc_risk_levels(sonar_distances_t d) {
    return (sonar_risks_t){
        .front_left = RISK_LEVEL(d.front_left),
        .front_right = RISK_LEVEL(d.front_right),
        .right = RISK_LEVEL(d.right),
        .left = RISK_LEVEL(d.left)
    };
}

static void autodrive_spin(DriveCommand *dc, int dir, double power, uint32_t duration) {
    dc->heading = 0;
    dc->power = 0;
    dc->turn = power*dir;
    dc->duration = duration;
}

static void autodrive_slide(DriveCommand *dc, int dir, double power, uint32_t duration) {
    dc->heading = dir;
    dc->power = power;
    dc->turn = 0.0;
    dc->duration = duration;
}


static DriveCommand autodrive(sonar_distances_t d) {
    sonar_risks_t risk = sonar_calc_risk_levels(d);

    DriveCommand dc = {FORWARD, 1, 0, 20};

    uint32_t max_risk = MAX_OF(MAX_OF(risk.front_left, risk.front_right),MAX_OF(risk.left, risk.right));

    if(IS_HIGH(risk.front_right)) {
        if(!IS_HIGH(risk.left)) {
            autodrive_spin(&dc, SPIN_LEFT, 1.0, 1000);
        } else {
            autodrive_spin(&dc, SPIN_RIGHT, 1.0, 1000);
        }
        return dc;
    } else if(IS_HIGH(risk.front_left)) {
        if(!IS_HIGH(risk.right)) {
            autodrive_spin(&dc, SPIN_LEFT, 1.0, 1000);
        } else {
            autodrive_spin(&dc, SPIN_RIGHT, 1.0, 1000);
        }
        return dc;
    } else if(IS_HIGH(risk.left)) {
        if(!IS_HIGH(risk.front_right)) {
            autodrive_slide(&dc, RIGHT, 1.0, 250);
        } else {
            autodrive_spin(&dc, SPIN_RIGHT, 1.0, 500);
        }
        return dc;
    } else if(IS_HIGH(risk.right)) {
        if(!IS_HIGH(risk.front_left)) {
            autodrive_slide(&dc, LEFT, 1.0, 250);
        } else {
            autodrive_spin(&dc, SPIN_LEFT, 1.0, 500);
        }
        return dc;
    }
    // ALL HIGH THEN SPIN

        dc.heading += front_right_rules[risk.front_right];
        dc.heading += front_left_rules[risk.front_left];
        dc.heading += left_rules[risk.left];
        dc.heading += right_rules[risk.right];
        dc.power = RISK_POWER_ADJ(max_risk);
        if(dc.heading != 90) {
            dc.duration = 100;
        }

    return dc;
}

static bool cmdEqual(DriveCommand c1, DriveCommand c2) {
    if (fabs(c1.power - c2.power) > 0.1)
        return false;
    if (fabs(c1.heading - c2.heading) > 0.1)
        return false;
    if (fabs(c1.turn - c2.turn) > 0.1)
        return false;
    return true;
}

void autodrive_task(void *args) {

    tof_init_all();

    bool auto_mode = false;
    uint8_t mode_request = 0;
    DriveCommand cmd = (DriveCommand) {.heading = 0, .power = 0, .turn = 0};

    while (true) {
        if (xQueueReceive(auto_mode_queue, (void *) &mode_request, 0) == pdTRUE) {
            auto_mode = !auto_mode;
            ESP_LOGI(TAG, "Got auto mode: %d", auto_mode);
        }

        uint16_t d[NUM_TOFS];
        tof_read_all(d);

        sonar_distances_t distances = {d[0]/10, d[1]/10, d[2]/10, d[3]/10};
        sonar_risks_t risks = sonar_calc_risk_levels(distances);

        ESP_LOGI(TAG, "<distances fl=%d,%d fr=%d,%d r=%d,%d l=%d,%d >",
                 distances.front_left, risks.front_left,
                 distances.front_right,risks.front_right,
                 distances.right, risks.right,
                 distances.left, risks.left);

        DriveCommand newCmd = autodrive(distances);

        ESP_LOGI(TAG, "<REC power = %f heading = %f turn = %f>\n",
                 newCmd.power,
                 newCmd.heading,
                 newCmd.turn
        );

        if (!cmdEqual(newCmd, cmd)) {
            cmd = newCmd;
            if (auto_mode) {
                xQueueSend(drive_queue, &cmd, 10);
                //vTaskDelay(pdMS_TO_TICKS(cmd.duration));
            }
        }

        vTaskDelay(10);
    }
}

#endif //ESPROVER_AUTODRIVE_H
