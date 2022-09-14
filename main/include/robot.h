#ifndef ROBOT_H
#define ROBOT_H



#include "esp_log.h"
#include "driver/gpio.h"
#include "drivetrain.h"

static const float CW_ROTATION_MATRIX[4]  = {1.0, -1.0, -1.0, 1.0};
static const float CCW_ROTATION_MATRIX[4] = {-1.0, 1.0, 1.0, -1.0};

typedef enum {
    ROTATE_CCW = -1,
    ROTATE_CW = 1,
} rotation_dir_t;


typedef struct {
    drivetrain_config_t dt_conf;
    gpio_num_t led_pin;
} robot_config_t;

void robot_config(const robot_config_t *rb_conf) {
    drivetrain_config(&rb_conf->dt_conf);
}

void robot_drive(const robot_config_t *rb_conf, float heading, float power) {
    ESP_LOGW(TAG, "Drive heading: %.2f%% for power: %.2f%%", heading, power*100);
    float radians = RADIANS * (90 - heading);

    float v1 = sin(radians + 0.25 * M_PI) * power;
    float v2 = sin(radians - 0.25 * M_PI) * power;
    
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(&rb_conf->dt_conf, i, ((i+1) % 2 > 0) ? v1 : v2);

}

void robot_rotate(const robot_config_t *rb_conf, rotation_dir_t dir, float power) {
    ESP_LOGW(TAG, "Rotate: %d for power: %.2f%%", dir, power*100);
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(
            &rb_conf->dt_conf, 
            i, 
            power * ((dir == ROTATE_CW) ? CW_ROTATION_MATRIX[i] : CCW_ROTATION_MATRIX[i])
        );

}

void robot_stop(const robot_config_t *rb_conf) {
    ESP_LOGW(TAG, "Stop Robot Motion");
    for(int i=0; i < 4 ; i++)
        drivetrain_motor_spin(&rb_conf->dt_conf, i, 0);

}


#endif