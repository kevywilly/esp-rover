//
// Created by Kevin Williams on 11/21/22.
//

#include <esp_log.h>
#include <thread>
#include "app_look.h"

static const char * TAG = "AppLook";

AppLook::AppLook(QueueHandle_t queue_in): xQueue_In(queue_in) {
    _yawServo = new Servo((gpio_num_t) CONFIG_NECK_YAW_PWM, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM0A, SERVO_ORIENTATION_NORMAL, SERVO_MIN_US, SERVO_MAX_US, -90, 90);
    _pitchServo = new Servo((gpio_num_t) CONFIG_NECK_PITCH_PWM, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM0B, SERVO_ORIENTATION_REVERSE, SERVO_MIN_US, SERVO_MAX_US, -90, 90);
    _pitchServo->init();
    _yawServo->init();
    apply(look_cmd_t {5,-5});
}

void AppLook::setYaw(float val) {
    //if(yaw < _servoYaw)
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "setting yaw: %f", val);
#endif
    _yaw = _yawServo->setAngle(val);
}

void AppLook::setPitch(float val) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "setting pitch: %f", val);
#endif
    _pitch = _pitchServo->setAngle(val);
}

float AppLook::getYaw() const {
    return _yaw;
}

float AppLook::getPitch() const {
    return _pitch;
}

void AppLook::apply(look_cmd_t cmd) {
    _cmd = new look_cmd_t(cmd);
    setYaw(cmd.yaw);
    setPitch(cmd.pitch);
}

static void printCommand(look_cmd_t cmd) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "Received Look Command: yaw = %f, pitch = %f", cmd.yaw, cmd.pitch);
#endif
}

static void task(AppLook *self) {
    look_cmd_t cmd = {.yaw = 5.0, .pitch = -10};
    self->apply(cmd);
    while (true) {

        if (xQueueReceive(self->xQueue_In, (void *) &cmd, 0) == pdTRUE) {
            printCommand(cmd);
            self->apply(cmd);
        }

        self->apply({.yaw = 5, .pitch = -20});
        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

void AppLook::run() {

    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 2 * 1024, this, 5, NULL, 1);

}