//
// Created by Kevin Williams on 11/21/22.
//

#include <esp_log.h>
#include <thread>
#include "app_look.hpp"

static const char * TAG = "AppLook";

static void printCommand(look_cmd_t cmd) {
    ESP_LOGI(TAG, "Received Look Command: yaw = %d, pitch = %d", cmd.yaw, cmd.pitch);
}
static void task(AppLook *self) {
    look_cmd_t cmd;
    int16_t pitch = -90;
    while (1) {

        if (xQueueReceive(self->xQueue_In, (void *) &cmd, 0) == pdTRUE) {
            printCommand(cmd);
            self->command(cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

AppLook::AppLook(const QueueHandle_t queue_in): xQueue_In(queue_in) {
    _yawServo = new Servo((gpio_num_t) CONFIG_NECK_YAW_PWM, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM1A, SERVO_ORIENTATION_REVERSE, SERVO_MIN_US, SERVO_MAX_US, -90, 90);
    _pitchServo = new Servo((gpio_num_t) CONFIG_NECK_PITCH_PWM, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B, MCPWM1B, SERVO_ORIENTATION_NORMAL, SERVO_MIN_US, SERVO_MAX_US, -90, 90);
    _pitchServo->init();
    _yawServo->init();
    command({0,0});
}

void AppLook::run() {

    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 2 * 1024, this, 5, NULL, 1);

}

void AppLook::setYaw(int16_t val) {
    //if(yaw < _servoYaw)
    ESP_LOGI(TAG, "setting yaw: %d", val);
    _yaw = _yawServo->setAngle(val);
}

void AppLook::setPitch(int16_t val) {
    ESP_LOGI(TAG, "setting pitch: %d", val);
    _pitch = _pitchServo->setAngle(val);
}

int16_t AppLook::getYaw() const {
    return _yaw;
}

int16_t AppLook::getPitch() const {
    return _pitch;
}

void AppLook::command(look_cmd_t cmd) {
    _cmd = cmd;
    setYaw(cmd.yaw);
    setPitch(cmd.pitch);
}
