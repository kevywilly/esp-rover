//
// Created by Kevin Williams on 11/21/22.
//


#include "app_drive.h"

#define MOTOR_COUNT 4
#define RADIANS 0.0174533

static const char TAG[] = "AppDrive";

static const float ROTATION_MATRIX[4] = {-1.0, 1.0, 1.0, -1.0};

static void log_drive_command(drive_command_t *c) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "Got Drive: p=%f h=%f turn=%f", c->power, c->heading, c->turn);
#endif
}

static float abs_max(float *values, int size) {
    double max = 0;
    for (int i = 0; i < size; i++) {
        if (fabs(values[i]) > max) {
            max = fabs(values[i]);
        }
    }
    return max;
}

AppDrive::AppDrive(QueueHandle_t queueDrive) : xQueue_In(queueDrive) {

    this->cmd = new drive_command_t{0, 0, 0};

    this->motors = new Servo[4]{
            Servo((gpio_num_t) CONFIG_M0_PWM, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM0A,
                  SERVO_ORIENTATION_NORMAL, SERVO_MIN_US - 50, SERVO_MAX_US - 50, -360, 360),
            Servo((gpio_num_t) CONFIG_M1_PWM, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM0B,
                  SERVO_ORIENTATION_REVERSE, SERVO_MIN_US - 50, SERVO_MAX_US - 50, -360, 360),
            Servo((gpio_num_t) CONFIG_M2_PWM, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM1A,
                  SERVO_ORIENTATION_REVERSE, SERVO_MIN_US - 50, SERVO_MAX_US - 50, -360, 360),
            Servo((gpio_num_t) CONFIG_M3_PWM, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, MCPWM1B,
                  SERVO_ORIENTATION_NORMAL, SERVO_MIN_US - 50, SERVO_MAX_US - 50, -360, 360)
    };

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].init();
    }
}


void AppDrive::apply_command(drive_command_t cmd) {
#ifdef CONFIG_ESP_ROVER_DEBUG
    ESP_LOGI(TAG, "<Move heading: %2.f power: %2.f %% turn: %2.f %%>", cmd.heading, cmd.power * 100,
             cmd.turn * 100);
#endif

    this->cmd = new drive_command_t{cmd};

    float max_val;
    int i;
    float adj;

    float radians = RADIANS * cmd.heading;

    float v1 = sin(radians + 0.25 * M_PI);// * cmd.power;
    float v2 = sin(radians - 0.25 * M_PI);// * cmd.power;

    float values[4] = {v1, v2, v1, v2};

    // level up factors so that max value of power factor based on heading is is 1
    max_val = abs_max(values, 4);
    adj = max_val > 0 && max_val < 1 ? 1.0 / max_val : 1.0;

    for (i = 0; i < 4; i++) {
        values[i] = values[i] * adj * cmd.power + ROTATION_MATRIX[i] * cmd.turn;
    }

    // level down so that no values are > 1.0 for power
    max_val = abs_max(values, 4);
    adj = max_val > 1.0 ? 1.0 / max_val : 1.0;

    for (i = 0; i < 4; i++) {
        motors[i].setPower(values[i] * adj);
    }
}

static void task(AppDrive *self) {
    drive_command_t cmd = {0, 0, 0};

    while (1) {
        if (xQueueReceive(self->xQueue_In, (void *) &cmd, 0) == pdTRUE) {
            log_drive_command(&cmd);
            self->apply_command(cmd);
        }

        vTaskDelay(10);
    }
}

void AppDrive::run() {

    xTaskCreatePinnedToCore((TaskFunction_t) task, TAG, 5 * 1024, this, 5, NULL, 1);
}

drive_command_t AppDrive::getCmd() const {
    return *cmd;
}
