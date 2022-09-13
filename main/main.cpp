#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "main.h"
#include "robot.h"
#include "drivetrain.h"

using namespace rover;

#define IN1_PINS   {GPIO_NUM_14, GPIO_NUM_16, GPIO_NUM_21,  GPIO_NUM_0}
#define IN2_PINS   {GPIO_NUM_12, GPIO_NUM_25, GPIO_NUM_22, GPIO_NUM_15}
#define PWM_PINS   {GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_23,  GPIO_NUM_4}
#define ENCA_PINS  {GPIO_NUM_32, GPIO_NUM_34, GPIO_NUM_18,  GPIO_NUM_5}
#define ENCB_PINS  {GPIO_NUM_13, GPIO_NUM_35, GPIO_NUM_19, GPIO_NUM_17}

#define LED_PIN GPIO_NUM_2

void ESPRover::run(int i)
{
    
    robot_config_t rb_conf = {
        .dt_conf = {.in1 = IN1_PINS, .in2 = IN2_PINS, .pwm = PWM_PINS, .enca = ENCA_PINS, .encb = ENCB_PINS},
        .led = LED_PIN
    };
    
    robot_config(&rb_conf);
    
    for(int h=0; h <= 360; h+=45) {
        std::cout << "\n" << "-----------------------------" << "\n";
        robot_drive(&rb_conf, h, 0.5);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    std::cout << "\n" << "-----------------------------" << "\n";
    robot_rotate(&rb_conf, ROTATE_CW, 0.5);
    vTaskDelay(pdMS_TO_TICKS(2000));
    std::cout << "\n" << "-----------------------------" << "\n";
    robot_rotate(&rb_conf, ROTATE_CCW, 0.5);
    vTaskDelay(pdMS_TO_TICKS(2000));
    std::cout << "\n" << "-----------------------------" << "\n";
    robot_stop(&rb_conf);

    //xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control

    while (true) {
        //std::cout << "Hello Rover"<< i << '\n';
        vTaskDelay(pdMS_TO_TICKS(1000));
        i++;
    }
}

extern "C" void app_main(void)
{
    
    ESPRover App;
    int i = 0;

    //while (true)
    //{
        App.run(i);
    //    i++;
    //}    
}