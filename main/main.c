/**
 * @file main.cpp
 * @author Kevin Williams (kevywilly@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// https://github.com/espressif/esp-idf/tree/d6d8324ad9b957f85ecb323ec0db12c4d03a93c4/examples/protocols/http_server/simple



#include "main.h"
#include "robot.h"

static const drivetrain_config_t dt_conf = {.in1 = IN1_PINS, .in2 = IN2_PINS, .pwm = PWM_PINS, .enca = ENCA_PINS, .encb = ENCB_PINS};

static robot_config_t rb_conf = {
        .dt_conf = dt_conf,
        .led_pin = LED_PIN
};

/*
static const gpio_num_t pin = 14;

static void config_pins(gpio_num_t pin) {
    
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << pin);
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    
    //disable pull-up mode
    //io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);

    gpio_set_level(pin, 1);
    

    
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 1);
    gpio_set_pull_mode(pin, GPIO_PULLDOWN_ENABLE);
    
    
}*/
static void run(void *arg)
{
   
    //config_pins(pin);
    
   
    robot_config(&rb_conf);
    
    
    /*
    robot_drive(&rb_conf, 0, 1.0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    robot_stop(&rb_conf);
    vTaskDelay(pdMS_TO_TICKS(2000));

    */

    int delay_ms = 1500;

    for(int h=0; h <= 360; h+=45) {
        printf("\n---- DRIVE WITH HEADING: %d ---\n", h);
        robot_drive(&rb_conf, h, 0.8);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    printf("\n---- ROTATE CW ---\n" );
    robot_rotate(&rb_conf, ROTATE_CW, 0.8);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    printf("\n---- ROTATE CW ---\n" );
    robot_rotate(&rb_conf, ROTATE_CCW, 0.8);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    printf("\n---- STOP ---\n" );
    robot_stop(&rb_conf);
    

    while (true) {
        //std::cout << "Hello Rover"<< i << '\n';
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    }
    
}

void app_main(void)
{
    xTaskCreate(run, "run esp rover", 2048, NULL, 10, NULL);

}