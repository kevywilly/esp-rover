
#ifndef MAIN_H
#define MAIN_H



#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "robot.h"

#define CALIBRATION_PERIOD 1000

// Define pins for motor control
#define IN1_PINS   {GPIO_NUM_14, GPIO_NUM_16, GPIO_NUM_21,  GPIO_NUM_0}
#define IN2_PINS   {GPIO_NUM_12, GPIO_NUM_25, GPIO_NUM_22, GPIO_NUM_15}
#define PWM_PINS   {GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_23,  GPIO_NUM_4}
#define ENCA_PINS  {GPIO_NUM_32, GPIO_NUM_34, GPIO_NUM_18,  GPIO_NUM_5}
#define ENCB_PINS  {GPIO_NUM_13, GPIO_NUM_35, GPIO_NUM_19, GPIO_NUM_17}
#define RPM_FACTORS {1.0, 0.87, 0.84, 0.81}
//#define RPM_FACTORS {1.0, 1, 1, 1}
// Define led pin
#define LED_PIN GPIO_NUM_2


const char * CMD_DRIVE = "drive";
const char * CMD_STOP = "stop";
const char * CMD_ROTATE = "rotate";
const char * P_HEADING = "heading";
const char * P_POWER = "power";

static const drivetrain_config_t dt_conf = {
    .in1 = IN1_PINS, 
    .in2 = IN2_PINS, 
    .pwm = PWM_PINS, 
    .enca = ENCA_PINS, 
    .encb = ENCB_PINS,
    .rpm_factor = RPM_FACTORS
    };

static const robot_config_t rb_conf = {
        .dt_conf = dt_conf,
        .led_pin = LED_PIN
};

static int encoder1 = 0;
static int encoder2 = 0;
static int encoder3 = 0;
static int encoder4 = 0;

static int rpms[4] = {0,0,0,0};
static float min_rpm = 0;
static float rpm_factors[4] = {1.0, 1.0, 1.0, 1.0};

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    int motor_num = (int) arg;

    switch(motor_num) {
        case 0:
            encoder1++;
            break;
        case 1:
            encoder2++;
            break;
        case 2:
            encoder3++;
            break;
        case 3:
            encoder4++;
            break;

    }
}

static void clear_encoders() {
    encoder1 = 0;
    encoder2 = 0;
    encoder3 = 0;
    encoder4 = 0;
}
static void calc_rpm() {

    rpms[0] = encoder1 * 60 * 1000.0 / CALIBRATION_PERIOD;
    rpms[1] = encoder2 * 60 * 1000.0 / CALIBRATION_PERIOD;
    rpms[2] = encoder3 * 60 * 1000.0 / CALIBRATION_PERIOD;
    rpms[3] = encoder4 * 60 * 1000.0 / CALIBRATION_PERIOD;

    clear_encoders();
    
    min_rpm = rpms[0];
    int i;
    for(i=1; i<4; i++) {
        if(rpms[i] < min_rpm) {
            min_rpm = rpms[i];
        }
    }
    for(i=0; i<4; i++) {
        if(min_rpm > 0) {
            rpm_factors[i] = min_rpm/rpms[i];
        } else {
            rpm_factors[i] = 1.0;
        }
    }

    printf("min_rpm: %f\n", min_rpm);
    for(int i=0; i< 4; i++) {
        printf("rpm-%d: %d, adj_factor: %f\n", i, rpms[i], rpm_factors[i]);
    }
    
}

static void calibrate_motors() {
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    
    int count = 0;
    for(count=0; count < 4; count++) {
        gpio_isr_handler_add(dt_conf.enca[count], gpio_isr_handler, (void*) count);
    }

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    for(count = 0; count < 4; count++) {
        drivetrain_motor_spin(&dt_conf, count, 1.0);
    }

    for(count=0;count<5;count++){    
        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_PERIOD));
        calc_rpm();
    }
    robot_stop(&rb_conf);

    for(count=0; count < 4; count++) {
        gpio_isr_handler_remove(dt_conf.enca[count]);
    }

}

const char * html_doc = "<!DOCTYPE html>"
"<html lang='en'>"
    "<head>"
        "<title>ESP Web Server</title>"
        "<meta content='width=device-width, initial-scale=1' name='viewport'>"
        "<link href='data:,' rel='icon'>"
        "<style>"
            "html {font-family: Helvetica;display: inline-block;margin: 0 auto;text-align: center;}"
            "h1 {color: #0F3376;padding: 2vh;}"
            "p {font-size: 1rem;}.button {display: inline-block;background-color: #e7bd3b;border: none;border-radius: 4px;color: white;padding: 8px 20px;text-decoration: none;font-size: 16px;margin: 2px 8px 2px 8px;cursor: pointer;}"
            ".button-red {background-color: red;}"
            ".button-green {background-color: green;}"
            "</style>"
    "</head>"
    "<body>"
        "<h1>ESP Rover</h1>"
        "<p>Action: <strong>{{action}}</strong></p>"
        "<p>"
            "<a href='/hello?cmd=led&status=1'>"
                "<button class='button'>LED ON</button>"
            "</a>"
            "<a href='/hello?cmd=led&status=0'><button class='button button-red'>LED OFF</button></a>"
        "</p>"
        "<hr/>"
        "<p>"
            "<a href='/hello?cmd=drive&heading=315&power=80'><button class='button button-green'>Forward LT</button></a>"
            "<a href='/hello?cmd=drive&heading=0&power=80'><button class='button button-green'>Forward</button></a>"
            "<a href='/hello?cmd=drive&heading=45&power=80'><button class='button button-green'>Forward RT</button></a>"
        "</p>"
        "<p>"
            "<a href='/hello?cmd=rotate&heading=-1&power=80'><button class='button button-green'>Rot Left</button></a>"
            "<a href='/hello?cmd=drive&heading=270&power=80'><button class='button button-green'>Left</button></a>"
            "<a href='/hello?cmd=drive&heading=90&power=80'><button class='button button-green'>Right</button></a>"
            "<a href='/hello?cmd=rotate&heading=1&power=80'><button class='button button-green'>Rot Right</button></a>"
        "</p>"
        "<p>"
            "<a href='/hello?cmd=drive&heading=225&power=80'><button class='button button-green'>Reverse LT</button></a>"
            "<a href='/hello?cmd=drive&heading=180&power=80'><button class='button button-green'>Reverse</button></a>"
            "<a href='/hello?cmd=drive&heading=135&power=80'><button class='button button-green'>Reverse RT</button></a>"
        "</p>"
        "<hr/>"
        "<p>"
            "<a href='/hello?cmd=stop'><button class='button button-red'>Stop</button></a>"
        "</p>"
        "<p></p>"
    "</body>"
"</html>";
#endif