
#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define pins for motor control
#define IN1_PINS   {GPIO_NUM_14, GPIO_NUM_16, GPIO_NUM_21,  GPIO_NUM_0}
#define IN2_PINS   {GPIO_NUM_12, GPIO_NUM_25, GPIO_NUM_22, GPIO_NUM_15}
#define PWM_PINS   {GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_23,  GPIO_NUM_4}
#define ENCA_PINS  {GPIO_NUM_32, GPIO_NUM_34, GPIO_NUM_18,  GPIO_NUM_5}
#define ENCB_PINS  {GPIO_NUM_13, GPIO_NUM_35, GPIO_NUM_19, GPIO_NUM_17}

// Define led pin
#define LED_PIN GPIO_NUM_2

static const char *TAG = "esprover";

#endif