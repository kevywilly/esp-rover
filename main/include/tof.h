//
// Created by Kevin Williams on 11/4/22.
//

#ifndef ESPROVER_TOF_H
#define ESPROVER_TOF_H

#include <esp_log.h>
#include "tof_sensor.hpp"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000         //100000   ??          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define NUM_TOF_SENSORS 4
#define TOF_START_ADDRESS 0x31

static int tof_failures = 0;
static TOFSensor tof_sensors[4] = {
        TOFSensor(I2C_MASTER_NUM, 0X29, CONFIG_TOF0_XSHUT),
        TOFSensor(I2C_MASTER_NUM, 0X29, CONFIG_TOF1_XSHUT),
        TOFSensor(I2C_MASTER_NUM, 0X29, CONFIG_TOF2_XSHUT),
        TOFSensor(I2C_MASTER_NUM, 0X29, CONFIG_TOF3_XSHUT)
};

static void tof_init_xshuts() {
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        gpio_num_t pin = (gpio_num_t)tof_sensors[i].xshut;
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
        gpio_set_level(pin, 0);
    }
}

static void tof_init_i2c() {
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = 100000;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(
            i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_MASTER_NUM, 80000));       // Clock stretching
    ESP_ERROR_CHECK(i2c_filter_enable(I2C_MASTER_NUM, 5));

}

static void tof_init_one(TOFSensor * sensor, uint8_t new_addr) {
    sensor->address = 0x29;
    sensor->init();
    sensor->setAddress(new_addr);
}

static void tof_init_all() {
    tof_failures = 0;
    tof_init_xshuts();
    tof_init_i2c();
    uint8_t addr = TOF_START_ADDRESS;
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        tof_init_one(&tof_sensors[i], addr++);
    }
}

static void tof_start_all() {
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        //tof_sensors[i].startContinuous(10);
    }
}


static esp_err_t tof_read_all_avg(uint16_t *distances, uint8_t count, uint8_t ms) {
    for(int i=0; i < NUM_TOF_SENSORS; i++) {
        distances[i] = 0;
    }
    esp_err_t status = ESP_OK;

    for(int i=0; i < count; i++) {
        for (int i = 0; i < NUM_TOF_SENSORS; i++) {
            uint16_t d = tof_sensors[i].readRangeSingleMillimeters();
            if(tof_sensors[i].did_timeout)
                status = ESP_FAIL;
            distances[i] = distances[i] + d/count;
        }
        usleep(ms*1000);
    }
    return ESP_OK;
}
static esp_err_t tof_read_all(uint16_t *distances) {
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        uint16_t d = tof_sensors[i].readRangeSingleMillimeters();
        if(tof_sensors[i].did_timeout) {
            tof_failures++;
            if(tof_failures >= 3) {
#ifdef CONFIG_ESP_ROVER_DEBUG
                ESP_LOGI(TAG, "TOF %d has failed multiple times", i);
#endif
                return ESP_FAIL;
            }
        } else {
            distances[i] = d;
        }
#ifdef    CONFIG_VL53L0X_DEBUG
        ESP_LOGI(TAG,"Got distance %d: %d", i, distances[i]);
#endif
    }
    return ESP_OK;
}

#endif //ESPROVER_TOF_H
