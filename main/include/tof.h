//
// Created by Kevin Williams on 11/4/22.
//

#ifndef ESPROVER_TOF_H
#define ESPROVER_TOF_H

#include "vl53l0x.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000         //100000   ??          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define NUM_TOFS 4
#define TOF_START_ADDRESS 0x31

static vl53l0x_t tofs[4] = {
        {.xshut = CONFIG_TOF0_XSHUT, .address = 0x29, .port=I2C_MASTER_NUM, .io_2v8 = 0, .io_timeout = 100},
        {.xshut = CONFIG_TOF1_XSHUT, .address = 0x29, .port=I2C_MASTER_NUM, .io_2v8 = 0, .io_timeout = 100},
        {.xshut = CONFIG_TOF2_XSHUT, .address = 0x29, .port=I2C_MASTER_NUM, .io_2v8 = 0, .io_timeout = 100},
        {.xshut = CONFIG_TOF3_XSHUT, .address = 0x29, .port=I2C_MASTER_NUM, .io_2v8 = 0, .io_timeout = 100}
};

static void tof_init_xshuts() {
    for(int i=0; i < NUM_TOFS; i++) {
        gpio_num_t pin = tofs[i].xshut;
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
            .master.clk_speed = 100000 //I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
    ESP_ERROR_CHECK(i2c_set_timeout (I2C_MASTER_NUM, 80000));       // Clock stretching
    ESP_ERROR_CHECK(i2c_filter_enable (I2C_MASTER_NUM, 5));

}

static void tof_init_all() {
    tof_init_xshuts();
    tof_init_i2c();
    uint8_t addr = TOF_START_ADDRESS;
    for(int i=0; i < NUM_TOFS; i++) {
        vl53l0x_init(&tofs[i]);
        vl53l0x_setAddress(&tofs[i], addr++);
    }
}

static void tof_read_all(uint16_t * distances) {
    for(int i=0; i < NUM_TOFS; i++) {
        distances[i] = vl53l0x_readRangeSingleMillimeters(&tofs[i]);
#ifdef	CONFIG_VL53L0X_DEBUG
        ESP_LOGI(TAG,"Got distance %d: %d", i, distances[i]);
#endif
    }
}


#endif //ESPROVER_TOF_H
