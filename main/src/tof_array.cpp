//
// Created by Kevin Williams on 11/23/22.
//

#include <thread>
#include "tof_array.hpp"

TOFArray::TOFArray() {
    sensors = new TOFSensor[size]{
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF0_XSHUT), 90+12),
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF1_XSHUT), 90),
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF2_XSHUT), 90-12),
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF3_XSHUT), 180),
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF4_XSHUT), 0)
    };
}

void TOFArray::init() {
    initI2c();
    initXShuts();
    initAllSensors();
}

void TOFArray::initI2c() {
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

void TOFArray::initXShuts() {
    for (int i = 0; i < size; i++) {
        gpio_num_t pin = sensors[i].device->xshut;
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
        gpio_set_level(pin, 0);
    }
}

void TOFArray::initSensor(VL53L0X *sensor, uint8_t newAddr) {
    sensor->address = VL53L0X_DEFAULT_ADDRESS;
    sensor->init();
    sensor->setAddress(newAddr);
}

void TOFArray::initAllSensors() {
    uint8_t addr = 0x31;
    for(int i=0; i < size; i++) {
        initSensor(sensors[i].device, addr++);
    }
}

void TOFArray::readAll() {
    for(int i=0; i < size; i++) {
        sensors[i].ping();
    }
    encodeProximity();
}

void TOFArray::readAllAvg() {
    uint16_t distances[size];
    for(int i=0; i < size; i++) {
        distances[i] = 0;
    }

    for(int i=0; i < 3; i++) {
        for(int j = 0; j < size; j++) {
            distances[j] = distances[j] + sensors[j].device->readRangeSingleMillimeters()/3;
        }
        usleep(1000);
    }

    for(int j = 0; j < size; j++) {
        sensors[j].distance = distances[j];
    }
    encodeProximity();
}

void TOFArray::encodeProximity() {
    uint8_t p = 0;
    uint8_t pos = 4;
    for(int i=0; i < size; i++) {
        uint8_t v = sensors[i].distance > thresholds[i] ? 1 : 0;
        p = p + (v << pos);
        pos--;
    }
    proximity = p;
}
