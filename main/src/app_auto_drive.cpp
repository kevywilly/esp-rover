//
// Created by Kevin Williams on 11/21/22.
//

#include <thread>
#include "vl53l0x.hpp"
#include "app_auto_drive.hpp"
#include "app_drive.hpp"

static const char TAG[] = "AppAutodrive";

AppAutoDrive::AppAutoDrive(QueueHandle_t queueMotion, QueueHandle_t queueAutoDrive) : _queueMotion(
        queueMotion), _queueAutoDrive(queueAutoDrive), proximity(nullptr, 0) {


    sensors = new TOFSensor[NUM_SENSORS]{
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF0_XSHUT), 90+12),
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF1_XSHUT), 90),
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF2_XSHUT), 90-12),
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF3_XSHUT), 0),
            TOFSensor(new VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF4_XSHUT), 180)
    };

    failures = 0;

    initXShuts();
    initI2c();
    initAllSensors();

    proximity.sensors = sensors;
    proximity.numSensors = NUM_SENSORS;
    proximity.analyze();

}

void AppAutoDrive::initI2c() {
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

void AppAutoDrive::initXShuts() {

    for (int i = 0; i < NUM_SENSORS; i++) {
        gpio_num_t pin = sensors[i].device->xshut;
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
        gpio_set_level(pin, 0);
    }
}

void AppAutoDrive::initSensor(VL53L0X *sensor, uint8_t newAddr) {
    sensor->address = VL53L0X_DEFAULT_ADDRESS;
    sensor->init();
    sensor->setAddress(newAddr);
}

void AppAutoDrive::initAllSensors() {
    uint8_t addr = 0x31;
    for(int i=0; i < NUM_SENSORS; i++) {
        initSensor(sensors[i].device, addr++);
    }
}

void AppAutoDrive::readAll() {
    for(int i=0; i < NUM_SENSORS; i++) {
        sensors[i].ping();
    }
    this->proximity.analyze();
}

void AppAutoDrive::readAllAvg() {

    uint16_t distances[NUM_SENSORS];
    for(int i=0; i < NUM_SENSORS; i++) {
        distances[i] = 0;
    }

    for(int i=0; i < 3; i++) {
        for(int j = 0; j < NUM_SENSORS; j++) {
            distances[j] = distances[j] + sensors[j].device->readRangeSingleMillimeters()/3;
        }
        usleep(1000);
    }

    for(int j = 0; j < NUM_SENSORS; j++) {
        sensors[j].distance = distances[j];
    }

    this->proximity.analyze();

}

void AppAutoDrive::stop() {
    _isActive = false;

}

esp_err_t AppAutoDrive::drive() {

    drive_command_t cmd;
    readAllAvg();


    if (proximity.clear()) {
        cmd = {0.5, 90, 0};
        xQueueSend(_queueMotion, &cmd, 10);
        while (proximity.clear() && checkIsActive()) {
            readAndWait(10);
        }
    } else if (proximity.frontClear()) {
        cmd = {0.5, proximity.headLeftOrRight(), 0};
        xQueueSend(_queueMotion, &cmd, 10);
    } else {
        cmd = {0, 0, proximity.spinLeftOrRight()};
        xQueueSend(_queueMotion, &cmd, 10);
        while (!proximity.frontClear() && checkIsActive()) {
            readAndWait(10);
        }
    }

    return ESP_OK;
}

bool AppAutoDrive::checkIsActive() {
    uint8_t buf = 0;
    if (xQueueReceive(_queueAutoDrive, (void *) &buf, 0) == pdTRUE) {
        _isActive = !_isActive;
        if(_isActive) {
            ESP_LOGI(TAG, "Autodrive is ACTIVE");
        }
        else {
            ESP_LOGI(TAG, "Autodrive is INACTIVE");
        }
    }
    return _isActive;
}

static void task(AppAutoDrive *self) {
    while(1) {
        ESP_LOGI(TAG, "RUNNING PROXIMITY...\n");

        self->checkIsActive();

        if(self->_isActive) {
            self->drive();
        } else {
            self->readAllAvg();
            for (int i = 0; i < self->NUM_SENSORS; i++) {
                ESP_LOGI(TAG, "Reading %d : %d", self->sensors[i].angle, self->sensors[i].distance);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void AppAutoDrive::run() {

    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 5 * 1024, this, 5, NULL, 1);
}

