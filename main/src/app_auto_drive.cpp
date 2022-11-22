//
// Created by Kevin Williams on 11/21/22.
//

#include <thread>
#include "vl53l0x.hpp"
#include "app_auto_drive.hpp"
#include "app_drive.hpp"

static const char TAG[] = "AppAutodrive";

AppAutoDrive::AppAutoDrive(QueueHandle_t queueMotion, QueueHandle_t queueAutoDrive) : _queueMotion(
        queueMotion), _queueAutoDrive(queueAutoDrive) {

    sensors = new VL53L0X[NUM_SENSORS]{
            VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF0_XSHUT),
            VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF1_XSHUT),
            VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF2_XSHUT),
            VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF3_XSHUT),
            VL53L0X(I2C_MASTER_NUM, 0x29, (gpio_num_t)CONFIG_TOF4_XSHUT)
    };

    failures = 0;

    initXShuts();
    initI2c();
    initAllSensors();

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
    gpio_num_t xshuts[5] = {
            (gpio_num_t)CONFIG_TOF0_XSHUT,
            (gpio_num_t)CONFIG_TOF1_XSHUT,
            (gpio_num_t)CONFIG_TOF2_XSHUT,
            (gpio_num_t)CONFIG_TOF3_XSHUT,
            (gpio_num_t)CONFIG_TOF4_XSHUT,
    };

    for (int i = 0; i < 5; i++) {
        gpio_num_t pin = xshuts[i];
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
        initSensor(&sensors[i], addr++);
    }
}

void AppAutoDrive::readAll() {

    for(int i=0; i < NUM_SENSORS; i++) {
        proximity.readings[i] = sensors[i].readRangeSingleMillimeters();
    }
    this->proximity.analyze();

}

void AppAutoDrive::readAllAvg() {

    uint16_t readings[NUM_SENSORS];
    for(int i=0; i < NUM_SENSORS; i++) {
        readings[i] = 0;
    }


    for(int i=0; i < 3; i++) {
        for(int j = 0; j < NUM_SENSORS; j++) {
            readings[j] = readings[j] + sensors[j].readRangeSingleMillimeters()/3;
        }
        usleep(1000);
    }

    for(int j = 0; j < NUM_SENSORS; j++)
        this->proximity.readings[j] = readings[j];

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
        ESP_LOGI(TAG, "RUNNING PROXIMITY...");

        self->checkIsActive();

        if(self->_isActive) {
            self->drive();
        } else {
            self->readAllAvg();
            for(int i=0; i < self->NUM_SENSORS; i++) {
                ESP_LOGI(TAG, "Proximity %d : %d", i, self->proximity.readings[i]);
            }
            /*
            ESP_LOGI(TAG, "Proximity: front: (%d), %d, %d, %d, sides: (%d), %d, %d",
                     self->proximity.front,
                     self->proximity.frontLeft,
                     self->proximity.frontMiddle,
                     self->proximity.frontRight,
                     self->proximity.sides,
                     self->proximity.right,
                     self->proximity.left
                     );
                     */
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void AppAutoDrive::run() {

    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 5 * 1024, this, 5, NULL, 1);
}

