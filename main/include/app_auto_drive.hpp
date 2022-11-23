//
// Created by Kevin Williams on 11/21/22.
//

#ifndef ESPROVER_APP_PROXIMITY_H
#define ESPROVER_APP_PROXIMITY_H

#include <esp_log.h>
#include "vl53l0x.hpp"
#include "driver/i2c.h"
#include "freertos/freertos.h"
#include "freertos/queue.h"
#include "proximity.hpp"
#include "app_drive.hpp"
#include "tof_sensor.hpp"

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000         //100000   ??          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

class AppAutoDrive {
public:

    TOFSensor * sensors;

    static const int NUM_SENSORS = 5;

    QueueHandle_t _queueMotion;
    QueueHandle_t _queueAutoDrive;

    bool _isActive = false;
    int failures;

    Proximity proximity;

    AppAutoDrive(QueueHandle_t queueMotion = nullptr, QueueHandle_t queueAutoDrive = nullptr);

    void run();

    void readAll();

    void readAllAvg();

    void stop();

    void sendCmd(drive_command_t cmd);

    bool checkIsActive();

    inline void readAndWait(int32_t millis) {
        readAllAvg();
        vTaskDelay(pdMS_TO_TICKS(millis));
    }

    esp_err_t drive();

private:

    void initI2c();
    void initXShuts();
    void initSensor(VL53L0X * sensor, uint8_t newAddr);
    void initAllSensors();
};


#endif //ESPROVER_APP_PROXIMITY_H
