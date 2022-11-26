//
// Created by Kevin Williams on 11/23/22.
//

#ifndef ESPROVER_TOFARRAY_H
#define ESPROVER_TOFARRAY_H
#include "vl53l0x.hpp"
#include "tof_sensor.hpp"

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000         //100000   ??          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define FL 0b10000
#define FM 0b01000
#define FR 0b00100
#define LFT 0b00010
#define RGT 0b00001
#define FRONT (FL+FM+FR)
#define SIDES (LFT+RGT)

class TOFArray {
public:

    static const int size = 5;
    uint16_t thresholds[5] = {400, 400, 400, 250, 250};

    TOFSensor * sensors;
    uint8_t proximity = 0b00000;

    TOFArray();
    void initI2c();

    void initXShuts();

    void initSensor(VL53L0X *sensor, uint8_t newAddr);

    void initAllSensors();

    void readAll();

    void readAllAvg();

    void init();

    void encodeProximity();

    inline int isClear(uint8_t mask) {
        return (proximity & mask) == mask;
    }

    inline int isClear(uint8_t val, uint8_t mask) {
        return (val & mask) == mask;
    }
};
#endif //ESPROVER_TOFARRAY_H
