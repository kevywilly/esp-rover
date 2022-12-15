//
// Created by Kevin Williams on 12/6/22.
//

#pragma once

#include <esp_log.h>
#include <cstring>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esplidartypes.h"
#include "sl_lidar_protocol.h"
#include "espserial.h"
#include "sl_lidar_cmd.h"

class ESPLidar {
    enum MeasurementState {
        MeasurementNotAwaiting,
        MeasurementStartFlag1,
        MeasurementStartFlag2,
        MeasurementBytes
    };
private:
    ESPSerial * _serial = nullptr;
    gpio_num_t _motor_pin;
    esp_err_t getDescriptor(lidar_descriptor_t & descriptor, _u32 timeout = 500);
    bool _scanning;
    bool _motor_running;
    QueueHandle_t xQueue_In = nullptr;

public:
    ESPLidar();

    virtual ~ESPLidar();

    void startMotor();
    void stopMotor();
    esp_err_t stopScan();
    uint8_t calculateCrc(const uint8_t *data, const int bytes);
    bool checkMeasurementCrc(uint8_t * data);
    bool isScanning();
    esp_err_t getInfo(lidar_info_t & info, _u32 timeout = 500);
    esp_err_t getHealth(lidar_health_t & health, _u32 timeout = 500);
    esp_err_t reset();
    esp_err_t startExpressScan();
    esp_err_t readExpressMeasurement(uint8_t * data);
    void init();
    void deinit();

    void ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);

    static void convert(const sl_lidar_response_measurement_node_t& from, sl_lidar_response_measurement_node_hq_t& to)
    {
        to.angle_z_q14 = (((from.angle_q6_checkbit) >> SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;  //transfer to q14 Z-angle
        to.dist_mm_q2 = from.distance_q2;
        to.flag = (from.sync_quality & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
        to.quality = (from.sync_quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  //remove the last two bits and then make quality from 0-63 to 0-255
    }

    static void convert(const sl_lidar_response_measurement_node_hq_t& from, sl_lidar_response_measurement_node_t& to)
    {
        to.sync_quality = (from.flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT) | ((from.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        to.angle_q6_checkbit = 1 | (((from.angle_z_q14 * 90) >> 8) << SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
        to.distance_q2 = from.dist_mm_q2 > sl_u16(-1) ? sl_u16(0) : sl_u16(from.dist_mm_q2);
    }

    static inline float getAngle(const sl_lidar_response_measurement_node_t& node)
    {
        return (node.angle_q6_checkbit >> SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.f;
    }

    static inline void setAngle(sl_lidar_response_measurement_node_t& node, float v)
    {
        sl_u16 checkbit = node.angle_q6_checkbit & SL_LIDAR_RESP_MEASUREMENT_CHECKBIT;
        node.angle_q6_checkbit = (((sl_u16)(v * 64.0f)) << SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) | checkbit;
    }

    static inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    static inline void setAngle(sl_lidar_response_measurement_node_hq_t& node, float v)
    {
        node.angle_z_q14 = sl_u32(v * 16384.f / 90.f);
    }

    static inline sl_u16 getDistanceQ2(const sl_lidar_response_measurement_node_t& node)
    {
        return node.distance_q2;
    }

    static inline sl_u32 getDistanceQ2(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.dist_mm_q2;
    }

protected:
    esp_err_t _sendCommand(uint8_t cmd);

};

