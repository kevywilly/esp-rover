//
// Created by Kevin Williams on 12/6/22.
//

typedef uint8_t _u8;
typedef uint16_t _u16;
typedef uint32_t _u32;

#pragma once
typedef struct _lidar_cmd_t {
    _u8 startByte;
    _u8 cmdByte;
    _u8 size;
    _u8 data[0];
} __attribute__((packed)) lidar_cmd_t;

typedef struct _lidar_descriptor_t {
    _u8  syncByte1; // must be RPLIDAR_ANS_SYNC_BYTE1
    _u8  syncByte2; // must be RPLIDAR_ANS_SYNC_BYTE2
    _u32 size:30;
    _u32 subType:2;
    _u8  type;
} __attribute__((packed)) lidar_descriptor_t;

typedef struct _lidar_info_t {
    _u8 model;
    _u8 minor;
    _u8 major;
    _u8 hardware;
    _u8 serialNumber[16];
} __attribute__((packed)) lidar_info_t;

typedef struct _lidar_health_t{
    _u8 status;
    _u16 error_code;
}  __attribute__((packed)) lidar_health_t;

typedef struct _lidar_measurement_response_t {
    _u8    sync_quality;      // startFlag:1;inverseStartFlag:1;quality:6;
    _u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
    _u16   distance_q2;
} __attribute__((packed)) lidar_measurement_response_t;

typedef struct _lidar_measurement_t{
    _u8 newScan;
    _u8 quality;
    float angle;
    float distance;
} __attribute__((packed)) lidar_measurement_t;
