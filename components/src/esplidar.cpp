//
// Created by Kevin Williams on 12/6/22.
//

#include <esplidar.h>
#include <driver/mcpwm.h>
#include <mutex>
#include "espserial.h"
#include "sl_lidar_cmd.h"
#include <algorithm>

#define EXPRESS_SIZE 132
#define OFFSET_MEASUREMENT_CANSULED_ULTRA_CRC 2

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef _delay_ms
#define _delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#endif

#ifndef _delay
#define _delay(ticks) vTaskDelay(ticks)
#endif

static const char * TAG = "LIDAR";

static QueueHandle_t * uart_queue = nullptr;

static sl_lidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
static bool _is_previous_capsuledataRdy = false;

static sl_u32 _varbitscale_decode(sl_u32 scaled, sl_u32 & scaleLevel)
{
    static const sl_u32 VBS_SCALED_BASE[] = {
            SL_LIDAR_VARBITSCALE_X16_DEST_VAL,
            SL_LIDAR_VARBITSCALE_X8_DEST_VAL,
            SL_LIDAR_VARBITSCALE_X4_DEST_VAL,
            SL_LIDAR_VARBITSCALE_X2_DEST_VAL,
            0,
    };

    static const sl_u32 VBS_SCALED_LVL[] = {
            4,
            3,
            2,
            1,
            0,
    };

    static const sl_u32 VBS_TARGET_BASE[] = {
            (0x1 << SL_LIDAR_VARBITSCALE_X16_SRC_BIT),
            (0x1 << SL_LIDAR_VARBITSCALE_X8_SRC_BIT),
            (0x1 << SL_LIDAR_VARBITSCALE_X4_SRC_BIT),
            (0x1 << SL_LIDAR_VARBITSCALE_X2_SRC_BIT),
            0,
    };

    for (size_t i = 0; i < _countof(VBS_SCALED_BASE); ++i) {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << scaleLevel);
        }
    }
    return 0;
}

void _ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_ultracapsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3) / 3;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_ultracapsuledata.ultra_cabins); ++pos) {
            int dist_q2[3];
            int angle_q6[3];
            int syncBit[3];


            sl_u32 combined_x3 = _cached_previous_ultracapsuledata.ultra_cabins[pos].combined_x3;

            // unpack ...
            int dist_major = (combined_x3 & 0xFFF);

            // signed partical integer, using the magic shift here
            // DO NOT TOUCH

            int dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
            int dist_predict2 = (((int)combined_x3) >> 22);

            int dist_major2;

            sl_u32 scalelvl1, scalelvl2;

            // prefetch next ...
            if (pos == _countof(_cached_previous_ultracapsuledata.ultra_cabins) - 1) {
                dist_major2 = (capsule.ultra_cabins[0].combined_x3 & 0xFFF);
            }
            else {
                dist_major2 = (_cached_previous_ultracapsuledata.ultra_cabins[pos + 1].combined_x3 & 0xFFF);
            }

            // decode with the var bit scale ...
            dist_major = _varbitscale_decode(dist_major, scalelvl1);
            dist_major2 = _varbitscale_decode(dist_major2, scalelvl2);


            int dist_base1 = dist_major;
            int dist_base2 = dist_major2;

            if ((!dist_major) && dist_major2) {
                dist_base1 = dist_major2;
                scalelvl1 = scalelvl2;
            }


            dist_q2[0] = (dist_major << 2);
            if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) {
                dist_q2[1] = 0;
            }
            else {
                dist_predict1 = (dist_predict1 << scalelvl1);
                dist_q2[1] = (dist_predict1 + dist_base1) << 2;

            }

            if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
                dist_q2[2] = 0;
            }
            else {
                dist_predict2 = (dist_predict2 << scalelvl2);
                dist_q2[2] = (dist_predict2 + dist_base2) << 2;
            }


            for (int cpos = 0; cpos < 3; ++cpos) {
                syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;

                int offsetAngleMean_q16 = (int)(7.5 * 3.1415926535 * (1 << 16) / 180.0);

                if (dist_q2[cpos] >= (50 * 4))
                {
                    const int k1 = 98361;
                    const int k2 = int(k1 / dist_q2[cpos]);

                    offsetAngleMean_q16 = (int)(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
                }

                angle_q6[cpos] = ((currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
                currentAngle_raw_q16 += angleInc_q16;

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                sl_lidar_response_measurement_node_hq_t node;

                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2F << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.angle_z_q14 = sl_u16((angle_q6[cpos] << 8) / 90);
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
            }

        }
    }

    _cached_previous_ultracapsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}

static void printDescriptor(lidar_descriptor_t &descriptor) {
    ESP_LOGI(TAG, "Descriptor: size: %d, subType: %d, type: %d", descriptor.size, descriptor.subType, descriptor.type);
}

ESPLidar::ESPLidar() :
_scanning(false),
_motor_running(false){
    init();
}

void ESPLidar::init() {
    _motor_pin = (gpio_num_t) CONFIG_RPLIDAR_MOTOR_GPIO;

    _serial = new ESPSerial(
            CONFIG_RPLIDAR_UART_PORT_NUM,
            CONFIG_RPLIDAR_UART_TX_PIN,
            CONFIG_RPLIDAR_UART_RX_PIN,
            CONFIG_RPLIDAR_UART_BAUDRATE);

    gpio_reset_pin(_motor_pin);
    gpio_set_direction(_motor_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(_motor_pin, GPIO_PULLUP_ONLY);

    /*
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, _motor_pin);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 25;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);    //Configure PW
    */
    _serial->init();
    _delay_ms(10);
    _serial->flushInput();
}

void ESPLidar::startMotor() {
    gpio_set_level(_motor_pin, 1);
    /*
    mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A,50);
    mcpwm_set_duty_type(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
     */
    _motor_running = true;
    _delay_ms(1000);
}

void ESPLidar::stopMotor() {
    if(_scanning) stopScan();
    _motor_running = false;
    /*
    mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A,0);
    mcpwm_set_duty_type(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
     */
    gpio_set_level(_motor_pin, 0);
    _delay_ms(1000);
}

esp_err_t ESPLidar::reset() {
    ESP_LOGI(TAG, "Resetting lidar");
    _sendCommand(SL_LIDAR_CMD_RESET);
    _serial->waitForData(3000);
    _serial->flushInput();
    return ESP_OK;
}

esp_err_t ESPLidar::getDescriptor(lidar_descriptor_t & descriptor, _u32 timeout) {
    if(_serial->waitForBytes(7) == ESP_OK) {
        _serial->read((uint8_t *) &descriptor, sizeof(lidar_descriptor_t), timeout);
        if(descriptor.syncByte1 == SL_LIDAR_ANS_SYNC_BYTE1 || descriptor.syncByte2 == SL_LIDAR_ANS_SYNC_BYTE2) {
            ESP_LOGI(TAG, "Descriptor dsize: %d", descriptor.size);
            _serial->printData((uint8_t*)&descriptor, sizeof(lidar_descriptor_t));
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Bad Descriptor");
    return ESP_FAIL;

}

esp_err_t ESPLidar::getInfo(lidar_info_t & info, _u32 timeout) {
    ESP_LOGI(TAG, "Getting RPLidar Info %d", sizeof (lidar_info_t));

    lidar_descriptor_t descriptor;

    _sendCommand(SL_LIDAR_CMD_GET_DEVICE_INFO);

    if(getDescriptor(descriptor) == ESP_OK) {
        _serial->read((uint8_t *) &info, descriptor.size);
        _serial->printData((uint8_t*)&info, descriptor.size);
    };


    ESP_LOGI(TAG, "LIDAR model: %d, firmware: %d.%d, hardware: %d, serialNumber: %s",
             info.model, info.major, info.minor, info.hardware, (char *)info.serialNumber);


    return ESP_OK;
}

esp_err_t ESPLidar::getHealth(lidar_health_t & health, _u32 timeout) {
    ESP_LOGI(TAG, "Getting RPLidar Health");
    lidar_descriptor_t descriptor;
    _sendCommand(SL_LIDAR_CMD_GET_DEVICE_HEALTH);

    if(getDescriptor(descriptor) == ESP_OK) {
        if(_serial->read((uint8_t *) &health, descriptor.size, 10) == ESP_OK) {
            _serial->printData((uint8_t *) &health, descriptor.size);
            ESP_LOGI(TAG, "LIDAR Health Status: %d, Error Code: %d", health.status, health.error_code);
            return ESP_OK;
        }
    }

    ESP_LOGI(TAG, "Heath Status Fail");
    return ESP_FAIL;


}

uint8_t ESPLidar::calculateCrc(const uint8_t *data, const int bytes) {

    uint8_t crc=0;
    for(int i=0;i<bytes;++i)
        crc ^= data[i];
    return crc;

}

esp_err_t ESPLidar::startExpressScan() {

    if(_scanning) {
        ESP_LOGI("TAG", "Scan already running");
        return ESP_FAIL;
    }

    startMotor();

    ESP_LOGI(TAG, "Starting Express Scan");
    uint8_t data[9];
    data[0]=SL_LIDAR_RESP_MEASUREMENT_HQ_SYNC;
    data[1]=SL_LIDAR_CMD_EXPRESS_SCAN;
    data[2]=0x05; //payload_size
    data[3]=0x02;
    data[4]=0; //reserved
    data[5]=0; //reserved
    data[6]=0; //reserved
    data[7]=0; //reserved
    data[8]=calculateCrc(data, 8); //crc
    _serial->write(data, 9);
    lidar_descriptor_t descriptor;
    esp_err_t status = getDescriptor(descriptor);
    if(status != ESP_OK || descriptor.syncByte1 != 0xA5 || descriptor.syncByte2 != 0x5A || descriptor.size != 132) {
        _scanning = false;
        stopMotor();
        status = ESP_FAIL;
        ESP_LOGI(TAG, "Express Scan Failed");
    } else {
        ESP_LOGI(TAG, "Express Scan Succeded");
        _scanning = true;
    }
    return status;

}

esp_err_t ESPLidar::stopScan() {
    _sendCommand(SL_LIDAR_CMD_STOP);
    _delay_ms(3);
    stopMotor();
    return 0;
}

esp_err_t ESPLidar::readExpressMeasurement(uint8_t * data) {
    MeasurementState state = MeasurementStartFlag1;
    bool skip_packet = false;
    int bytes_to_read = EXPRESS_SIZE;
    int pos=0;

    while(_serial->bytesAvailable())
    {
        uint8_t byte=_serial->read();
        switch(state)
        {
            case MeasurementStartFlag1:
                //ESP_LOGI(TAG, "Reading StartFlag1");
                if(byte >> 4 != SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1)
                {
                    skip_packet=true;
                    continue;
                }
                else {
                    skip_packet = false;
                    data[pos++] = byte;
                }
                //ESP_LOGI(TAG, "Got StartFlag1 %d", pos);

                state = MeasurementStartFlag2;
                break;
            case MeasurementStartFlag2:
               // ESP_LOGI(TAG, "Reading StartFlag2");
                if(byte >> 4 != SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2)
                {
                    pos=0;
                    state=MeasurementStartFlag1;
                    skip_packet=true;
                    continue;
                }
                data[pos++]=byte;
                //ESP_LOGI(TAG, "Got StartFlag2 %d", pos);

                state=MeasurementBytes;
                break;
            case MeasurementBytes:

                //ESP_LOGI(TAG, "Got Data %d", pos);
                data[pos++]=byte;

                if(pos < bytes_to_read)
                    continue;

                //got whole measurement
                state=MeasurementStartFlag1;

                if(checkMeasurementCrc(data)) {
                    _serial->printData(data, EXPRESS_SIZE);
                    return ESP_OK;
                }

                skip_packet=true;
                pos=0;
                break;
            default:
                break; //should never get here,
        }
    }
    return ESP_FAIL;
}

bool ESPLidar::checkMeasurementCrc(uint8_t * data)
{
    uint8_t crc_rcv = (data[0] & 0xF) | (data[1] << 4);
    uint8_t crc_calc = calculateCrc(data + OFFSET_MEASUREMENT_CANSULED_ULTRA_CRC, EXPRESS_SIZE - OFFSET_MEASUREMENT_CANSULED_ULTRA_CRC);

    return crc_rcv == crc_calc;
}

bool ESPLidar::isScanning() {
    return _scanning;
}

void ESPLidar::ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t &capsule,
                                    sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount) {
    _ultraCapsuleToNormal(capsule, nodebuffer, nodeCount);

}

ESPLidar::~ESPLidar() {
    deinit();
}

void ESPLidar::deinit() {
    stopScan();
}

esp_err_t ESPLidar::_sendCommand(uint8_t cmd) {

    uint8_t data[2] = {SL_LIDAR_RESP_MEASUREMENT_HQ_SYNC, cmd};
    _serial->write(data, 2);
    vTaskDelay(pdMS_TO_TICKS(2));

    return ESP_OK;
}
