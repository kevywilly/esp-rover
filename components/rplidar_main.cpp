#include <esp_log.h>
#include "esplidar.h"
#include <algorithm>

static const char * TAG = "esp32-lidar";

static ESPSerial serial(
        CONFIG_RPLIDAR_UART_PORT_NUM,
        CONFIG_RPLIDAR_UART_TX_PIN,
        CONFIG_RPLIDAR_UART_RX_PIN,
        CONFIG_RPLIDAR_UART_BAUDRATE);

static RPLidar * lidar;

extern "C" void app_main(void) {
    lidar = new RPLidar();
    lidar->init();
}