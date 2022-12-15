//
// Created by Kevin Williams on 12/10/22.
//
#include <esp_log.h>
#include "espserial.h"

//#define SERIAL_DEBUG 1

static const char* TAG = "ESP SERIAL";
static const uart_port_t uart_num = UART_NUM_2;
static QueueHandle_t * uart_queue = nullptr;

ESPSerial::ESPSerial(uart_port_t port,  int tx_pin, int rx_pin, int baudrate) : _port(port), _tx_pin(tx_pin), _rx_pin(rx_pin), _baudrate(baudrate) {
    init();
}

esp_err_t ESPSerial::init()  {
    if(uart_is_driver_installed(_port))
        return ESP_OK;


    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(_port, _tx_pin, _rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const int uart_buffer_size = (1024 * 2);
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 4096, 256, 10, uart_queue, 0));

    flush();
    flushInput();
    return ESP_OK;
}

void ESPSerial::printData(uint8_t *data, size_t size) {
#ifdef SERIAL_DEBUG
    ESP_LOGI(TAG, "Got %d bytes", size);

    for(int i=0; i < size; i++) {
        ESP_LOGI(TAG, "%d", data[i]);
    }
#endif
}

size_t  ESPSerial::bytesAvailable() {
    size_t bytes = 0;
    auto status = uart_get_buffered_data_len(_port, &bytes);
    return(status == ESP_OK ? bytes : 0);
}



void ESPSerial::flush() {
    uart_flush(_port);
}

void  ESPSerial::flushInput() {
    size_t length = 0;
    uint8_t buff[1024];
    uart_get_buffered_data_len(_port, &length);
    while(length > 0) {
        uart_read_bytes(_port, &buff, length > 1024 ? 1024 : length, 100);
        uart_get_buffered_data_len(_port, &length);
    }
}

uint8_t  ESPSerial::read() {
    uint8_t b;
    while(!bytesAvailable()) {
        vTaskDelay(10);
    }
    uart_read_bytes(_port, &b, 1, 10);
    return b;
}

esp_err_t  ESPSerial::read(uint8_t * data, size_t len, uint32_t timeout_us) {
    size_t length = uart_read_bytes(_port, data, len, timeout_us);
    if(length >= len)
        return ESP_OK;
    else
        return ESP_FAIL;
}

int  ESPSerial::write(const void * data, size_t len) {
    return uart_write_bytes(_port, data, len);
}


void ESPSerial::deInit() {
    if(uart_is_driver_installed(_port))
        uart_driver_delete(_port);
}

ESPSerial::~ESPSerial() {
    deInit();
}

void ESPSerial::clearDTR() {
    uart_set_dtr(_port, 0);
}

void ESPSerial::setDTR() {
    uart_set_dtr(_port, 1);
}

esp_err_t ESPSerial::waitForData(uint64_t waitTicks) {
    int64_t ticks = 0;
    while(!bytesAvailable()) {
        vTaskDelay(10);
        ticks += 10;
        if(ticks > waitTicks) return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t ESPSerial::waitForBytes(size_t bytes, uint64_t waitTicks) {
    size_t available = 0;
    int64_t ticks = 0;
    while(ticks < waitTicks) {
        esp_err_t status = uart_get_buffered_data_len(_port, &available);
        if(status != ESP_OK) return status;
        if(available >= bytes) return ESP_OK;
        vTaskDelay(10);
        ticks+=10;
    }

    return ESP_ERR_TIMEOUT;
}
