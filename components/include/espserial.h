//
// Created by Kevin Williams on 12/10/22.
//

#pragma once

#pragma once
#include "driver/uart.h"

class ESPSerial {

public:
    ESPSerial(uart_port_t port,  int tx_pin, int rx_pin, int baudrate);

    virtual ~ESPSerial();

    esp_err_t init();
    void deInit();
    void printData(uint8_t * data, size_t size);

    size_t bytesAvailable();

    void flush();
    void flushInput();
    void clearDTR();
    void setDTR();

    uint8_t read();
    esp_err_t read(uint8_t * data, size_t len, uint32_t timeout_us = 100);

    int write(const void * data, size_t len);
    int write(uint8_t byte) {return write(&byte, 1);}
    esp_err_t waitForData(uint64_t waitTicks = 1000);
    esp_err_t waitForBytes(size_t bytes, uint64_t waitTicks = 1000);


private:
    uart_port_t _port;
    int _tx_pin;
    int _rx_pin;
    int _baudrate;
};