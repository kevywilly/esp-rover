//
// Created by Kevin Williams on 11/28/22.
//
#include <esp_http_server.h>
#include <app_auto_drive.hpp>
#include "robot.hpp"

#ifndef ESPROVER_APP_HTTPD_H
#define ESPROVER_APP_HTTPD_H
httpd_handle_t start_webserver(Robot *robot);
#endif //ESPROVER_APP_HTTPD_H
