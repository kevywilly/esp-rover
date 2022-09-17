#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include "esp_tls_crypto.h"
#include <esp_http_server.h>
#include "robot/robot.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include <fcntl.h>
#include "cJSON.h"
#include "docroot.h"
#include <sys/param.h>

static char * process_request(cJSON *root) {

    char * cmd = cJSON_GetObjectItem(root, CONFIG_CMD_PARAM_NAME)->valuestring;

    ESP_LOGI(TAG, "received command cmd = %s", cmd);
    if(strcmp(cmd, CONFIG_DRIVE_COMMAND) == 0 || strcmp(cmd, CONFIG_ROTATE_COMMAND) == 0) {
        double heading = cJSON_GetObjectItem(root, CONFIG_HEADING_PARAM_NAME)->valuedouble;
        double power = cJSON_GetObjectItem(root, CONFIG_POWER_PARAM_NAME)->valuedouble;

        ESP_LOGI(TAG, "Received Command: <%s heading = %f, power = %f>", cmd, heading, power);

        if(strcmp(cmd, "drive") == 0) {
            robot_drive(&robot, heading, power);
            return "Processed DRIVE Request.";
        } else if(strcmp(cmd, "rotate") == 0) {
            robot_rotate(&robot, heading, power);
            return "Processed DRIVE Request.";
        }
    }

    return "Unknown api request was ignored.";
}

/* An HTTP GET handler */
static esp_err_t home_get_handler(httpd_req_t *req)
{
    httpd_resp_send(req, docroot, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t api_post_handler(httpd_req_t *req) {

    char buf[1000];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                                  MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    cJSON *root = cJSON_Parse(buf);
    char * resp = process_request(root);
    cJSON_Delete(root);

    // End response
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static httpd_handle_t start_webserver()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    httpd_uri_t home_get_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = home_get_handler,
            .user_ctx  = NULL
    };

    httpd_uri_t api_post_uri = {
            .uri       = "/api",
            .method    = HTTP_POST,
            .handler   = api_post_handler,
            .user_ctx  = NULL
    };

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &home_get_uri);
        httpd_register_uri_handler(server, &api_post_uri);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
