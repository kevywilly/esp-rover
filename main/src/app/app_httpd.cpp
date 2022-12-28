#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include <esp_http_server.h>
#include "cJSON.h"
#include "docroot.h"
#include "freertos/queue.h"
#include <app_httpd.h>

static const char *TAG = "ESP Web";

static Robot *pRobot = NULL;


static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data);


static void sendAutoDriveToggleCmmand() {
    uint8_t msg = 1;
    xQueueSend(pRobot->appAutoDrive->xQueue_In, &msg, 10);
}

static void sendDriveCmd(float power, float heading, float turn) {
    drive_command_t cmd = {power, heading, turn};
    xQueueSend(pRobot->appDrive->xQueue_In, &cmd, 10);
}

static esp_err_t process_move_request(cJSON *root) {
    double heading = cJSON_GetObjectItem(root, CONFIG_HEADING_PARAM_NAME)->valuedouble;
    double power = cJSON_GetObjectItem(root, CONFIG_POWER_PARAM_NAME)->valuedouble;
    double turn = cJSON_GetObjectItem(root, CONFIG_TURN_PARAM_NAME)->valuedouble;
    if (pRobot->appAutoDrive->isActive()) {
        uint8_t adf = 1;
        xQueueSend(pRobot->appAutoDrive->xQueue_In, &adf, 10);
    }
    sendDriveCmd(power, heading, turn);
    return ESP_OK;
}

static esp_err_t process_auto_mode() {
    sendDriveCmd(0, 0, 0);
    sendAutoDriveToggleCmmand();
    return ESP_OK;
}

static esp_err_t process_request(cJSON *root) {
    char *cmd = cJSON_GetObjectItem(root, CONFIG_CMD_PARAM_NAME)->valuestring;

    if (strcmp(cmd, CONFIG_MOVE_COMMAND) == 0) {
        return process_move_request(root);
    } else if (strcmp(cmd, CONFIG_AUTO_DRIVE_COMMAND) == 0) {
        return process_auto_mode();
    }
    return ESP_OK;
}

/* An HTTP GET handler */
static esp_err_t home_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, (const char *) DOCROOT, DOCROOT_LEN);
    return ESP_OK;
}

static esp_err_t state_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, HTTPD_TYPE_JSON);
    drive_command_t cmd = pRobot->appDrive->getCmd();

    cJSON *root = cJSON_CreateObject();
    cJSON *readings = NULL;

    cJSON_AddNumberToObject(root, "proximity", pRobot->appAutoDrive->tofArray->proximity);
    cJSON_AddBoolToObject(root, "autodrive", pRobot->appAutoDrive->isActive());

    cJSON *driveCmd = cJSON_AddObjectToObject(root, "drive_command");
    cJSON_AddNumberToObject(driveCmd, "power", cmd.power);
    cJSON_AddNumberToObject(driveCmd, "heading", cmd.heading);
    cJSON_AddNumberToObject(driveCmd, "turn", cmd.turn);

    readings = cJSON_AddArrayToObject(root, "readings");
    for (int i = 0; i < pRobot->appAutoDrive->tofArray->size; i++) {
        cJSON *reading = cJSON_CreateObject();
        cJSON_AddNumberToObject(reading, "angle", pRobot->appAutoDrive->tofArray->sensors[i].angle);
        cJSON_AddNumberToObject(reading, "distances", pRobot->appAutoDrive->tofArray->sensors[i].distance);
        cJSON_AddItemToArray(readings, reading);
    }
    const char *resp = cJSON_Print(root);
    httpd_resp_sendstr(req, resp);
    free((void *) resp);
    cJSON_Delete(root);
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

        //httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
#ifdef CONFIG_ESP_ROVER_DEBUG
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
#endif
    }

    cJSON *root = cJSON_Parse(buf);
    ESP_ERROR_CHECK(process_request(root));
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    cJSON *r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "status", "ok");
    const char *resp = cJSON_Print(r);
    httpd_resp_sendstr(req, resp);
    free((void *) resp);
    cJSON_Delete(r);

    return ESP_OK;
}

httpd_handle_t start_webserver(Robot *robot) {

    pRobot = robot;

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));


    httpd_uri_t home_get_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = home_get_handler,
            .user_ctx  = NULL
    };

    httpd_uri_t api_get_state_uri = {
            .uri       = "/api/state",
            .method    = HTTP_GET,
            .handler   = state_get_handler,
            .user_ctx  = NULL
    };

    httpd_uri_t api_post_uri = {
            .uri       = "/api",
            .method    = HTTP_POST,
            .handler   = api_post_handler,
            .user_ctx  = NULL
    };


    // Start the httpd server-**-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &home_get_uri);
        httpd_register_uri_handler(server, &api_post_uri);
        httpd_register_uri_handler(server, &api_get_state_uri);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server) {
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    httpd_handle_t *server = (httpd_handle_t *) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data) {
    httpd_handle_t *server = (httpd_handle_t *) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver(pRobot);
    }
}
