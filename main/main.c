/**
 * @file main.cpp
 * @author Kevin Williams (kevywilly@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// https://github.com/espressif/esp-idf/tree/d6d8324ad9b957f85ecb323ec0db12c4d03a93c4/examples/protocols/http_server/simple


static const char *TAG = "esprover";

#include "webserver.h"

/*
#define CALIBRATE 0

static void run(void *arg)
{
   

    //config_pins(pin);
    int delay_ms = 2000;
   
    robot_config(&rb_conf);

    if(CALIBRATE) {
        calibrate_motors();
        while(true) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }

    for(int h=0; h <= 360; h+=45) {
        printf("\n---- DRIVE WITH HEADING: %d ---\n", h);
        robot_drive(&rb_conf, h, 0.8);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    printf("\n---- ROTATE CW ---\n" );
    robot_rotate(&rb_conf, ROTATE_CW, 0.8);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    printf("\n---- ROTATE CW ---\n" );
    robot_rotate(&rb_conf, ROTATE_CCW, 0.8);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    printf("\n---- STOP ---\n" );
    robot_stop(&rb_conf);
    
    
    while (true) {
        //std::cout << "Hello Rover"<< i << '\n';
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    }
    
}
*/
/*
void app_main(void)
{

    xTaskCreate(run, "run esp rover", 4096, NULL, 10, NULL);

}
*/


void app_main(void)
{
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* Start the server for the first time */
    //xTaskCreate(run, "run esp rover", 4096, NULL, 10, NULL);
    server = start_webserver();
}