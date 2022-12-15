//
// Created by Kevin Williams on 12/14/22.
//

#include "app_proximity.h"
#include "rplidar_cmd.h"

static const char * TAG = "AppProximity";

//static sl_lidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
//static bool _is_previous_capsuledataRdy = false;
static lidar_info_t info;
static lidar_health_t health;

AppProximity::AppProximity(QueueHandle_t xQueueIn, ProximityFrame * frame): xQueue_In(xQueueIn), pFrame(frame) {}

static void task(AppProximity *app) {

    ESPLidar * lidar = new ESPLidar();

    lidar->init();
    lidar->getInfo(info);
    lidar->getHealth(health);
    lidar->startExpressScan();

    while(true) {
        sl_lidar_response_ultra_capsule_measurement_nodes_t nodes;
        uint8_t * packet = (uint8_t*) &nodes;

        if(lidar->readExpressMeasurement(packet) == ESP_OK) {
            if((nodes.s_checksum_1 >> 4) == 0xA && (nodes.s_checksum_2 >> 4) == 0x5) {
                sl_lidar_response_measurement_node_hq_t hq[132];
                size_t nodecount = 1;
                lidar->ultraCapsuleToNormal(nodes, hq, nodecount);

                proximity_reading_t readings[nodecount];

                for(int i=0; i < nodecount; i++) {
                    float angle = lidar->getAngle(hq[i]);
                    sl_u32 distance = lidar->getDistanceQ2(hq[i]);
                        //ESP_LOGI(TAG, "HQ %f, %d %d", angle, distance, hq->quality);
                        readings[i] = {.angle = angle, .distance = distance};
                    app->pFrame->applyReading({.angle = angle, .distance = distance, .quality = hq->quality});
                }
            }
        }

        ESP_LOGI(TAG, "Proximity: %d, %d", app->pFrame->status(), app->pFrame->status_in_progress());

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void AppProximity::run() {
    xTaskCreatePinnedToCore((TaskFunction_t)task, "Proximity", 5 * 1024, this, 5, NULL, 1);
}
