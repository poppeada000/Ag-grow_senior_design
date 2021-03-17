/*
 * main.c
 *
 *  Created on: Mar 2, 2021
 *      Author: popad
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "gatts_server.h"

#include "sdkconfig.h"
#include "includes/vl53l0x.h"
#include "i2c_main.h"

sensorValues activeSensors;
ReportData sendToRead;
static void lidar_timer_callback(void* arg);
static void envSensors_timer_callback(void* arg);

void app_main(void)
{
    const esp_timer_create_args_t lidar_timer_args = {
            .callback = &lidar_timer_callback,
            .name = "lidarTimer"
    };
    esp_timer_handle_t lidar_timer;
    ESP_ERROR_CHECK(esp_timer_create(&lidar_timer_args, &lidar_timer));

    const esp_timer_create_args_t envSensors_timer_args = {
            .callback = &envSensors_timer_callback,
            .name = "environmentSensors"
    };
    esp_timer_handle_t envSensors_timer;
    ESP_ERROR_CHECK(esp_timer_create(&envSensors_timer_args, &envSensors_timer));
    //Timers setup complete -- Not yet running
    //Ready the Gatt Server for read event
    gatts_server_init();
    printf("BLE GATTS Server Running\n");
    //Starting the sensors
    i2c_main_init();
    activeSensors.lidarOne = vl53l0x_config(1, 25, 26, -1, 0x29, 1);
	vl53l0x_init(activeSensors.lidarOne);

	//Start the 50Hz Lidar polling
	ESP_ERROR_CHECK(esp_timer_start_periodic(lidar_timer, 20000));

	//Uncomment this line once gnss, humidity, temp, and lux are connected or else application may crash every 3 seconds
	//ESP_ERROR_CHECK(esp_timer_start_periodic(envSensors_timer, 3000000));
}

static void lidar_timer_callback(void* arg)
{
	uint16_t distance;
    distance = vl53l0x_readRangeSingleMillimeters(activeSensors.lidarOne);
    sendToRead.value = distance;
    printf("Distance from LIDAR Sensor: %d mm\n", distance);
}

static void envSensors_timer_callback(void* arg)
{
	updateEnvSens(&activeSensors);
	printf("Relative Humidity: %f\n", activeSensors.humidity);
	printf("Ambient Temperature: %f degrees Celsius\n", activeSensors.temp);
	printf("Ambient Illuminance: %f lux\n", activeSensors.lux);
	printf("GPS Location: Latitude %f°, Longitude %f°\n", activeSensors.latitude, activeSensors.longitude);
}
