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
#include "lcd_ssd_driver.h"


sensorValues activeSensors;
ReportData sendA;
ReportData sendB;
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

    begin_displaying();

    //Ready the Gatt Server for read event
    gatts_server_init();
    printf("BLE GATTS Server Running\n");
    //Starting the sensors
    //i2c_main_init();
    //activeSensors.lidarOne = vl53l0x_config(1, 25, 26, -1, 0x29, 1);
	//vl53l0x_init(activeSensors.lidarOne);

	//Start the 50Hz Lidar polling
	//ESP_ERROR_CHECK(esp_timer_start_periodic(lidar_timer, 20000));

	//Uncomment this line once gnss, humidity, temp, and lux are connected or else application may crash every 3 seconds
	//ESP_ERROR_CHECK(esp_timer_start_periodic(envSensors_timer, 3000000));
}

static void lidar_timer_callback(void* arg)
{
	uint16_t distance;
    distance = vl53l0x_readRangeSingleMillimeters(activeSensors.lidarOne);
    ReportData *ptr_temp = &sendA;
    ptr_temp->msbValue = (distance>>8) & 0x00FF;
    ptr_temp->lsbValue = (distance & 0x00FF);
    setDataForRead(&sendA, 0x01);
    printf("Distance from LIDAR Sensor: %d mm\n", distance);
}

static void envSensors_timer_callback(void* arg)
{
	printf("3 second TIMER CALL\n");
	updateEnvSens(&activeSensors);
	ReportData *ptr_temp = &sendB;
	sensorValues *ptr_temp_sens = &activeSensors;
	ptr_temp->msbValue = ptr_temp_sens->humidity;
	ptr_temp->bValue = ptr_temp_sens->temp;
	ptr_temp->lsbValue = ptr_temp_sens->lux;
	ptr_temp->longValue = ptr_temp_sens->longitude;
	ptr_temp->latValue = ptr_temp_sens->latitude;

	setDataForRead(&sendB, 0x00);
	printf("Relative Humidity: %f \n", ptr_temp_sens->humidity);
	printf("Ambient Temperature: %f degrees Celsius\n", ptr_temp_sens->temp);
	printf("Ambient Illuminance: %f lux\n", ptr_temp_sens->lux);
	printf("GPS Location: Latitude %f, Longitude %f\n", ptr_temp_sens->latitude, ptr_temp_sens->longitude);
}
