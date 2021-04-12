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

#include "touch_pad.h"
#include "sdkconfig.h"
#include "includes/vl53l0x.h"
#include "i2c_main.h"
#include "lcd_ssd_driver.h"


sensorValues activeSensors;
ReportData sendA;
ReportData sendB;

static void lidar_timer_callback(void* arg)
{
	uint16_t distance;
    distance = vl53l0x_readRangeSingleMillimeters(activeSensors.lidarOne);
    ReportData *ptr_temp = &sendA;

    ptr_temp->byte0 = (distance>>8) & 0x00FF;
    ptr_temp->byte1 = (distance & 0x00FF);
    setDataForRead(&sendA, 0x01);
    printf("Distance from LIDAR Sensor: %d mm\n", distance);
}
static void touchSensor_timer_callback(void* arg)
{
	scan_touch ptr_inp;
	//scan_touch *ptr_temp = &ptr_inp;
	checkTouch(&ptr_inp);

	//nt len = snprintf(NULL, 0, "%f", amount);
	//char *result = (char *)malloc(len + 1);
	//snprintf(result, len + 1, "%f", amount);
	// do stuff with result
	//printf(result);
	//free(result);
	if(ptr_inp.detect && updateActivePage() == 0)
	{
		touchInterface(ptr_inp.y);
	}else if(updateActivePage() == 1)
	{
		checkSetting(ptr_inp.y);
	}else if(updateActivePage() == 8)
	{
		checkNew(ptr_inp.y);
	}else{
		updateSetting(ptr_inp.y);
	}
}
static void humSensors_timer_callback(void* arg)
{
	updateEnvSens(&activeSensors, 1);
	ReportData *ptr_temp = &sendB;
	sensorValues *ptr_temp_sens = &activeSensors;
	float hum = (float)(ptr_temp_sens->humidity);
	uint32_t humi = (uint32_t)(*(uint32_t*)&hum);

	ptr_temp->byte0 = (uint8_t)(humi>>24);
	ptr_temp->byte1 = (uint8_t)(humi>>16) & 0x00FF;
	ptr_temp->byte2 = (uint8_t)(humi>>8) & 0x00FF;
	ptr_temp->byte3 = (uint8_t)(humi) & 0x00FF;
	setDataForRead(&sendB, 0x00);
	printf("Relative Humidity: %f \n", ptr_temp_sens->humidity);
}
static void tempSensors_timer_callback(void* arg)
{
	updateEnvSens(&activeSensors, 1);
	ReportData *ptr_temp = &sendB;
	sensorValues *ptr_temp_sens = &activeSensors;

	float temp = (float)(ptr_temp_sens->temp);
	uint32_t tempi = (uint32_t) temp;
	ptr_temp->byte4 = (uint8_t)(tempi>>24);
	ptr_temp->byte5 = (uint8_t)(tempi>>16) & 0x00FF;
	ptr_temp->byte6 = (uint8_t)(tempi>>8) & 0x00FF;
	ptr_temp->byte7 = (uint8_t)(tempi & 0x00FF);
	setDataForRead(&sendB, 0x00);
	printf("Ambient Temperature: %f degrees Celsius\n", ptr_temp_sens->temp);

}
static void luxSensors_timer_callback(void* arg)
{
	updateEnvSens(&activeSensors, 2);
	ReportData *ptr_temp = &sendB;
	sensorValues *ptr_temp_sens = &activeSensors;
	float lux = (float)(ptr_temp_sens->lux);
	uint32_t luxi = (uint32_t)(*(uint32_t*)&lux);
	ptr_temp->byte8 = (uint8_t)(luxi>>24);
	ptr_temp->byte9 = (uint8_t)((luxi>>16) & 0x00FF);
	ptr_temp->byte10 = (uint8_t)((luxi>>8) & 0x00FF);
	ptr_temp->byte11 = (uint8_t)(luxi & 0x00FF);
	setDataForRead(&sendB, 0x00);
	printf("Ambient Illuminance: %f lux\n", ptr_temp_sens->lux);
}
static void gpsSensors_timer_callback(void* arg)
{
	updateEnvSens(&activeSensors, 3);
	ReportData *ptr_temp = &sendA;
	sensorValues *ptr_temp_sens = &activeSensors;
	uint32_t longi = (uint32_t)(*(uint32_t*)&(ptr_temp_sens->longitude));
	uint32_t lati = (uint32_t)(*(uint32_t*)&(ptr_temp_sens->latitude));

	ptr_temp->byte2 = (uint8_t)(lati>>24);
	ptr_temp->byte3 = (uint8_t)(lati>>16) & 0x00FF;
	ptr_temp->byte4 = (uint8_t)(lati>>8) & 0x00FF;
	ptr_temp->byte5 = (uint8_t)(lati) & 0x00FF;
	ptr_temp->byte6 = (uint8_t)(longi>>24);
	ptr_temp->byte7 = (uint8_t)(longi>>16) & 0x00FF;
	ptr_temp->byte8 = (uint8_t)(longi>>8) & 0x00FF;
	ptr_temp->byte9 = (uint8_t)(longi) & 0x00FF;

	setDataForRead(&sendA, 0x01);
	printf("GPS Location: Latitude %f, Longitude %f\n", ptr_temp_sens->latitude, ptr_temp_sens->longitude);
}
void app_main(void)
{
    const esp_timer_create_args_t lidar_timer_args = {
            .callback = &lidar_timer_callback,
            .name = "lidarTimer"
    };
    esp_timer_handle_t lidar_timer;
    ESP_ERROR_CHECK(esp_timer_create(&lidar_timer_args, &lidar_timer));

    const esp_timer_create_args_t humSensors_timer_args = {
            .callback = &humSensors_timer_callback,
            .name = "humSensors"
    };
    esp_timer_handle_t humSensors_timer;
    ESP_ERROR_CHECK(esp_timer_create(&humSensors_timer_args, &humSensors_timer));

    const esp_timer_create_args_t tempSensors_timer_args = {
            .callback = &tempSensors_timer_callback,
            .name = "tempSensors"
    };
    esp_timer_handle_t tempSensors_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tempSensors_timer_args, &tempSensors_timer));

    const esp_timer_create_args_t luxSensors_timer_args = {
            .callback = &luxSensors_timer_callback,
            .name = "luxSensors"
    };
    esp_timer_handle_t luxSensors_timer;
    ESP_ERROR_CHECK(esp_timer_create(&luxSensors_timer_args, &luxSensors_timer));

    const esp_timer_create_args_t gpsSensors_timer_args = {
            .callback = &gpsSensors_timer_callback,
            .name = "gpsSensors"
    };
    esp_timer_handle_t gpsSensors_timer;
    ESP_ERROR_CHECK(esp_timer_create(&gpsSensors_timer_args, &gpsSensors_timer));

    const esp_timer_create_args_t touchSensor_timer_args = {
            .callback = &touchSensor_timer_callback,
            .name = "touchSensor"
    };
    esp_timer_handle_t touchSensor_timer;
    ESP_ERROR_CHECK(esp_timer_create(&touchSensor_timer_args, &touchSensor_timer));

    begin_displaying();

    //Ready the Gatt Server for read event
    gatts_server_init();
    printf("BLE GATTS Server Running\n");
    //Starting the sensors
/*    i2c_main_init();
    activeSensors.lidarOne = vl53l0x_config(1, 25, 26, -1, 0x29, 1);
	vl53l0x_init(activeSensors.lidarOne);
*/
	//Start the 50Hz Lidar polling
	//ESP_ERROR_CHECK(esp_timer_start_periodic(lidar_timer, 20000));

	//Uncomment this line once gnss, humidity, temp, and lux are connected or else application may crash every 3 seconds
	//ESP_ERROR_CHECK(esp_timer_start_periodic(humSensors_timer, 3000000));
	ESP_ERROR_CHECK(esp_timer_start_periodic(touchSensor_timer, 200000));
	//ESP_ERROR_CHECK(esp_timer_start_periodic(tempSensors_timer, 3000000));
	//ESP_ERROR_CHECK(esp_timer_start_periodic(luxSensors_timer, 3000000));
	//ESP_ERROR_CHECK(esp_timer_start_periodic(gpsSensors_timer, 3000000));
}
