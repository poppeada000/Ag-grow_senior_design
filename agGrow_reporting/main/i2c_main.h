/*
 * i2c_main.h
 *
 *  Created on: Mar 4, 2021
 *      Author: popad
 */


#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"
typedef struct{
	vl53l0x_t *lidarOne;
	uint8_t *hum_temp_data;
	uint8_t lux_cmd_code;
	uint8_t *lux_data;
	double lux;
	double humidity;
	double temp;
	float longitude;
	float latitude;
}sensorValues;

void updateEnvSens(sensorValues* ptr_inp);
void i2c_main_init();
