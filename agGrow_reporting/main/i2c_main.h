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
	uint8_t *lux_cmd_code;
	uint8_t *lux_data;
	double lux;
	double humidity;
	double temp;
	float longitude;
	float latitude;
}sensorValues;

/*
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
*/
//static esp_err_t i2c_humidity_temp_read(i2c_port_t i2c_num, uint8_t *hum_temp_data);
//esp_err_t i2c_humidity_temp_read(i2c_port_t i2c_num, uint8_t *hum_temp_data);
//esp_err_t i2c_lux_read(i2c_port_t i2c_num, uint8_t *cmd_code, uint8_t *lux_data);
void updateEnvSens(sensorValues* ptr_inp);
void i2c_main_init();
//static esp_err_t i2c_humidity_temp_read(i2c_port_t i2c_num, uint8_t *hum_temp_data);
