/*
 * gatts_server.h
 *
 *  Created on: Mar 2, 2021
 *      Author: popad
 */

#pragma once
#include <stdint.h>
#include "esp_err.h"
/*
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
*/
typedef struct{
	uint8_t header;
	uint8_t msbValue;
	uint8_t bValue;
	uint8_t lsbValue;
	uint8_t longValue;
	uint8_t latValue;
}ReportData;

void gatts_server_init();
void setDataForRead(ReportData* ptr_inp, uint8_t channel);
