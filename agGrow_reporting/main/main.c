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
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_master_main.h"
#include "pretty_effect.h"
#include "gatts_server.h"

/*
esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}
*/
void app_main(void)
{
	gatts_server_init();
    begin_displaying();

}

