/*
 * touch_pad.c
 *
 *  Created on: Apr 6, 2021
 *      Author: popad
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <driver/adc.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "touch_pad.h"

#define neg_X 15//swapped
#define pos_X 14
#define neg_Y 32
#define pos_Y 33//swapped
#define DEFAULT_VREF    1100
scan_touch scanner;
//static esp_adc_cal_characteristics_t *adc_chars;
//static const adc_unit_t unit = ADC_UNIT_1;
static const adc_channel_t channel = ADC_CHANNEL_4;
static const adc_channel_t channel2 = ADC_CHANNEL_6;
//static const adc_atten_t atten = ADC_ATTEN_0db ;
/*
 * ADC1 8 channels 32-39
 * ADC2 10 Channels 0,2,4,12-15,25-27
 *
 */
void scanX()
{
	int numSamp = 10;
	uint32_t reading = 0;
    gpio_set_direction(pos_X, GPIO_MODE_OUTPUT);
    gpio_set_direction(neg_X, GPIO_MODE_OUTPUT);
    gpio_set_direction(neg_Y, GPIO_MODE_INPUT);

    //vTaskDelay(20 / portTICK_RATE_MS);
    gpio_set_level(pos_X, 0);
    gpio_set_level(neg_X, 1);
    //gpio_set_level(neg_Y, 0);

    //adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    //esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    //vTaskDelay(10 / portTICK_RATE_MS);
    vTaskDelay(10 / portTICK_RATE_MS);
    //adc2_config_width(ADC_WIDTH_BIT_12);
    adc2_config_channel_atten( (adc2_channel_t)channel, ADC_ATTEN_11db );
    //reading = adc2_get_raw(ADC2_CHANNEL_6);
    for(int i = 0; i<numSamp; i++)
    {
    	int raw;
        adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
        reading += raw;
    }
    printf("Raw: %d\n", reading/numSamp);
    scanner.x = reading/numSamp;
    //uint32_t voltage = esp_adc_cal_raw_to_voltage(reading/numSamp, adc_chars);

}

void scanY()
{
	int numSamp = 64;
	uint32_t reading = 0;
    gpio_set_direction(pos_Y, GPIO_MODE_OUTPUT);
    gpio_set_direction(neg_Y, GPIO_MODE_OUTPUT);
    gpio_set_direction(neg_X, GPIO_MODE_INPUT);
    gpio_set_level(pos_Y, 0);
    gpio_set_level(neg_Y, 1);

    vTaskDelay(10 / portTICK_RATE_MS);
    //adc2_config_width(ADC_WIDTH_BIT_12);
    adc2_config_channel_atten( (adc2_channel_t)channel2, ADC_ATTEN_DB_11 );
    //reading = adc2_get_raw(ADC2_CHANNEL_6);
    for(int i = 0; i<numSamp; i++)
    {
    	int raw;
        adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
        reading += raw;
    }
    printf("Raw: %d\n", reading/numSamp);
    scanner.y = reading/numSamp;
}

void checkTouch(scan_touch *ptr_inp)
{
	scan_touch *ptr_temp = &scanner;
	scanY();
	if(ptr_inp->y > 4000 && ptr_temp->y<4000)
	{
		ptr_inp->detect = 1;
	}else
	{
		ptr_inp->detect = 0;
	}
	ptr_inp->y = ptr_temp->y;
	//printf("Relative Humidity: %u \n", (unsigned int)(ptr_temp->x));
}

