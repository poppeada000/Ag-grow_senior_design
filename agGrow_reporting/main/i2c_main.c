/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
//necessary external libraries for sensors
#include "vl53l0x.h"
#include "includes/mjd.h"
#include "includes/mjd_neom8n.h"
#include "i2c_main.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */




SemaphoreHandle_t print_mux = NULL;

// ***CUSTOM SENSOR FUNCTIONS***
/* Humidity-Temperature Sensor Read
 *
 * ___________________________________________
 * | start | slave_addr + wr_bit +ack | stop |
 * --------|--------------------------|------|
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_humidity_temp_read(i2c_port_t i2c_num, uint8_t *hum_temp_data)
{
	// Initialize values
	uint8_t slave_address = 0x27;
	size_t size = sizeof(hum_temp_data);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Initiate measurement by sending write bit
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_address << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	// Wait for 100ms for measurement to complete (normal measurement cycle is 36.65ms/each for humidity and temperature)
    vTaskDelay(100 / portTICK_RATE_MS);

    // Read collected data
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_address << 1 | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, hum_temp_data, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, hum_temp_data + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Lux Sensor Read: DATA IS LSB FIRST
 *
 * ___________________________________________
 * | start | slave_addr + wr_bit +ack | command code +ack |
 * --------|--------------------------|-------------------|
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read 1 bytes + ack   | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_lux_read(i2c_port_t i2c_num, uint8_t *cmd_code, uint8_t *lux_data)
{
	// Initialize values
	uint8_t slave_address = 0x10;
	size_t size = sizeof(lux_data);

	// Send write command and write command code
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_address << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, cmd_code[0], ACK_CHECK_EN);
	//i2c_master_stop(cmd);
    //esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    //i2c_cmd_link_delete(cmd);

    // Read data
    //cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_address << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, lux_data, size - 1, ACK_VAL);
    i2c_master_read_byte(cmd, lux_data + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = 1;//I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 25;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = 26;
    conf.scl_pullup_en = 0;
    conf.master.clk_speed = 400000;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

//VEML7700 initialization
static esp_err_t i2c_lux_init(i2c_port_t i2c_num, uint8_t *cmd_code, uint8_t *lux_wr)
{
	// Initialize values
	uint8_t slave_address = 0x10;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Send write command, command code, and configuration bytes
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_address << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, cmd_code[0], ACK_CHECK_EN);
	i2c_master_write(cmd, lux_wr, sizeof(lux_wr), ACK_CHECK_EN);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


void i2c_main_init()
{
	// Startup delay to allow time for bus capacitance to charge
    //vTaskDelay(1500 / portTICK_RATE_MS);
    /*
	//variable initialization
	uint8_t *cmd_code = (uint8_t *)malloc(1*sizeof(uint8_t));
	uint8_t *lux_wr = (uint8_t *)malloc(2*sizeof(uint8_t));
	lux_wr[0] = 0x00;
	lux_wr[1] = 0x00;
	cmd_code[0] = 0x00;
	*/

    //GNSS initialization
	//mjd_neom8n_config_t neom8n_config;
	//mjd_neom8n_data_t neom8n_data;
    //neom8n_config.uart_port = UART_NUM_1;
    //neom8n_config.uart_rx_gpio_num = 17;
    //neom8n_config.uart_tx_gpio_num = 16;
    //neom8n_config.do_cold_start = false;
    //mjd_log_wakeup_details();
    //mjd_log_chip_info();
    //mjd_log_time();
    //mjd_log_memory_statistics();
    //mjd_neom8n_init(&neom8n_config);
    //vTaskDelay(5000 / portTICK_RATE_MS);

	//GPIO initialization
    print_mux = xSemaphoreCreateMutex();
    //ESP_ERROR_CHECK(i2c_master_init());
    i2c_master_init();

    //ESP_ERROR_CHECK(gnss_init());

    //Test Read Functions
    /*
    uint8_t *hum_temp_data = (uint8_t *)malloc(4*sizeof(uint8_t));
    cmd_code[0] = 0x04;
    uint8_t *lux_data = (uint8_t *)malloc(2*sizeof(uint8_t));
    uint16_t mm_distance;
     */

    //while (true)
    //{
    	//i2c_humidity_temp_read(1, hum_temp_data);
    	//i2c_lux_read(I2C_MASTER_NUM, cmd_code, lux_data);
    	//Refresh time for lux sensor is 600ms with default settings
        //vTaskDelay(600 / portTICK_RATE_MS);
    	//mm_distance = vl53l0x_readRangeSingleMillimeters(v);
        //mjd_neom8n_read(&neom8n_config, &neom8n_data);

        //vTaskDelay(5000 / portTICK_RATE_MS);

    	/*
    	//Interpretation of data
    	// Humidity: in first two bytes
    	int hum_read = (hum_temp_data[0] << 8) + hum_temp_data[1];
    	double humidity = (hum_read/16382.0) * 100.0;
    	//printf("%d %d\n",hum_temp_data[0],hum_temp_data[1]);
    	printf("Relative Humidity: %f\n", humidity);

    	// Temperature: located in next two bytes, padded by two trailing bits
    	int temp_read = (hum_temp_data[2] << 6) + (hum_temp_data[3] >> 2);
    	double temp = 165*(temp_read/16382.0) - 40;
    	//printf("%d %d\n",hum_temp_data[2],hum_temp_data[3]);
    	printf("Ambient Temperature: %f degrees Celsius\n", temp);

    	//Lux: value returned is a count, must be multiplied by resolution of lx/count for lux value
    	//default settings use 0.0576 lx/ct
    	//LSByte is lux_data[0] and MSByte is lux_data[1]
    	double lux = (lux_data[0] + (lux_data[1] << 8)) * 0.0576;
    	printf("Ambient Illuminance: %f lux\n", lux);
    	*/
    	//Lidar:
    	//printf("Distance from LIDAR Sensor: %d mm\n", mm_distance);

    	//GNSS: Latitude and Longitude
    	//printf("GPS Location: Latitude %f°, Longitude %f°\n", neom8n_data.latitude, neom8n_data.longitude);
    //}
}
