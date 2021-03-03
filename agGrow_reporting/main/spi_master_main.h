/*
 * spi_master_main.h
 *
 *  Created on: Mar 2, 2021
 *      Author: popad
 */
#pragma once
#include <stdint.h>
#include "esp_err.h"
#ifndef MAIN_SPI_MASTER_MAIN_H_
#define MAIN_SPI_MASTER_MAIN_H_

void begin_displaying();

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd);

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len);

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t);

uint32_t lcd_get_id(spi_device_handle_t spi);
//Initialize the display
void lcd_init(spi_device_handle_t spi);

/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
//static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata);


//static void send_line_finish(spi_device_handle_t spi);


//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
//static void display_pretty_colors(spi_device_handle_t spi);


#endif /* MAIN_SPI_MASTER_MAIN_H_ */
