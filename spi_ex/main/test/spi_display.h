/*
 * spi_master_main.h
 *
 *  Created on: Mar 2, 2021
 *      Author: popad
 */
#pragma once
#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"
typedef uint8_t u8;
typedef uint16_t u16;

//*****************************************************************************
//
// Various internal SD2119 registers name labels
//
//*****************************************************************************
#define SSD2119_DEVICE_CODE_READ_REG  0x00
#define SSD2119_OSC_START_REG         0x00
#define SSD2119_OUTPUT_CTRL_REG       0x01
#define SSD2119_LCD_DRIVE_AC_CTRL_REG 0x02
#define SSD2119_PWR_CTRL_1_REG        0x03
#define SSD2119_DISPLAY_CTRL_REG      0x07
#define SSD2119_FRAME_CYCLE_CTRL_REG  0x0B
#define SSD2119_PWR_CTRL_2_REG        0x0C
#define SSD2119_PWR_CTRL_3_REG        0x0D
#define SSD2119_PWR_CTRL_4_REG        0x0E
#define SSD2119_GATE_SCAN_START_REG   0x0F
#define SSD2119_SLEEP_MODE_1_REG      0x10
#define SSD2119_ENTRY_MODE_REG        0x11
#define SSD2119_SLEEP_MODE_2_REG      0x12
#define SSD2119_GEN_IF_CTRL_REG       0x15
#define SSD2119_PWR_CTRL_5_REG        0x1E
#define SSD2119_RAM_DATA_REG          0x22
#define SSD2119_FRAME_FREQ_REG        0x25
#define SSD2119_ANALOG_SET_REG        0x26
#define SSD2119_VCOM_OTP_1_REG        0x28
#define SSD2119_VCOM_OTP_2_REG        0x29
#define SSD2119_GAMMA_CTRL_1_REG      0x30
#define SSD2119_GAMMA_CTRL_2_REG      0x31
#define SSD2119_GAMMA_CTRL_3_REG      0x32
#define SSD2119_GAMMA_CTRL_4_REG      0x33
#define SSD2119_GAMMA_CTRL_5_REG      0x34
#define SSD2119_GAMMA_CTRL_6_REG      0x35
#define SSD2119_GAMMA_CTRL_7_REG      0x36
#define SSD2119_GAMMA_CTRL_8_REG      0x37
#define SSD2119_GAMMA_CTRL_9_REG      0x3A
#define SSD2119_GAMMA_CTRL_10_REG     0x3B
#define SSD2119_V_RAM_POS_REG         0x44
#define SSD2119_H_RAM_START_REG       0x45
#define SSD2119_H_RAM_END_REG         0x46
#define SSD2119_X_RAM_ADDR_REG        0x4E
#define SSD2119_Y_RAM_ADDR_REG        0x4F

#define ENTRY_MODE_DEFAULT 0x6830
#define MAKE_ENTRY_MODE(x) ((ENTRY_MODE_DEFAULT & 0xFF00) | (x))

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
//void display_pretty_colors(spi_device_handle_t spi);
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


