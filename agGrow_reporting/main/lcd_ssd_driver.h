/*
 * lcd_ssd_driver.h
 *
 *  Created on: Mar 29, 2021
 *      Author: popad
 */

#ifndef MAIN_INCLUDES_LCD_SSD_DRIVER_H_
#define MAIN_INCLUDES_LCD_SSD_DRIVER_H_




//============================================================================
// lcd.h: Adapted from the lcdwiki.com examples.
//============================================================================
#include "stdlib.h"
#include "driver/spi_master.h"
// shorthand notation for 8-bit and 16-bit unsigned integers
typedef uint8_t u8;
typedef uint16_t u16;

// The LCD device structure definition.
//
typedef struct
{
    u16 width;
    u16 height;
    u16 id;
    u8  dir;
    u16  wramcmd;
    u16  setxcmd;
    u16  setycmd;
} lcd_dev_t;

typedef struct
{
	spi_device_handle_t spi_bus_call;
} spi_bus;

// The LCD device.
// This will be initialized by LCD_direction() so that the
// width and height will be appropriate for the rotation.
// The setxcmd and setycmd will be set so that cursor selection
// is defined properly for the rotation.
extern lcd_dev_t lcddev;

// Rotation:
// 0: rotate 0
// 1: rotate: 90
// 2: rotate: 180
// 3: rotate 270
#define USE_HORIZONTAL       0

// The dimensions of the display.
#define LCD_W 240
#define LCD_H 320
#define MAPPED_X(x, y)  (319 - (x))
#define MAPPED_Y(x, y)  (239 - (y))
// Some popular colors
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define BRED        0XF81F
#define GRED        0XFFE0
#define GBLUE       0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN       0XBC40
#define BRRED       0XFC07
#define GRAY        0X8430
#define DARKBLUE    0X01CF
#define LIGHTBLUE   0X7D7C
#define GRAYBLUE    0X5458
#define LIGHTGREEN  0X841F
#define LIGHTGRAY   0XEF5B
#define LGRAY       0XC618
#define LGRAYBLUE   0XA651
#define LBBLUE      0X2B12

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
void LCD_Clear(spi_device_handle_t spi, u16 Color);
void LCD_DrawPoint(spi_device_handle_t spi, u16 x,u16 y,u16 c);
void LCD_DrawLine(spi_device_handle_t spi, u16 x1, u16 y1, u16 x2, u16 y2, u16 c);
void LCD_DrawRectangle(spi_device_handle_t spi, u16 x1, u16 y1, u16 x2, u16 y2, u16 c);
void LCD_DrawFillRectangle(spi_device_handle_t spi, u16 x1, u16 y1, u16 x2, u16 y2, u16 c);
void LCD_Circle(spi_device_handle_t spi, u16 xc, u16 yc, u16 r, u16 fill, u16 c);
void LCD_DrawTriangle(spi_device_handle_t spi, u16 x0,u16 y0, u16 x1,u16 y1, u16 x2,u16 y2, u16 c);
void LCD_DrawFillTriangle(spi_device_handle_t spi, u16 x0,u16 y0, u16 x1,u16 y1, u16 x2,u16 y2, u16 c);
void LCD_DrawChar(spi_device_handle_t spi, u16 x,u16 y,u16 fc, u16 bc, u8 num, u8 size, u8 mode);
void LCD_DrawString(spi_device_handle_t spi, u16 x,u16 y, u16 fc, u16 bg, const char *p, u8 size, u8 mode);

//===========================================================================
// C Picture data structure.
//===========================================================================
typedef const struct {
    unsigned int   width;
    unsigned int   height;
    unsigned int   bytes_per_pixel; // 2:RGB16, 3:RGB, 4:RGBA
    unsigned char  pixel_data[0]; // variable length array
} Picture;

typedef struct{
	uint8_t page;
	uint8_t loc;
	char ID0[3];
} pageID;

void Main_menu();
void userSeletion();
uint8_t updateActivePage();
void touchInterface(uint16_t yCord);
#endif /* MAIN_INCLUDES_LCD_SSD_DRIVER_H_ */
