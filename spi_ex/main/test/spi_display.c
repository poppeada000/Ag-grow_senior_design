/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_display.h"
#include "includes/pretty_effect.h"
#include "gatts_server.h"


/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

#define PIN_NUM_MISO 27
#define PIN_NUM_SDI 16
#define PIN_NUM_SCL  19
#define PIN_NUM_SCS   17

#define PIN_NUM_SDC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 10

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum {
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[]={
    /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
    {SSD2119_SLEEP_MODE_1_REG, {0x00, 0x01}, 2},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    {SSD2119_PWR_CTRL_5_REG, {0x00, 0xB2}, 2},
    /* Porch Setting */
    {SSD2119_VCOM_OTP_1_REG, {0x00, 0x06}, 2},
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    {SSD2119_OSC_START_REG, {0x00, 0x01}, 2},
    /* VCOM Setting, VCOM=1.175V */
    {SSD2119_OUTPUT_CTRL_REG, {0x30, 0xEF}, 2},
    /* LCM Control, XOR: BGR, MX, MH */
    {SSD2119_LCD_DRIVE_AC_CTRL_REG, {0x06, 0x00}, 2},
    {SSD2119_SLEEP_MODE_1_REG, {0x00, 0x00}, 2},
    {SSD2119_ENTRY_MODE_REG, {0x68, 0x30}, 2},
    {SSD2119_SLEEP_MODE_2_REG, {0x09, 0x99}, 2},
    {SSD2119_ANALOG_SET_REG, {0x38, 0x00}, 2},
    {SSD2119_DISPLAY_CTRL_REG, {0x00, 0x33}, 2},
    {SSD2119_PWR_CTRL_2_REG, {0x00, 0x05}, 2},
    {SSD2119_GAMMA_CTRL_1_REG, {0x00, 0x00}, 2},
    {SSD2119_GAMMA_CTRL_2_REG, {0x03, 0x03}, 2},
    {SSD2119_GAMMA_CTRL_3_REG, {0x04, 0x07}, 2},
    {SSD2119_GAMMA_CTRL_4_REG, {0x03, 0x01}, 2},
    {SSD2119_GAMMA_CTRL_5_REG, {0x03, 0x01}, 2},
    {SSD2119_GAMMA_CTRL_6_REG, {0x04, 0x03}, 2},
    {SSD2119_GAMMA_CTRL_7_REG, {0x07, 0x07}, 2},
    {SSD2119_GAMMA_CTRL_8_REG, {0x04, 0x00}, 2},
    {SSD2119_GAMMA_CTRL_9_REG, {0x0A, 0x00}, 2},
    {SSD2119_GAMMA_CTRL_10_REG, {0x10, 0x00}, 2},

    {SSD2119_PWR_CTRL_3_REG, {0x00, 0x0A}, 2},
    {SSD2119_PWR_CTRL_4_REG, {0x2E, 0x00}, 2},
    {SSD2119_V_RAM_POS_REG, {0xEF, 0x00}, 2},
    {SSD2119_H_RAM_START_REG, {0x00, 0x00}, 2},
    {SSD2119_H_RAM_END_REG, {0x01, 0x3F}, 2},
    {SSD2119_X_RAM_ADDR_REG, {0x00, 0x00}, 2},
    {SSD2119_Y_RAM_ADDR_REG, {0x00, 0x00}, 2},
};


/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_SDC, dc);
}

uint32_t lcd_get_id(spi_device_handle_t spi)
{
    //get_id cmd
    lcd_cmd(spi, 0x04);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    return *(uint32_t*)t.rx_data;
}

//Initialize the display
void lcd_init(spi_device_handle_t spi)
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_SDC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //detect LCD type
    uint32_t lcd_id = lcd_get_id(spi);
    int lcd_detected_type = 0;
    int lcd_type;

    printf("LCD ID: %08X\n", lcd_id);
    if ( lcd_id == 0 ) {
        //zero, ili
        lcd_detected_type = LCD_TYPE_ST;
        printf("ILI9341 detected.\n");
    } else {
        // none-zero, ST
        lcd_detected_type = LCD_TYPE_ST;
        printf("ST7789V detected.\n");
    }

#ifdef CONFIG_LCD_TYPE_AUTO
    lcd_type = lcd_detected_type;
#elif defined( CONFIG_LCD_TYPE_ST7789V )
    printf("kconfig: force CONFIG_LCD_TYPE_ST7789V.\n");
    lcd_type = LCD_TYPE_ST;
#elif defined( CONFIG_LCD_TYPE_ILI9341 )
    printf("kconfig: force CONFIG_LCD_TYPE_ILI9341.\n");
    lcd_type = LCD_TYPE_ILI;
#endif
    if ( lcd_type == LCD_TYPE_ST ) {
        printf("LCD ST7789V initialization.\n");
        lcd_init_cmds = st_init_cmds;
    } else {
        printf("LCD ILI9341 initialization.\n");
        //lcd_init_cmds = ili_init_cmds;
    }

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 0);
}


/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[8];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x=0; x<8; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }

    trans[0].tx_data[0]= SSD2119_H_RAM_START_REG;           //Column Address Set
    trans[1].tx_data[0]=0;              //Start Col High
    trans[1].tx_data[1]=0;              //Start Col Low

    trans[2].tx_data[0]= SSD2119_H_RAM_END_REG;           //Column Address Set
    trans[3].tx_data[0]=(320)>>8;       //End Col High
    trans[3].tx_data[1]=(320)&0xff;     //End Col Low

    trans[4].tx_data[0]=SSD2119_V_RAM_POS_REG;           //Page address set
    //trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[5].tx_data[0]=ypos&0xff;      //start page low

    //trans[3].tx_data[2]=(ypos+PARALLEL_LINES)>>8;    //end page high
    trans[5].tx_data[1]=(ypos+PARALLEL_LINES)&0xff;  //end page low

    trans[6].tx_data[0]= SSD2119_RAM_DATA_REG;           //memory write
    trans[7].tx_buffer=linedata;        //finally send the line data
    trans[7].length=320*2*8*PARALLEL_LINES;          //Data length, in bits
    trans[7].flags=0; //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x=0; x<8; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}


static void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}


//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
static void display_pretty_colors(spi_device_handle_t spi)
{
    uint16_t *lines[2];
    //Allocate memory for the pixel buffers
    for (int i=0; i<2; i++) {
        lines[i]=heap_caps_malloc(320*PARALLEL_LINES*sizeof(uint16_t), MALLOC_CAP_DMA);
        assert(lines[i]!=NULL);
        printf("Sheap.\n");
    }
    int frame=0;
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;
	frame++;
    for (int y=0; y<240; y+=PARALLEL_LINES) {
    	//Calculate a line.
    	pretty_effect_calc_lines(lines[calc_line], y, frame, PARALLEL_LINES);
            //Finish up the sending process of the previous line, if any
        if (sending_line!=-1) send_line_finish(spi);
            //Swap sending_line and calc_line
        sending_line = calc_line;
        calc_line = (calc_line==1)?0:1;
            //Send the line we currently calculated.
        send_lines(spi, y, lines[sending_line]);
        printf("line send\n");
            //The line set is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line set as long as we do not
            //touch line[sending_line]; the SPI sending process is still reading from that.
    }
}

void begin_displaying()
{
    esp_err_t ret;
    //gatts_server_init();
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_SDI,
        .sclk_io_num=PIN_NUM_SCL,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
#else
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
#endif
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_SCS,               //CS pin
        .queue_size=9,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    //Initialize the LCD
    lcd_init(spi);
    //Initialize the effect displayed
    ret=pretty_effect_init();
    ESP_ERROR_CHECK(ret);

    //Go do nice stuff.
    display_pretty_colors(spi);

}

