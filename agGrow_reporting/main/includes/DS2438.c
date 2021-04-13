
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "includes/owb.h"
#include "includes/owb_rmt.h"
#include "includes/DS2438.h"

#include "driver/rmt.h"
#include "driver/gpio.h"

#define MAX_DEVICES          (1)

/**
 * Sets the DS2438 to read Vad or Vdd
 *
 * portnum  the port number of the port being used for the
 *          1-Wire Network.
 */
void SetupAtoD(const OneWireBus * owb)
{
	bool reset_present;

	// reset and presence pulse
	owb_reset(owb, &reset_present);

	// skip ROM
	owb_write_byte(owb, 0xCC);

	// issue write SP 00h command (4Eh00h)
	owb_write_byte(owb, 0x4E);
	owb_write_byte(owb, 0x00);

	// set AD bit active (0001000 binary)
	owb_write_byte(owb, 0x08);

	// reset and presence pulse
	owb_reset(owb, &reset_present);

	// skip ROM
	owb_write_byte(owb, 0xCC);

	// issue read SP 00h command (BEh00h)
	owb_write_byte(owb, 0xBE);
	owb_write_byte(owb, 0x00);

	// read scratchpad data and CRC (9 data bytes)
	uint8_t * data = (uint8_t *)malloc(9*sizeof(u_int8_t));
	owb_read_bytes(owb, data, 9);

	// reset and presence pulse
	owb_reset(owb, &reset_present);

	// skip ROM
	owb_write_byte(owb, 0xCC);

	// issue copy CP 00h command (48h00h)
	owb_write_byte(owb, 0x48);
	owb_write_byte(owb, 0x00);

	// Read slots, DS2438 returns a "1" when copy SP is complete
	uint8_t * busy_byte = 0;
	do
	{
		owb_read_byte(owb, busy_byte);
	}while(busy_byte==0);

	// reset and presence pulse, done
	owb_reset(owb, &reset_present);

}


/**
 * Read the Vdd voltage value from the DS2438
 *
 * portnum  the port number of the port being used for the
 *          1-Wire Network.
 *
 * @return the floating point value of Vad or Vdd
 */
float ReadAtoD(const OneWireBus * owb)
{
	bool reset_present;

	// reset and presence pulse
	owb_reset(owb, &reset_present);

	// skip ROM
	owb_write_byte(owb, 0xCC);

	// issue convert voltage vommand, read slots (B4h)
	owb_write_byte(owb, 0xB4);
	uint8_t * busy_byte = 0;
	do
	{
		owb_read_byte(owb, busy_byte);
	}while(busy_byte==0);

	// reset and presence pulse
	owb_reset(owb, &reset_present);

	// skip ROM
	owb_write_byte(owb, 0xCC);

	// issue recall memory page 00h command (B8h00h)
	owb_write_byte(owb, 0xB8);
	owb_write_byte(owb, 0x00);

	// reset and presence pulse
	owb_reset(owb, &reset_present);

	// skip ROM
	owb_write_byte(owb, 0xCC);

	// Issue Read SP 00h command (BEh00h)
	owb_write_byte(owb, 0xBE);
	owb_write_byte(owb, 0x00);

	// Read scratchpad data and CRC containing 9 data bytes
	uint8_t * device_data = (uint8_t *)malloc(9*sizeof(u_int8_t));
	owb_read_bytes(owb, device_data, 9);

	// reset and presence pulse, done
	owb_reset(owb, &reset_present);


	// extract voltage data from page and convert to float
	uint16_t raw  = ((device_data[4] << 8) & (uint16_t)(0x03)) | device_data[3];
	const float voltage = raw / 100.0f;   // V
	return voltage;
}


float oneWireMain()
{
	// Startup delay to allow time for bus capacitance to charge
    //vTaskDelay(2000 / portTICK_RATE_MS);

    //initialize one-wire drivers
    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, 13, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    printf("Find devices:\n");
    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    printf("Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

    // In this example, if a single device is present, then the ROM code is probably
    // not very interesting, so just print it out. If there are multiple devices,
    // then it may be useful to check that a specific device is present.

    if (num_devices == 1)
    {
        // For a single device only:
        OneWireBus_ROMCode rom_code;
        owb_status status = owb_read_rom(owb, &rom_code);
        if (status == OWB_STATUS_OK)
        {
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
            printf("Single device %s present\n", rom_code_s);
        }
        else
        {
            printf("An error occurred reading ROM code: %d", status);
        }
    }

    //setup configuration/status register of DS2438
    SetupAtoD(owb);

    //read the voltage data
    float Vdd = ReadAtoD(owb);
    printf("Battery voltage is %f V", Vdd);
    return Vdd;
}
