#include "D:/VISHNUPRIYA_WORKSPACE-1/SDK52_DeviceDownload/nRF5_SDK_17.1.0_ddde560/modules/nrfx/hal/nrf_gpio.h"
#include "D:/VISHNUPRIYA_WORKSPACE-1/SDK52_DeviceDownload/nRF5_SDK_17.1.0_ddde560/components/libraries/delay/nrf_delay.h"
#include <stdio.h>
#include <zephyr/kernel.h> 
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include"sensor.h"

typedef uint8_t ScratchPad[9];
DS18B20::DS18B20() {
    // define pin
	#define DS_PIN 26

	// Commands
	#define STARTCONVO      0x44
	#define READSCRATCH     0xBE
	#define WRITESCRATCH    0x4E
	
	// Scratchpad locations
	#define TEMP_LSB        0
	#define TEMP_MSB        1
	
	// Device resolution
	#define TEMP_9_BIT  0x1F //  9 bit
	#define TEMP_10_BIT 0x3F // 10 bit
	#define TEMP_11_BIT 0x5F // 11 bit
	#define TEMP_12_BIT 0x7F // 12 bit

	
}
 
DS18B20::~DS18B20() {
    // define pin
	#define DS_PIN 26

	// Commands
	#define STARTCONVO      0x44
	#define READSCRATCH     0xBE
	#define WRITESCRATCH    0x4E
	
	// Scratchpad locations
	#define TEMP_LSB        0
	#define TEMP_MSB        1
	
	// Device resolution
	#define TEMP_9_BIT  0x1F //  9 bit
	#define TEMP_10_BIT 0x3F // 10 bit
	#define TEMP_11_BIT 0x5F // 11 bit
	#define TEMP_12_BIT 0x7F // 12 bit
	//DS18B20();
}
 
void DS18B20::sendBit(char bit) {
    // Implementation for sending a bit

    
	nrf_gpio_cfg_output(DS_PIN); // Set DS pin as output
    nrf_gpio_pin_clear(DS_PIN); // Clear DS pin (send 0)

    nrf_delay_us(5);

    if(bit==1)// If sending 1 bit, set DS_PIN high (send 1)
    {
        nrf_gpio_pin_set(DS_PIN);
    }
    nrf_delay_us(100);
    nrf_gpio_pin_set(DS_PIN);// Set DS pin high to end transaction
}
 
void DS18B20::sendByte(char data) {
    // Implementation for sending a byte
    /*convering the Hexadecimal string to Binary and sending bit by bit to sendBit Function */
	    unsigned char i;
		unsigned char x;
		for(i=0;i<8;i++)
		{
		x = data>>i;// Shift the data byte
		x &= 0x01; // Extract the last bit
		sendBit(x);// Send the extracted bit
		}
		nrf_delay_us(100);// Delay after sending a byte
}

 
 
unsigned char DS18B20::reset() {
    // Implementation for resetting the sensor
	unsigned char presence;

	nrf_gpio_cfg_output(DS_PIN);
	nrf_gpio_pin_clear(DS_PIN);
	
	nrf_delay_us(500);
	nrf_gpio_pin_set(DS_PIN);
	
	nrf_gpio_cfg_input(DS_PIN,NRF_GPIO_PIN_NOPULL); 
	nrf_delay_us(30);


    if(nrf_gpio_pin_read(DS_PIN) == 0)
    {
        presence = 1;
    }
    else
    {
        presence = 0;
    }

    nrf_delay_us(470);

    if(nrf_gpio_pin_read(DS_PIN) == 1)
    {
        presence = 1;
    }
    else
    {
        presence = 0;
    }

  return presence;
	
}

uint8_t DS18B20::OneWire_read_bit()
{
    uint8_t r;//variable to store bit read
    nrf_gpio_cfg_output(DS_PIN);
    nrf_gpio_pin_clear(DS_PIN);
    nrf_delay_us(3);
    nrf_gpio_cfg_input(DS_PIN,NRF_GPIO_PIN_NOPULL);
    nrf_delay_us(10);

    r =nrf_gpio_pin_read(DS_PIN);// Read pin state and store into variable r
    nrf_delay_us(53);
    return r;
}

uint8_t DS18B20::OneWire_read()
{
    uint8_t bitMask;//mask for each bit 
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( OneWire_read_bit()) 
    {
        r |= bitMask;
    }
    }
    return r;

}
void DS18B20::ds18b20_readScratchPad(uint8_t *scratchPad, uint8_t fields)
{
    reset();
    sendByte(0xCC);
    sendByte(READSCRATCH);

    for(uint8_t i=0; i < fields; i++)
    {
        scratchPad[i] = OneWire_read();
    }
    reset();
}

void DS18B20::ds18b20_requestTemperatures(void)
{
    reset();
    sendByte(0xCC);
    sendByte(STARTCONVO);
}


float DS18B20::getTemperature() {
    // Implementation for getting the temperature
	    ds18b20_requestTemperatures();
		//unsigned char check;
	
		ScratchPad scratchPad;
		ds18b20_readScratchPad(scratchPad, 2);
		int16_t rawTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 8) | scratchPad[TEMP_LSB];
		float temp = (0.0625 * rawTemperature)/3.5;
	
		return temp;
}

void DS18B20::ds18b20_setResolution(uint8_t resolution)
{
    /*write 3 bytes (TH, TL, CONFIG REGISTER)*/
    reset();// Reset DS18B20 to read data
    sendByte(0xCC);// skip ROM command
    sendByte(WRITESCRATCH);
    // two dummy values for LOW & HIGH TRIGGER ALARM REGISTERS
    sendByte(0);
    sendByte(100);
    //set temperature resolution
    switch (resolution)
    {
        case 12:
            sendByte(TEMP_12_BIT);//0x7F -> 0001 1111
            break;

        case 11:
            sendByte(TEMP_11_BIT);//0X5F -> 0101 1111
            break;

        case 10:
            sendByte(TEMP_10_BIT);//0X3F -> 0011 1111
            break;

        case 9:
        default:
            sendByte(TEMP_9_BIT);//0X1F -> 0001 1111
            break;
    }
    reset();
}

