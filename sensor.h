#ifndef _SENSOR_H_
#define _SENSOR_H_

class DS18B20{
	
	public:
		DS18B20();
		~DS18B20();
		void sendBit(char bit);
		void sendByte(char data);
		unsigned char reset();
		uint8_t OneWire_read_bit();
		uint8_t OneWire_read();
		void ds18b20_readScratchPad(uint8_t *scratchPad, uint8_t fields);
		void ds18b20_requestTemperatures();
		float getTemperature();
		void ds18b20_setResolution(uint8_t resolution);
};

	#endif
		
