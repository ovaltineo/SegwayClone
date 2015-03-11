#include <EEPROM.h>

void writeIntToEEPROM(int address, int value)
{
	byte low = value & 0xFF;
	byte high = value >> 8;
	
	EEPROM.write(address, low);
	EEPROM.write(address+1, high);
}

int readIntFromEEPROM(int address)
{
	byte low = EEPROM.read(address);
	byte high = EEPROM.read(address+1);
	int value = (high << 8) + low;
	return value;
}