/************************************************************************
* This program is free software; you can redistribute it and/or modify *
* it under the terms of the GNU License V2. *
* This program is distributed in the hope that it will be useful, *
* but WITHOUT ANY WARRANTY; without even the implied warranty of *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the *
* GNU General Public License, version 2 for more details *
* *
* Bare bones ADXL345 i2c example for Arduino 1.0 *
* by Jens C Brynildsen <http://www.flashgamer.com> *
* This version is not reliant of any external lib *
* (Adapted for Arduino 1.0 from http://code.google.com/p/adxl345driver)*
* *
* Demonstrates use of ADXL345 (using the Sparkfun ADXL345 breakout) *
* with i2c communication. Datasheet: *
* http://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf *
* If you need more advanced features such as freefall and tap *
* detection, check out: *
* https://github.com/jenschr/Arduino-libraries *
***********************************************************************/

// Cabling for i2c using Sparkfun breakout with an Arduino Uno / Duemilanove:
// Arduino <-> Breakout board
// Gnd - GND
// 3.3v - VCC
// 3.3v - CS
// Analog 4 - SDA
// Analog 5 - SLC

// Cabling for i2c using Sparkfun breakout with an Arduino Mega / Mega ADK:
// Arduino <-> Breakout board
// Gnd - GND
// 3.3v - VCC
// 3.3v - CS
// 20 - SDA
// 21 - SLC
#include "EEPROM.h"

#define DEVICE (0x53) // Device address as specified in data sheet

byte _buff[6];

char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

int ax;
int ay;
int az;



void getAnglesADXL345(sensorType *sensor)
{
	readAccel();

	// 1G=256, angle=asin(ACC/1G). But ACC/1G is approxmiately equal to angle in radians if less than +-30 degrees.
	// To convert to degrees (ACC/1G)*180/pi
	// 180/(256*pi)=.22381
	#define ACC_TO_DEGREES_ADXL	0.22381
	sensor->x_accel = (ax - sensor->x_accel_offset)*ACC_TO_DEGREES_ADXL;
	sensor->y_accel = (ay - sensor->y_accel_offset)*ACC_TO_DEGREES_ADXL;
	sensor->z_accel = (az - sensor->z_accel_offset)*ACC_TO_DEGREES_ADXL;
}


void calibrateADXL345(sensorType *sensor)
{
	long x_total = 0;
	long y_total = 0;
	long z_total = 0;

    delay(1000);
    Serial.println("calibrateADXL345");
	
    for (int i=0; i<400; i++)
    {
		readAccel();
		Serial.print(" AX: ");
		Serial.print(ax);
		Serial.print(" AY: ");
		Serial.print(ay);
		Serial.print(" AZ: ");
		Serial.println(az);
		x_total+=ax;
		y_total+=ay;
		z_total+=az;
		delay(20);
	}
	sensor->x_accel_offset = x_total/400;
	sensor->y_accel_offset = y_total/400;
	sensor->z_accel_offset = z_total/400;

	Serial.print("X Offset   ");
	Serial.print(sensor->x_accel_offset);
	Serial.print("Y Offset   ");
	Serial.print(sensor->y_accel_offset);
	Serial.print("Z Offset   ");
	Serial.println(sensor->z_accel_offset);
}

void initADXL345(sensorType *sensor, boolean calibrate)
{      
	Serial.println("initADXL345");
	//Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
	writeTo(DEVICE, DATA_FORMAT, 0x01);
	//Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
	writeTo(DEVICE, POWER_CTL, 0x08);

	if (calibrate)
	{
		calibrateADXL345(sensor);
		// write calibration values to EEPROM
		writeIntToEEPROM(EEPROM_BOARD_X_ACCEL_OFFSET_ADDR, sensor->x_accel_offset);
		writeIntToEEPROM(EEPROM_BOARD_Y_ACCEL_OFFSET_ADDR, sensor->y_accel_offset);
		writeIntToEEPROM(EEPROM_BOARD_Z_ACCEL_OFFSET_ADDR, sensor->z_accel_offset);
	}
	else
	{
		// load calibration values from EEPROM
		sensor->x_accel_offset = readIntFromEEPROM(EEPROM_BOARD_X_ACCEL_OFFSET_ADDR);
		sensor->y_accel_offset = readIntFromEEPROM(EEPROM_BOARD_Y_ACCEL_OFFSET_ADDR);
		sensor->z_accel_offset = readIntFromEEPROM(EEPROM_BOARD_Z_ACCEL_OFFSET_ADDR);
	}
}

void readAccel() 
{
	uint8_t howManyBytesToRead = 6;
	readFrom(DEVICE, DATAX0, _buff, howManyBytesToRead); //read the acceleration data from the ADXL345

	// each axis reading comes in 10 bit resolution, ie 2 bytes. Least Significat Byte first!!
	// thus we are converting both bytes in to one int
	ax = (((int)_buff[1]) << 8) | _buff[0];
	ay = (((int)_buff[3]) << 8) | _buff[2];
	az = (((int)_buff[5]) << 8) | _buff[4];
}

int writeTo(uint8_t address, uint8_t reg, uint8_t data)
{
	int n;

	I2c.setSpeed(0);
	I2c.timeOut(10);
	n = I2c.write(address, reg, data); 
	return (-n);
}

// Reads num bytes starting from address register on device in to _buff array
int readFrom(uint8_t address, uint8_t start, uint8_t *buffer, uint8_t size) 
{
	int n;

	I2c.setSpeed(0);	// slow 100KHz
	I2c.timeOut(10);	// wait 10ms
	n = I2c.read(address, start, size, buffer);
	return (-n);
}
