#include "EEPROM.h"

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define L3G4200D_I2C_ADDRESS 105 //I2C address of the L3G4200D

int x;
int y;
int z;

void getAnglesL3G4200D(sensorType *sensor)
{
	getGyroValues();

	// check if gyro value is between rest min and max, if so, set it equal to offset so it becomes 0 when subtracted
	if ((x >= sensor->x_gyro_offset_min) && (x <= sensor->x_gyro_offset_max))
		x = sensor->x_gyro_offset;
	if ((y >= sensor->y_gyro_offset_min) && (y <= sensor->y_gyro_offset_max))
		y = sensor->y_gyro_offset;
	if ((z >= sensor->z_gyro_offset_min) && (z <= sensor->z_gyro_offset_max))
		z = sensor->z_gyro_offset;

	// as per datasheet, resolution at +-250dps sensitivity is 8.75 mdps/digit or .00875 dps/digit 
	#define DEG_PER_SECOND_PER_DIGIT  .00875
	sensor->x_gyro = (x - sensor->x_gyro_offset)*DEG_PER_SECOND_PER_DIGIT; 
	sensor->y_gyro = (y - sensor->y_gyro_offset)*DEG_PER_SECOND_PER_DIGIT; 
	sensor->z_gyro = (z - sensor->z_gyro_offset)*DEG_PER_SECOND_PER_DIGIT; 

}


void initL3G4200D(sensorType *sensor, boolean calibrate)
{
	Serial.println("Setting up L3G4200D");
	setupL3G4200D(250); // Configure L3G4200  - 250, 500 or 2000 deg/sec

	delay(1500); //wait for the sensor to be ready 
	if (calibrate)
	{
		calibrateL3G4200D(sensor);
		// write calibration values to EEPROM
		writeIntToEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_ADDR, sensor->x_gyro_offset);
		writeIntToEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_MIN_ADDR, sensor->x_gyro_offset_min);
		writeIntToEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_MAX_ADDR, sensor->x_gyro_offset_max);

		writeIntToEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_ADDR, sensor->y_gyro_offset);
		writeIntToEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_MIN_ADDR, sensor->y_gyro_offset_min);
		writeIntToEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_MAX_ADDR, sensor->y_gyro_offset_max);

		writeIntToEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_ADDR, sensor->z_gyro_offset);
		writeIntToEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_MIN_ADDR, sensor->z_gyro_offset_min);
		writeIntToEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_MAX_ADDR, sensor->z_gyro_offset_max);
	}
	else
	{
		// load calibration values from EEPROM
		sensor->x_gyro_offset = readIntFromEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_ADDR);
		sensor->x_gyro_offset_min = readIntFromEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_MIN_ADDR);
		sensor->x_gyro_offset_max = readIntFromEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_MAX_ADDR);
		
		sensor->y_gyro_offset = readIntFromEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_ADDR);
		sensor->y_gyro_offset_min = readIntFromEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_MIN_ADDR);
		sensor->y_gyro_offset_max = readIntFromEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_MAX_ADDR);

		sensor->z_gyro_offset = readIntFromEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_ADDR);
		sensor->z_gyro_offset_min = readIntFromEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_MIN_ADDR);
		sensor->z_gyro_offset_max = readIntFromEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_MAX_ADDR);
	}
}


void calibrateL3G4200D(sensorType *sensor)
{
	long x_total;
	long y_total;
	long z_total;

	sensor->x_gyro_offset_min=16384;
	sensor->x_gyro_offset_max=-16384;
	sensor->y_gyro_offset_min=16384;
	sensor->y_gyro_offset_max=-16384;
	sensor->z_gyro_offset_min=16384;
	sensor->z_gyro_offset_max=-16384;
  
  for(int i = 0; i < 400; i++)
  {
    getGyroValues();

//	Serial.print("Gyro   ");
//	Serial.println(y);
	
    if(x > sensor->x_gyro_offset_max)
    {
      sensor->x_gyro_offset_max = x;
    }
    else if(x < sensor->x_gyro_offset_min)
    {
      sensor->x_gyro_offset_min = x;
    }
	
    if(y > sensor->y_gyro_offset_max)
    {
      sensor->y_gyro_offset_max = y;
    }
    else if(y < sensor->y_gyro_offset_min)
    {
      sensor->y_gyro_offset_min = y;
    }
	
    if(z > sensor->z_gyro_offset_max)
    {
      sensor->z_gyro_offset_max = z;
    }
    else if(z < sensor->z_gyro_offset_min)
    {
      sensor->z_gyro_offset_min = z;
    }
	x_total+=x;
	y_total+=y;
	z_total+=z;
	delay(20);
  }
  sensor->x_gyro_offset = x_total/400;
  sensor->y_gyro_offset = y_total/400;
  sensor->z_gyro_offset = z_total/400;
  
}

void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_I2C_ADDRESS, 0x29);
  byte xLSB = readRegister(L3G4200D_I2C_ADDRESS, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_I2C_ADDRESS, 0x2B);
  byte yLSB = readRegister(L3G4200D_I2C_ADDRESS, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_I2C_ADDRESS, 0x2D);
  byte zLSB = readRegister(L3G4200D_I2C_ADDRESS, 0x2C);
  z = ((zMSB << 8) | zLSB);
  
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable 400Hz, x, y, z and disable power down:
  writeRegister(L3G4200D_I2C_ADDRESS, CTRL_REG1, 0b10001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_I2C_ADDRESS, CTRL_REG2, 0b00000000);

  // disable all interrupts
  writeRegister(L3G4200D_I2C_ADDRESS, CTRL_REG3, 0b00000000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_I2C_ADDRESS, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_I2C_ADDRESS, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_I2C_ADDRESS, CTRL_REG4, 0b00110000);
  }

  // Enable low-pass filter
  writeRegister(L3G4200D_I2C_ADDRESS, CTRL_REG5, 0b00000010);
}

int writeRegister(uint8_t address, uint8_t reg, uint8_t data)
{
	int n;

	I2c.setSpeed(0);
	I2c.timeOut(10);
	n = I2c.write(address, reg, data); 
	return (-n);
}

int readRegister(uint8_t address, uint8_t reg){

    uint8_t v;
	
	int n;

	I2c.setSpeed(0);	// slow 100KHz
	I2c.timeOut(10);	// wait 10ms
	n = I2c.read(address, reg, 1, &v);
	if (n == 0)
		return v;
	else
		return (-n);
}
