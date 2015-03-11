// MPU-6050 Accelerometer + Gyro
// -----------------------------
//
// By arduino.cc user "Krodal".
// June 2012
// Open Source / Public Domain
// 
// Modified by "Ovaltineo"
// for use on DIY Segway clone
//
// Using Arduino 1.0.1
// It will not work with an older version,
// since Wire.endTransmission() uses a parameter
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-6000 and MPU-6050 Product Specification",
//     PS-MPU-6000A.pdf
//   - "MPU-6000 and MPU-6050 Register Map and Descriptions",
//     RM-MPU-6000A.pdf or RS-MPU-6000A.pdf
//   - "MPU-6000/MPU-6050 9-Axis Evaluation Board User Guide"
//     AN-MPU-6000EVB.pdf
//
// The accuracy is 16-bits.
//
// Temperature sensor from -40 to +85 degrees Celsius
//   340 per degrees, -512 at 35 degrees.
//
// At power-up, all registers are zero, except these two:
//      Register 0x6B (PWR_MGMT_2) = 0x40  (I read zero).
//      Register 0x75 (WHO_AM_I)   = 0x68.
//

#if defined(MPU6050_X2_STEERING) || defined(MPU6050_X1_STEERING) || defined(POT_STEERING)
#include "MPU6050.h"
#include "EEPROM.h"

static sensorType MPU6050_BOARD;

void initBoardSensor(boolean calibrate)
{
	MPU6050_BOARD.address = MPU6050_I2C_ADDRESS;
	initMPU6050(&MPU6050_BOARD, calibrate);
	if (calibrate)
	{
		// write calibration values to EEPROM
		writeIntToEEPROM(EEPROM_BOARD_X_ACCEL_OFFSET_ADDR, MPU6050_BOARD.x_accel_offset);
		writeIntToEEPROM(EEPROM_BOARD_Y_ACCEL_OFFSET_ADDR, MPU6050_BOARD.y_accel_offset);
		writeIntToEEPROM(EEPROM_BOARD_Z_ACCEL_OFFSET_ADDR, MPU6050_BOARD.z_accel_offset);
		
		writeIntToEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_ADDR, MPU6050_BOARD.x_gyro_offset);
		writeIntToEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_MIN_ADDR, MPU6050_BOARD.x_gyro_offset_min);
		writeIntToEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_MAX_ADDR, MPU6050_BOARD.x_gyro_offset_max);
		
		writeIntToEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_ADDR, MPU6050_BOARD.y_gyro_offset);
		writeIntToEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_MIN_ADDR, MPU6050_BOARD.y_gyro_offset_min);
		writeIntToEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_MAX_ADDR, MPU6050_BOARD.y_gyro_offset_max);
		
		writeIntToEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_ADDR, MPU6050_BOARD.z_gyro_offset);
		writeIntToEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_MIN_ADDR, MPU6050_BOARD.z_gyro_offset_min);
		writeIntToEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_MAX_ADDR, MPU6050_BOARD.z_gyro_offset_max);
	}
	else
	{
		// load calibration values from EEPROM
		MPU6050_BOARD.x_accel_offset = readIntFromEEPROM(EEPROM_BOARD_X_ACCEL_OFFSET_ADDR);
		MPU6050_BOARD.y_accel_offset = readIntFromEEPROM(EEPROM_BOARD_Y_ACCEL_OFFSET_ADDR);
		MPU6050_BOARD.z_accel_offset = readIntFromEEPROM(EEPROM_BOARD_Z_ACCEL_OFFSET_ADDR);
		
		MPU6050_BOARD.x_gyro_offset = readIntFromEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_ADDR);
		MPU6050_BOARD.x_gyro_offset_min = readIntFromEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_MIN_ADDR);
		MPU6050_BOARD.x_gyro_offset_max = readIntFromEEPROM(EEPROM_BOARD_X_GYRO_OFFSET_MAX_ADDR);
		
		MPU6050_BOARD.y_gyro_offset = readIntFromEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_ADDR);
		MPU6050_BOARD.y_gyro_offset_min = readIntFromEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_MIN_ADDR);
		MPU6050_BOARD.y_gyro_offset_max = readIntFromEEPROM(EEPROM_BOARD_Y_GYRO_OFFSET_MAX_ADDR);
		
		MPU6050_BOARD.z_gyro_offset = readIntFromEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_ADDR);
		MPU6050_BOARD.z_gyro_offset_min = readIntFromEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_MIN_ADDR);
		MPU6050_BOARD.z_gyro_offset_max = readIntFromEEPROM(EEPROM_BOARD_Z_GYRO_OFFSET_MAX_ADDR);
	}
}

float getBoardAccel(void)
{
	getAnglesMPU6050((sensorType *)&MPU6050_BOARD);	
	return -MPU6050_BOARD.x_accel;
}

float getBoardGyro(void)
{
	return MPU6050_BOARD.y_gyro;
}

float getBoardYawGyro(void)
{
	return MPU6050_BOARD.z_gyro;
}

float getBoardBankGyro(void)
{
	return -MPU6050_BOARD.x_gyro;
}

float getBoardBankAccel(void)
{
	return -MPU6050_BOARD.y_accel;
}

#ifdef MPU6050_X2_STEERING
static sensorType MPU6050_STEER;
void initSteerSensor(boolean calibrate)
{
	MPU6050_STEER.address = MPU6050_ALT_I2C_ADDRESS;
	initMPU6050(&MPU6050_STEER, calibrate);
	if (calibrate)
	{
		// write calibration values to EEPROM
		writeIntToEEPROM(EEPROM_STEER_X_ACCEL_OFFSET_ADDR, MPU6050_STEER.x_accel_offset);
		writeIntToEEPROM(EEPROM_STEER_Y_ACCEL_OFFSET_ADDR, MPU6050_STEER.y_accel_offset);
		writeIntToEEPROM(EEPROM_STEER_Z_ACCEL_OFFSET_ADDR, MPU6050_STEER.z_accel_offset);
		
		writeIntToEEPROM(EEPROM_STEER_X_GYRO_OFFSET_ADDR, MPU6050_STEER.x_gyro_offset);
		writeIntToEEPROM(EEPROM_STEER_X_GYRO_OFFSET_MIN_ADDR, MPU6050_STEER.x_gyro_offset_min);
		writeIntToEEPROM(EEPROM_STEER_X_GYRO_OFFSET_MAX_ADDR, MPU6050_STEER.x_gyro_offset_max);
		
		writeIntToEEPROM(EEPROM_STEER_Y_GYRO_OFFSET_ADDR, MPU6050_STEER.y_gyro_offset);
		writeIntToEEPROM(EEPROM_STEER_Y_GYRO_OFFSET_MIN_ADDR, MPU6050_STEER.y_gyro_offset_min);
		writeIntToEEPROM(EEPROM_STEER_Y_GYRO_OFFSET_MAX_ADDR, MPU6050_STEER.y_gyro_offset_max);
		
		writeIntToEEPROM(EEPROM_STEER_Z_GYRO_OFFSET_ADDR, MPU6050_STEER.z_gyro_offset);
		writeIntToEEPROM(EEPROM_STEER_Z_GYRO_OFFSET_MIN_ADDR, MPU6050_STEER.z_gyro_offset_min);
		writeIntToEEPROM(EEPROM_STEER_Z_GYRO_OFFSET_MAX_ADDR, MPU6050_STEER.z_gyro_offset_max);
	}
	else
	{
		// load calibration values from EEPROM
		MPU6050_STEER.x_accel_offset = readIntFromEEPROM(EEPROM_STEER_X_ACCEL_OFFSET_ADDR);
		MPU6050_STEER.y_accel_offset = readIntFromEEPROM(EEPROM_STEER_Y_ACCEL_OFFSET_ADDR);
		MPU6050_STEER.z_accel_offset = readIntFromEEPROM(EEPROM_STEER_Z_ACCEL_OFFSET_ADDR);

		MPU6050_STEER.x_gyro_offset = readIntFromEEPROM(EEPROM_STEER_X_GYRO_OFFSET_ADDR);
		MPU6050_STEER.x_gyro_offset_min = readIntFromEEPROM(EEPROM_STEER_X_GYRO_OFFSET_MIN_ADDR);
		MPU6050_STEER.x_gyro_offset_max = readIntFromEEPROM(EEPROM_STEER_X_GYRO_OFFSET_MAX_ADDR);

		MPU6050_STEER.y_gyro_offset = readIntFromEEPROM(EEPROM_STEER_Y_GYRO_OFFSET_ADDR);
		MPU6050_STEER.y_gyro_offset_min = readIntFromEEPROM(EEPROM_STEER_Y_GYRO_OFFSET_MIN_ADDR);
		MPU6050_STEER.y_gyro_offset_max = readIntFromEEPROM(EEPROM_STEER_Y_GYRO_OFFSET_MAX_ADDR);

		MPU6050_STEER.z_gyro_offset = readIntFromEEPROM(EEPROM_STEER_Z_GYRO_OFFSET_ADDR);
		MPU6050_STEER.z_gyro_offset_min = readIntFromEEPROM(EEPROM_STEER_Z_GYRO_OFFSET_MIN_ADDR);
		MPU6050_STEER.z_gyro_offset_max = readIntFromEEPROM(EEPROM_STEER_Z_GYRO_OFFSET_MAX_ADDR);
	}
}


#ifdef STEER_Z_GYRO
float getSteerAccel(void)
{
	getAnglesMPU6050((sensorType *)&MPU6050_STEER);	
	return MPU6050_STEER.y_accel;
}

float getSteerGyro(void)
{
	return MPU6050_STEER.z_gyro;
}
#else
float getSteerAccel(void)
{
	getAnglesMPU6050((sensorType *)&MPU6050_STEER);	
	return -MPU6050_STEER.y_accel;
}

float getSteerGyro(void)
{
	return -MPU6050_STEER.x_gyro;
}
#endif
#endif



#ifdef MPU6050_X1_STEERING
void initSteerSensor(boolean calibrate)
{
// do nothing because sensor is already initialized
}

float getSteerAccel(void)
{
	return -MPU6050_BOARD.y_accel;
}

float getSteerGyro(void)
{
	return MPU6050_BOARD.x_gyro;
}
#endif


void getAnglesMPU6050(sensorType *sensor)
{
	accel_t_gyro_union accel_t_gyro;

	readSampleMPU6050(sensor->address, (uint8_t *)&accel_t_gyro);

	// 1G=16384, angle=asin(ACC/1G). But ACC/1G is approxmiately equal to angle in radians if less than +-30 degrees.
	// To convert to degrees (ACC/1G)*180/pi
	// 180/(16384*pi)=.003497
	#define ACC_TO_DEGREES	0.003497
	sensor->x_accel = (accel_t_gyro.value.x_accel - sensor->x_accel_offset)*ACC_TO_DEGREES;	
	sensor->y_accel = (accel_t_gyro.value.y_accel - sensor->y_accel_offset)*ACC_TO_DEGREES;
	sensor->z_accel = (accel_t_gyro.value.z_accel - sensor->z_accel_offset - 16384)*ACC_TO_DEGREES;  // subtract 1G from z-axis
	
	// check if gyro value is between rest min and max, if so, set it equal to offset so it becomes 0 when subtracted
	if ((accel_t_gyro.value.y_gyro >= sensor->y_gyro_offset_min) && (accel_t_gyro.value.y_gyro <= sensor->y_gyro_offset_max))
		accel_t_gyro.value.y_gyro = sensor->y_gyro_offset;

	#define LSB_TO_DEG_PER_SECOND  131.0
	sensor->x_gyro = (accel_t_gyro.value.x_gyro - sensor->x_gyro_offset)/LSB_TO_DEG_PER_SECOND; 
	sensor->y_gyro = (accel_t_gyro.value.y_gyro - sensor->y_gyro_offset)/LSB_TO_DEG_PER_SECOND; 
	sensor->z_gyro = (accel_t_gyro.value.z_gyro - sensor->z_gyro_offset)/LSB_TO_DEG_PER_SECOND; 
}



void readSampleMPU6050(int address, uint8_t *accel_t_gyro_var)
{
	int error;
	accel_t_gyro_union *accel_t_gyro;

	error = MPU6050_read (address, MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro_var, sizeof(accel_t_gyro_union));
	if (error)
	{
		alarmArray[ALARM_I2C] = 1;
		Serial.print(F("I2C Error = "));
		Serial.println(error,DEC);
	}
	accel_t_gyro = (accel_t_gyro_union *)accel_t_gyro_var;

	// Swap all high and low bytes.
	// After this, the registers values are swapped,
	// so the structure name like x_accel_l does no
	// longer contain the lower byte.
	uint8_t swap;
	#define SWAP(x,y) swap = x; x = y; y = swap

	SWAP (accel_t_gyro->reg.x_accel_h, accel_t_gyro->reg.x_accel_l);
	SWAP (accel_t_gyro->reg.y_accel_h, accel_t_gyro->reg.y_accel_l);
	SWAP (accel_t_gyro->reg.z_accel_h, accel_t_gyro->reg.z_accel_l);
	SWAP (accel_t_gyro->reg.t_h, accel_t_gyro->reg.t_l);
	SWAP (accel_t_gyro->reg.x_gyro_h, accel_t_gyro->reg.x_gyro_l);
	SWAP (accel_t_gyro->reg.y_gyro_h, accel_t_gyro->reg.y_gyro_l);
	SWAP (accel_t_gyro->reg.z_gyro_h, accel_t_gyro->reg.z_gyro_l);
}


void calibrateMPU6050(sensorType *sensor)
{
	accel_t_gyro_union accel_t_gyro;
	long x_total = 0;
	long y_total = 0;
	long z_total = 0;
	long x_g_total = 0;
	long y_g_total = 0;
	long z_g_total = 0;

	sensor->x_gyro_offset_min=16384;
	sensor->x_gyro_offset_max=-16384;
	sensor->y_gyro_offset_min=16384;
	sensor->y_gyro_offset_max=-16384;
	sensor->z_gyro_offset_min=16384;
	sensor->z_gyro_offset_max=-16384;

	
    for (int i=0; i<400; i++)
    {
		readSampleMPU6050(sensor->address, (uint8_t *)&accel_t_gyro);
		Serial.print("Gyro   ");
		Serial.println(accel_t_gyro.value.y_gyro);
		if (accel_t_gyro.value.x_gyro < sensor->x_gyro_offset_min)
			sensor->x_gyro_offset_min = accel_t_gyro.value.x_gyro;
		else if (accel_t_gyro.value.x_gyro > sensor->x_gyro_offset_max)
			sensor->x_gyro_offset_max = accel_t_gyro.value.x_gyro;
			
		if (accel_t_gyro.value.y_gyro < sensor->y_gyro_offset_min)
			sensor->y_gyro_offset_min = accel_t_gyro.value.y_gyro;
		else if (accel_t_gyro.value.y_gyro > sensor->y_gyro_offset_max)
			sensor->y_gyro_offset_max = accel_t_gyro.value.y_gyro;

		if (accel_t_gyro.value.z_gyro < sensor->z_gyro_offset_min)
			sensor->z_gyro_offset_min = accel_t_gyro.value.z_gyro;
		else if (accel_t_gyro.value.z_gyro > sensor->z_gyro_offset_max)
			sensor->z_gyro_offset_max = accel_t_gyro.value.z_gyro;
		x_total+=accel_t_gyro.value.x_accel;
		y_total+=accel_t_gyro.value.y_accel;
		z_total+=accel_t_gyro.value.z_accel;
		x_g_total+=accel_t_gyro.value.x_gyro;
		y_g_total+=accel_t_gyro.value.y_gyro;
		z_g_total+=accel_t_gyro.value.z_gyro;
		delay(20);
	}
	sensor->x_accel_offset = x_total/400;
	sensor->y_accel_offset = y_total/400;
	sensor->z_accel_offset = z_total/400;
	sensor->x_gyro_offset = x_g_total/400;
	sensor->y_gyro_offset = y_g_total/400;
	sensor->z_gyro_offset = z_g_total/400;
	
	Serial.print("X Accel Offset   ");
	Serial.print(sensor->x_accel_offset);
	Serial.print("Y Gyro Offset   ");
	Serial.print(sensor->y_gyro_offset);
	Serial.print("Y Gyro Min   ");
	Serial.print(sensor->y_gyro_offset_min);
	Serial.print("Y Gyro Max   ");
	Serial.println(sensor->y_gyro_offset_max);
}

void initMPU6050(sensorType *sensor, boolean calibrate)
{      
  int error;
  uint8_t c;




  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (sensor->address, MPU6050_WHO_AM_I, &c, 1);
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
  error = MPU6050_read (sensor->address, MPU6050_PWR_MGMT_2, &c, 1);
  Serial.print(F("PWR_MGMT_2 : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);


  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (sensor->address, MPU6050_PWR_MGMT_1, 0);
  if (calibrate)
    calibrateMPU6050(sensor);
}


// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus.
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read.
// There is no function for a single byte.
//
int MPU6050_read(uint8_t address, uint8_t start, uint8_t *buffer, uint8_t size)
{
	int n;

	I2c.setSpeed(1);
	I2c.timeOut(MPU6050_I2C_WAIT);
	n = I2c.read(address, start, size, buffer);
	return (-n);
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(uint8_t address, uint8_t start, uint8_t *pData, uint8_t size)
{
	int n;

	I2c.setSpeed(0);
	I2c.timeOut(MPU6050_I2C_WAIT);
	n = I2c.write(address, start, pData, size);
	return (-n);
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(uint8_t address, uint8_t reg, uint8_t data)
{
	int n;

	I2c.setSpeed(0);
	I2c.timeOut(MPU6050_I2C_WAIT);
	n = I2c.write(address, reg, data); 
	return (-n);
}

#endif
