/*
Segway* Clone by "Ovaltineo" for Arduino

*Segway is a trademark of Segway Inc.

Copyright (C) 2013  Carmelo Porciuncula

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

This program includes MPU-6050 routines from arduino.cc user "Krodal".

See the README for instructions.

*/



#include "I2C.h"
#include "SegwayClone.h"
#include "Motors.h"
#include "Alarms.h"
#include "TC74.h"
#include "EEPROM.h"


static unsigned long currMicros = 0;
static unsigned long pastMicros = 0;
static unsigned long dt;

#ifdef HX711_DOUT
static float weight = 0;
static float sample_weight = 0;
#endif

static float board_angle = 0;
static float steer_angle = 0;
static float integral_angle = 0;
static int board_offset = 0;
static int steer_offset = 0;
static float last_board_offset = 0;
static float motor = 0;
static float Pf, Df, Pf_off, Df_off;
static int loopcount = 0;
static uint8_t alarmArray[11];
static int remote_active = 0;
static boolean waitForLevel;
static boolean waitForEver = false;
#ifdef LOCKED_BY_DEFAULT
static boolean locked = true;
#else
static boolean locked = false;
#endif


long left_motor;
long right_motor;
float Vref = 0, Vf = 0;
int Vi;

int temp1=0;
int temp2=0;

int current1=0;
int current2=0;

void getPD()
{
	//int Pv = analogRead(P_PIN);	// read P potentiometer
	//Pf = (.8 * Pv/1023);	// range 0 to 0.8
	//int Dv = analogRead(D_PIN);	// read D potentiometer
	//Df = (.5 * Dv/1023);	// range 0 to 0.5

#ifdef MODE_CALIBRATE
	Pf = KP;                    // use static value
	Df = KD;                    // use static value
	Pf_off = KP_OFF;			// use static value
	Df_off = KD_OFF;			// use static value
	
	writeIntToEEPROM(EEPROM_P_ADDR, Pf*100);		// save P to EEPROM
	writeIntToEEPROM(EEPROM_D_ADDR, Df*100);		// save D to EEPROM
	writeIntToEEPROM(EEPROM_PD_SAVED_ADDR, PD_SAVED);	// set saved flag in EEPROM
	writeIntToEEPROM(EEPROM_P_OFF_ADDR, Pf_off*100);		// save P_OFF to EEPROM
	writeIntToEEPROM(EEPROM_D_OFF_ADDR, Df_off*100);		// save D_OFF to EEPROM
	writeIntToEEPROM(EEPROM_PD_OFF_SAVED_ADDR, PD_SAVED);	// set saved flag in EEPROM
#endif
#if !defined(MODE_CALIBRATE) && defined(BLUETOOTH) 
	int saved = readIntFromEEPROM(EEPROM_PD_SAVED_ADDR);	// get saved flag from EEPROM
	if (saved == PD_SAVED)							// check if PD has been saved to EEPROM
	{
		int Pi = readIntFromEEPROM(EEPROM_P_ADDR);	// read P from EEPROM
		Pf = Pi/100.0;
		int Di = readIntFromEEPROM(EEPROM_D_ADDR);	// read D from EEPROM
		Df = Di/100.0;
	}
	else
	{
		Pf = KP;                      // use static value
		Df = KD;                      // use static value
	}
	saved = readIntFromEEPROM(EEPROM_PD_OFF_SAVED_ADDR);	// get OFF saved flag from EEPROM
	if (saved == PD_SAVED)							// check if PD_OFF has been saved to EEPROM
	{
		int Pi = readIntFromEEPROM(EEPROM_P_OFF_ADDR);	// read P_OFF from EEPROM
		Pf_off = Pi/100.0;
		int Di = readIntFromEEPROM(EEPROM_D_OFF_ADDR);	// read D_OFF from EEPROM
		Df_off = Di/100.0;
	}
	else
	{
		Pf_off = KP_OFF;                      // use static value
		Df_off = KD_OFF;                      // use static value
	}
	saved = readIntFromEEPROM(EEPROM_VOLTAGE_REF_SAVED_ADDR);	// get VOLTAGE_REF saved flag from EEPROM
	if (saved == PD_SAVED)							// check if VOLTAGE_REF has been saved to EEPROM
	{
		int Vi = readIntFromEEPROM(EEPROM_VOLTAGE_REF_ADDR);	// read Vref from EEPROM
		Vref = Vi/100.0;
	}
	
#endif	
#if !defined(MODE_CALIBRATE) && !defined(BLUETOOTH) 
	Pf = KP;                      // use static value
	Df = KD;                      // use static value
	Pf_off = KP_OFF;              // use static value
	Df_off = KD_OFF;              // use static value
#endif
}

boolean riderIsOn(void)
{
	static boolean lastValue = true;
	boolean value = true;
	
#ifdef HX711_DOUT
	value = checkHX711();
#elif defined(RIDER_SENSOR)
	int state = digitalRead(RIDER_SENSOR);
	#ifdef INVERT_RIDER_SENSOR
		value = state;
	#else
		value = !state;
	#endif
#endif
	if (lastValue != value)
	{
		lastValue = value;
		if (lastValue)
			alarmArray[ALARM_RIDER] = 1;
		else
			alarmArray[ALARM_RIDER] = 2;
	}
	return (value);
}

float complementaryFilter(float accel, float gyro, long micros, float filtered_accel, float angle)
{
	angle = ALPHA * (angle + (gyro * micros/1000000.0));
	angle += (1-ALPHA) * filtered_accel;
	
	return angle;
}

float complementaryFilterSlow(float accel, float gyro, long micros, float filtered_accel, float angle)
{
	angle = ALPHA_SLOW * (angle + (gyro * micros/1000000.0));
	angle += (1-ALPHA_SLOW) * filtered_accel;
	
	return angle;
}
float getBoardPitchAngle(float accel, float gyro, long micros)
{
	static float filtered_accel = 0;
	static float angle = 0;

	filtered_accel = lowpassFilter(filtered_accel, accel, ACC_FILTER);
	angle = complementaryFilter(accel, gyro, micros, filtered_accel, angle);
	return angle;
}

#if defined(SUBTRACT_BANK_ANGLE) && defined(MPU6050_X2_STEERING)
float getBoardBankAngle(float accel, float gyro, long micros)
{
	static float filtered_accel = 0;
	static float angle = 0;
	filtered_accel = lowpassFilter(filtered_accel, accel, ACC_FILTER);
//	angle = complementaryFilter(accel, SUBTRACT_BANK_ANGLE*gyro, micros, filtered_accel, angle);
	angle = complementaryFilterSlow(accel, 0, micros, filtered_accel, angle);
	return SUBTRACT_BANK_ANGLE*angle;
}
#endif

float computePID(float angle, float gyro, boolean riderOn)
{
	float result, Ptemp, Dtemp;
	
	if (riderOn)
	{
#if defined(HX711_DOUT) && defined (HX711_REFERENCE_WEIGHT)
  #if defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
		result = ((Pf_off + (Pf-Pf_off) * sample_weight/HX711_REFERENCE_WEIGHT) * 10 * angle) + ((Df_off + (Df-Df_off) * sample_weight/HX711_REFERENCE_WEIGHT) * 10 * gyro);
  #else
		result = ((Pf_off + (Pf-Pf_off) * sample_weight/HX711_REFERENCE_WEIGHT) * angle) + ((Df_off + (Df-Df_off) * sample_weight/HX711_REFERENCE_WEIGHT) * gyro);
  #endif
#else
  #if defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
		result = (Pf * 10 * angle) + (Df * 10 * gyro);
  #else
		result = (Pf * angle) + (Df * gyro);
  #endif
#endif
	}
	else
	{
#if defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
		result = (Pf_off * 10 * angle) + (Df_off * 10 * gyro);
#else
		result = (Pf_off * angle) + (Df_off * gyro);
#endif
	}
	return result;
}

#ifdef POT_STEERING
/* Use the following if using a potentiometer for the steering.
   This assumes pot center returns 512, full left returns 0, and full right returns 1023.
*/
float getSteerAngle(long micros)
{
	int pot = analogRead(POT_STEERING_PIN);	// read D potentiometer, range is 0 to 1023
        pot = pot - 512;                // range is now -512 to +511
	return (pot*240.0/1023.0)-POT_STEERING_ADJUST;	        // convert to degrees (assumes pot has 240 degree sweep)
}
#else

/* Use the following if using an accelerometer for the steering.
*/
float getSteerAngle(long micros)
{
	static float filtered_accel = 0;
	static float angle = 0;
	float accel;
	float gyro;

	accel = getSteerAccel();
	gyro = getSteerGyro();
	filtered_accel = lowpassFilter(filtered_accel, accel, ACC_FILTER);
	angle = complementaryFilter(accel, gyro, micros, filtered_accel, angle);
	return angle;
}
#endif

/* Use the following if using a pot to adjust forward/backward tilt
*/
#ifdef POT_TILT_PIN
float getTiltAngle(void)
{
	int pot = analogRead(POT_TILT_PIN);	// read D potentiometer, range is 0 to 1023
	pot = pot - 512;                // range is now -512 to +511
	return (pot*MAX_ANGLE/1023.0);	// convert to degrees (this should return -MAX_ANGLE TO MAX_ANGLE)
}
#endif

void buzzer(float on)
{
	if (on)
		digitalWrite(BUZZER_PIN, HIGH);
	else
		digitalWrite(BUZZER_PIN, LOW);
}

void setup()
{
	Serial.begin(115200);			// initialize serial port (for debugging output)
	I2c.begin();					// initialize I2C
	
	initMotors();			// initialize the motors
	initAlarm();			// initialize alarm including LEDs
	
#if defined(RIDER_SENSOR)	
	pinMode(RIDER_SENSOR, INPUT);
	digitalWrite(RIDER_SENSOR, HIGH);
#endif
	
	buzzer(true);			// turn on buzzer
#ifdef FAN_PIN
	pinMode(FAN_PIN, OUTPUT);		// set fan pin as output
	digitalWrite(FAN_PIN, HIGH);	// turn on the fan
#endif
	delay(1000);			// give sensors a second to stabilize
#ifdef FAN_PIN
	digitalWrite(FAN_PIN, LOW);	// turn off the fan
#endif
	buzzer(false);			// turn off buzzer
	
#ifdef MODE_CALIBRATE
	delay(1000);			// wait 1 second
	initBoardSensor(CALIBRATE_YES);		// initialize board tilt sensor and calibrate
  #ifndef POT_STEERING
	initSteerSensor(CALIBRATE_YES);		// initialize steering tilt sensor and calibrate
  #endif
#else
	initBoardSensor(CALIBRATE_NO);		// initialize board tilt sensor
  #ifndef POT_STEERING
	initSteerSensor(CALIBRATE_NO);		// initialize steering tilt sensor
  #endif
#endif

#ifdef BLUETOOTH
	initBluetooth();
#endif

#ifdef HX711_DOUT
	initHX711();		// initialize weight (rider) sensor
#endif

	getPD();			// get P and D values
	
	waitForLevel = true;	// set flag to wait for board to level
#ifdef MODE_CALIBRATE
	buzzer(true);			// turn on buzzer to indicate end of calibration
#endif	
}

void loop()
{
#ifndef MODE_CALIBRATE
	float board_accel;
	float board_gyro;
	float speed_adjust;
	float yaw_gyro;
	float steer;
	long motorMax;
	float steer_gyro;
	boolean riderOn;
	
	currMicros = micros();

	dt = currMicros-pastMicros;

#if defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(SABERTOOTH_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
	if (dt >= 12500) // check if 12500 microseconds (12.5 milliseconds) has elapsed. Needs slower rate due to 38400 bps serial
#else	
	if (dt >= 10000) // check if 10000 microseconds (10 milliseconds) has elapsed
#endif	
	{
		riderOn = riderIsOn();
		pastMicros = currMicros;		// reset timer
		board_accel = getBoardAccel();	// get board pitch accelerometer value
		board_gyro = getBoardGyro();	// get board pitch (tilt) gyro value
		board_angle = getBoardPitchAngle(board_accel, board_gyro, dt);	// compute board pitch angle
		
#ifdef MAX_YAW_RATE_RIDER_ON		
		yaw_gyro = abs(getBoardYawGyro());	// get board yaw (direction) gyro value
#endif
		if (locked)						// check if locked
		{
			alarmArray[ALARM_LOCKED] = 1;
			if (motor > 0)				// slow down quickly and stop turning.  Warning: rider will most likely fall if lock is engaged while moving!!!
				motor--;
			else if (motor < 0)
				motor++;
			controlMotors(motor, motor);	
			waitForLevel = true;			// set flag to wait for board to be levelled
		}
#ifdef MAX_YAW_RATE_RIDER_ON		
		else if (waitForEver || (yaw_gyro > MAX_YAW_RATE_RIDER_ON && riderOn) || (yaw_gyro > MAX_YAW_RATE_RIDER_OFF && !riderOn))	// check if out of control!!
		{
			alarmArray[ALARM_MAX_ANGLE] = 1;
			controlMotors(0, 0);	
			waitForEver = true;			// set flag to wait for ever - safety shutdown
		}
#endif		
		else if (waitForLevel)	// check if we need to wait for board to level
		{
			alarmArray[ALARM_LEVEL] = 1;
			controlMotors(0, 0);		// send stop to motors
			if (board_angle > -2 && board_angle < 2)	// check if board is sort of level (+/- 2 degrees)
			{
				waitForLevel = false;	// set flag to indicate board has been levelled
				motor = 0;				// make sure motor is reset to zero
				integral_angle = 0;		// make sure integral angle is reset to zero
				buzzer(false);			// turn off buzzer
			}
		}
		else
		{
#ifdef POT_TILT_PIN
			board_offset = getTiltAngle();
#else
			if (remote_active == 0)
				board_offset = 0;
#endif
				
			// make small changes to offset to avoid jerk which causes oscillation
			if (last_board_offset - board_offset/BOARD_OFFSET_DIVIDER >= .1)
				last_board_offset = last_board_offset - .1;
			else if (board_offset/BOARD_OFFSET_DIVIDER - last_board_offset  >= .1)
				last_board_offset = last_board_offset + .1;
			else
				last_board_offset = board_offset/BOARD_OFFSET_DIVIDER;
			speed_adjust = computePID(board_angle + last_board_offset, board_gyro, riderOn);		// compute speed adjustment using PID routine, with gradual offset
//			speed_adjust = computePID(board_angle + board_offset, board_gyro);		// compute speed adjustment using PID routine, with immediate offset
//			speed_adjust = computePID(board_angle, board_gyro);		// compute speed adjustment using PID routine
			motor = motor + speed_adjust;

#if defined(VOLTAGE_CHECK) || defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
			// compensate for voltage change
			if (Vref > 0)
				motor = motor * Vref/Vf;
#endif			
			
			if (riderOn)
				motorMax = MOTOR_MAX;
			else
				motorMax = MOTOR_MAX_OFF;

			if (motor > motorMax)
				motor = motorMax;
			else if (motor < -motorMax)
				motor = -motorMax;


			if (remote_active > 0)
				steer_angle = steer_offset/STEER_OFFSET_DIVIDER;					// get steering angle from remote steer offset
			else
			{
				steer_angle = getSteerAngle(dt);				// get steering angle from sensor
#if defined(SUBTRACT_BANK_ANGLE) && defined(MPU6050_X2_STEERING)
				float board_bank_accel;
				float board_bank_gyro;
				float board_bank_angle;

				board_bank_accel = getBoardBankAccel();	// get board bank accelerometer value
				board_bank_gyro = getBoardBankGyro();	// get board bank gyro value
				board_bank_angle = getBoardBankAngle(board_bank_accel, board_bank_gyro, dt);	// compute board bank angle
				steer_angle = steer_angle - board_bank_angle;
#endif				
			}
#ifdef EXPONENTIAL_STEER_DIVIDER
			steer = steer_angle*steer_angle*STEER_MULTIPLIER/EXPONENTIAL_STEER_DIVIDER;
			if (steer_angle < 0)
				steer = -steer;
#else
			steer = steer_angle * STEER_MULTIPLIER;
#endif
			
		#ifdef INVERT_STEERING
			steer = -steer;								// invert steering if needed
		#endif

			left_motor = motor - steer;					// subtract steering offset to left motor
			right_motor = motor + steer;				// add steering offset from right motor
			
		#ifdef PUSHBACK
			left_motor += processPushback(motor);
			right_motor += processPushback(motor);
		#else
			// turn on alarm for motor if required
			if (motor > (motorMax * MOTOR_ALARM_PERCENT/100.0) || motor < -(motorMax * MOTOR_ALARM_PERCENT/100.0))
				alarmArray[ALARM_MAX_MOTOR] = 1;
		#endif
			
			left_motor = left_motor * MOTOR_LR_RATIO;
			right_motor = right_motor / MOTOR_LR_RATIO;
			
			// clip motor to motorMax
			if (left_motor > motorMax)
			  left_motor = motorMax;
			else if (left_motor < -motorMax)
			  left_motor = -motorMax;
			
			if (right_motor > motorMax)
			  right_motor = motorMax;
			else if (right_motor < -motorMax)
			  right_motor = -motorMax;
		
			if (board_angle < MIN_ANGLE || board_angle > MAX_ANGLE ) // stop motors if over-angle
			{
				left_motor = 0;
				right_motor = 0;
				motor = 0;
				waitForLevel = true;	// set flag to wait for board to level
			}
			
			loopcount++;
			if (loopcount==20)			
			{
			#ifdef VOLTAGE_CHECK
				int voltage = analogRead(VOLTAGE_PIN);  // voltage sensor

				float newVf = 5*voltage/(1023*VOLTAGE_DIVIDER);
				
				Vf = lowpassFilter(Vf, newVf, VOLT_FILTER);
				Vi = Vf * 100;
			#endif
			
			#if defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
				float Vx = readVoltage();
Serial.print(F(" Vx: ")); 
Serial.println(Vx);
				
				if (Vx > 0)
				{
					Vf = Vx;
					Vi = Vf * 100;
				}
				int Tempx = readTemperature();
				if (Tempx > 0)
				{
					temp1 = Tempx;
					temp2 = temp1;
				}
				//readCurrents();
			#endif

			#if defined(VOLTAGE_CHECK) || defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
				if (Vf < 5)										// check if powered by USB, if so then don't alarm
						alarmArray[ALARM_BATTERY] = 0;
				else if (Vf < MIN_BATTERY_4)					// check if below battery level 4
						alarmArray[ALARM_BATTERY] = 4;
				else if (Vf < MIN_BATTERY_3)					// check if below battery level 3
						alarmArray[ALARM_BATTERY] = 3;
				else if (Vf < MIN_BATTERY_2)					// check if below battery level 2
						alarmArray[ALARM_BATTERY] = 2;
				else if (Vf < MIN_BATTERY_1)					// check if below battery level 1
						alarmArray[ALARM_BATTERY] = 1;
				else
						alarmArray[ALARM_BATTERY] = 0;
			#endif

			
			#ifdef TEMPERATURE_SENSORS
				int error1 = TC74_read (TC74_ADD1, &temp1);
				int error2 = TC74_read (TC74_ADD2, &temp2);
				if (error1)
				{
								temp1 = -1;
					alarmArray[ALARM_I2C] = 1;
					Serial.print(F("I2C Error = "));
					Serial.println(error1,DEC);
				}
				if (error2)
				{
								temp2 = -1;
					alarmArray[ALARM_I2C] = 1;
					Serial.print(F("I2C Error = "));
					Serial.println(error2,DEC);
				}
			#endif

			#if defined(TEMPERATURE_SENSORS) || defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
				// set alarm if very hot
				if (temp1 > MAX_TEMP || temp2 > MAX_TEMP)
						alarmArray[ALARM_TEMP] = 1;
				#ifdef FAN_PIN
				// turn on fan if getting hot
				if (temp1 > TEMP_FAN_ON || temp2 > TEMP_FAN_ON)
						digitalWrite(FAN_PIN, HIGH);
				// turn off fan if cool
				else if (temp1 < TEMP_FAN_OFF && temp2 < TEMP_FAN_OFF)
						digitalWrite(FAN_PIN, LOW);
				#endif
			#endif
			
				loopcount = 0;
			
				if (remote_active > 0)			// slowly turn off remote_active if it is on
					remote_active--;
                Serial.println();
				#ifdef HX711_DOUT
				Serial.print(F(" WEIGHT: ")); 
				Serial.print(weight);
				#endif
				Serial.print(F(" ACCEL: ")); 
				Serial.print(board_accel);
				Serial.print(F(" GYRO: ")); 
				Serial.print(board_gyro);
#ifdef MAX_YAW_RATE_RIDER_ON		
				Serial.print(F(" YAW: ")); 
				Serial.print(yaw_gyro);
#endif				
				Serial.print(F(" BOARD ANGLE: ")); 
				Serial.print(board_angle);
				Serial.print(F(" STEER ANGLE: ")); 
				Serial.print(steer_angle);
				Serial.print(F(" MOTOR: ")); 
				Serial.print(motor);		
				Serial.print(F(" LEFT: ")); 
				Serial.print(left_motor);		
				Serial.print(F(" RIGHT: ")); 
				Serial.print(right_motor);		
				Serial.print(F(" VOLTAGE: ")); 
				Serial.print(Vf);		
				Serial.print(F(" TEMP: ")); 
				Serial.print(temp1);		
				Serial.print(F(" CURRENT1: ")); 
				Serial.print(current1);		
				Serial.print(F(" CURRENT2: ")); 
				Serial.println(current2);		
			}
			
			controlMotors(left_motor, right_motor);		// control left and right motors
		}
				
	#ifdef BLUETOOTH
		processBluetooth();
	#endif
	}
	delay(1);
	alarmHandler();
#endif
}
