#ifdef BLUETOOTH
	#ifndef MEGA
		#include <SoftwareSerial.h>
		SoftwareSerial Serial3(SOFT_RX_PIN3, SOFT_TX_PIN3);
	#endif

	
void setbaud(long speed)
{
	if (speed == 9600)
		Serial3.print("AT+BAUD4");
	else if (speed == 19200)
		Serial3.print("AT+BAUD5");
	else if (speed == 38400)
		Serial3.print("AT+BAUD6");
	else if (speed == 57600)
		Serial3.print("AT+BAUD7");
	else if (speed == 115200)
		Serial3.print("AT+BAUD8");
		
	Serial3.begin(speed);
#ifdef MEGA
	digitalWrite(15, HIGH);
#endif
	delay(1000);
    while (Serial3.available())
            Serial.write(Serial3.read());
	Serial.println();
}

boolean check(long speed)
{
	char buffer[2];
	int pointer = 0;

	Serial.print("Try ");
	Serial.println(speed);
	Serial3.begin(speed);
#ifdef MEGA
	digitalWrite(15, HIGH);
#endif
	Serial3.print("AT");
	delay(1000);
	while (Serial3.available()  && pointer < 2)
	{
		buffer[pointer] = Serial3.read();
		//		Serial.write(buffer[pointer]);
		pointer++;
	}
	if (buffer[0] == 'O' && buffer[1] == 'K')
	{
		Serial.print("Found bluetooth module at ");
		Serial.println(speed);
		return true;
	}
	return false;
}

void setName(String name)
{
	Serial3.print("AT+NAME");  // change name from linvor or HC-06 or whatever
	Serial3.print(name);  
	delay(1000);  
    while (Serial3.available())
            Serial.write(Serial3.read());
	Serial.println();
}

void getVersion(void)
{
    Serial3.print("AT+VERSION");
	delay(1000);
    while (Serial3.available())
            Serial.write(Serial3.read());
	Serial.println();
}

void initBluetooth(void)
{
	boolean found = false;

	found = check(115200);
	if (!found)
		found = check(19200);
	if (!found)
		found = check(9600);
	if (!found)
		found = check(57600);
	if (!found)
		found = check(38400);
	if (!found)
		Serial.println("Bluetooth module not found!");
#ifdef MODE_CALIBRATE		
	if (found)
	{
		setName("SEGWAY");
	#ifdef MEGA
		setbaud(115200);
	#else
		setbaud(19200);
	#endif		
	}
#endif
}
	
void processBluetooth(void)
{
	byte timeout;
        int volt;
	
	while (Serial3.available())
	{
		// Check for commands 
		unsigned char command=Serial3.read();
		if (command=='u')	// unlock
		{
			Serial3.write(0xFF);			// send unlock ack success (byte 1)
			Serial3.write(0xFF);			// send unlock ack success (byte 2)
			locked = false;
		}
		else if (command=='l')	// lock
		{
			if (motor > -MOTOR_MAX/10 && motor < MOTOR_MAX/10)		// lock only if speed is less than 1/10th of MOTOR_MAX
			{
				Serial3.write(0xFF);			// send lock ack success (byte 1)
				Serial3.write(0xFF);			// send lock ack success (byte 2)
				locked = true;
			}
			else
			{
				Serial3.write((uint8_t) 0);				// send lock ack failed (byte 1)
				Serial3.write((uint8_t) 0);				// send lock ack failed (byte 2)
			}
		}
		else if (command=='c')	// calibrate
		{
	#if defined(VOLTAGE_CHECK) || defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
			if (Vf < 5 || (motor > -MOTOR_MAX/10 && motor < MOTOR_MAX/10))		// lock only if using USB power or speed is less than 1/10th of MOTOR_MAX
	#else
			if (motor > -MOTOR_MAX/10 && motor < MOTOR_MAX/10)		// lock only if speed is less than 1/10th of MOTOR_MAX
	#endif
			{
				controlMotors(0, 0);				// make sure motors are stopped
				initBoardSensor(CALIBRATE_YES);		// initialize board tilt sensor and calibrate
				#ifndef POT_STEERING
					initSteerSensor(CALIBRATE_YES);	// initialize steering tilt sensor and calibrate
				#endif
	#if defined(VOLTAGE_CHECK) || defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
				writeIntToEEPROM(EEPROM_VOLTAGE_REF_ADDR, Vi);					// save voltage to EEPROM
				writeIntToEEPROM(EEPROM_VOLTAGE_REF_SAVED_ADDR, PD_SAVED);			// set save flag in EEPROM

	#endif
				Serial3.write(0xFF);				// send calibrate ack (byte 1)
				Serial3.write(0xFF);				// send calibrate ack (byte 2)

				waitForLevel = true;				// set flag to wait for board to level
				buzzer(true);						// turn on buzzer to indicate board is waiting to level, or end of calibration
			}
			else
			{
				Serial3.write((uint8_t) 0);				// send calibrate ack failed (byte 1)
				Serial3.write((uint8_t) 0);				// send calibrate ack failed (byte 2)
			}
		}
		else if (command=='x')	// "Joystick" XY in next byte
		{
			timeout = 30; //set timeout to 30x100us or 3ms
			while (!Serial3.available() && timeout > 0)
			{
				  delayMicroseconds(100);
				  timeout--;
			} // wait
			if (!timeout)
				return;			// read timeout so just abort
			command=Serial3.read();
			steer_offset=command << 8;

			timeout = 30; //set timeout to 30x100us or 3ms
			while (!Serial3.available() && timeout > 0)
			{
				  delayMicroseconds(100);
				  timeout--;
			} // wait
			if (!timeout)
				return;			// read timeout so just abort
			command=Serial3.read();
			steer_offset= steer_offset + command;
			
			timeout = 30; //set timeout to 30x100us or 3ms
			while (!Serial3.available() && timeout > 0)
			{
				  delayMicroseconds(100);
				  timeout--;
			} // wait
			if (!timeout)
				return;			// read timeout so just abort
			command=Serial3.read();
			board_offset=command << 8;

			timeout = 30; //set timeout to 30x100us or 3ms
			while (!Serial3.available() && timeout > 0)
			{
				  delayMicroseconds(100);
				  timeout--;
			} // wait
			if (!timeout)
				return;			// read timeout so just abort
			command=Serial3.read();
			board_offset= board_offset + command;

			remote_active = 10;									// set flag to indicate remote control is on
			Serial3.write(0xFF);								// send joystick ack (byte 1)
			Serial3.write(0xFF);								// send joystick ack (byte 2)


			Serial.print(F("board: "));		
			Serial.print(board_offset);		
			Serial.print(F(" steer: "));		
			Serial.print(steer_offset);		
			Serial.println("");		
		}
		else if (command=='P')	// save P and D
		{
			timeout = 30; //set timeout to 30x100us or 3ms
			while (!Serial3.available() && timeout > 0)
			{
				  delayMicroseconds(100);
				  timeout--;
			} // wait
			if (!timeout)
				return;			// read timeout so just abort
			command=Serial3.read();
			int pidp = command << 8;

			timeout = 30; //set timeout to 30x100us or 3ms
			while (!Serial3.available() && timeout > 0)
			{
				  delayMicroseconds(100);
				  timeout--;
			} // wait
			if (!timeout)
				return;			// read timeout so just abort
			command=Serial3.read();
			pidp = pidp + command;

			if (riderIsOn())
			{
				writeIntToEEPROM(EEPROM_P_ADDR, pidp);				// save P to EEPROM
				Pf = pidp/100.0;
			}
			else
			{
				writeIntToEEPROM(EEPROM_P_OFF_ADDR, pidp);			// save P_OFF to EEPROM
				Pf_off = pidp/100.0;
			}

			timeout = 30; //set timeout to 30x100us or 3ms
			while (!Serial3.available() && timeout > 0)
			{
				  delayMicroseconds(100);
				  timeout--;
			} // wait
			if (!timeout)
				return;			// read timeout so just abort
			command=Serial3.read();
			int pidd = command << 8;

			timeout = 30; //set timeout to 30x100us or 3ms
			while (!Serial3.available() && timeout > 0)
			{
				  delayMicroseconds(100);
				  timeout--;
			} // wait
			if (!timeout)
				return;			// read timeout so just abort
			command=Serial3.read();
			pidd = pidd + command;

			if (riderIsOn())
			{
				writeIntToEEPROM(EEPROM_D_ADDR, pidd);				// save D to EEPROM
				Df = pidd/100.0;
				writeIntToEEPROM(EEPROM_PD_SAVED_ADDR, PD_SAVED);	// set saved flag in EEPROM
			}
			else
			{
				writeIntToEEPROM(EEPROM_D_OFF_ADDR, pidd);				// save D to EEPROM
				Df_off = pidd/100.0;
				writeIntToEEPROM(EEPROM_PD_OFF_SAVED_ADDR, PD_SAVED);	// set saved flag in EEPROM
			}
			Serial.print(F("P: "));		
			Serial.print(pidp);		
			Serial.print(F(" D: "));		
			Serial.print(pidd);		
			Serial.println("");		
		}
		else if (command=='d')	// request data
		{
			int board = board_angle * 100;
			int steer = steer_angle * 100;
			int i_left_motor;
			int i_right_motor;
			int min_batt;
			int pidp; 
			int pidd; 

			if (riderIsOn())
			{
				pidp = Pf * 100.0;
				pidd = Df * 100.0;
			}
			else
			{
				pidp = Pf_off * 100.0;
				pidd = Df_off * 100.0;
			}
			Serial3.write(Vi >> 8);	
			Serial3.write(Vi & 0xFF);	

			Serial3.write(board >> 8);	
			Serial3.write(board & 0xFF);	

			Serial3.write(steer >> 8);	
			Serial3.write(steer & 0xFF);	
			
		#if defined(TEMPERATURE_SENSORS) || defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
			Serial3.write(temp1 >> 8);	
			Serial3.write(temp1 & 0xFF);	

			Serial3.write(temp2 >> 8);	
			Serial3.write(temp2 & 0xFF);
		#else
			Serial3.write((uint8_t)0);	
			Serial3.write((uint8_t)0);	

			Serial3.write((uint8_t)0);	
			Serial3.write((uint8_t)0);
		#endif

			i_left_motor = left_motor*255.0/MOTOR_MAX;
			i_right_motor = right_motor*255.0/MOTOR_MAX;
			Serial3.write(i_left_motor >> 8);	
			Serial3.write(i_left_motor & 0xFF);	

			Serial3.write(i_right_motor >> 8);	
			Serial3.write(i_right_motor & 0xFF);	

			Serial3.write(pidp >> 8);	
			Serial3.write(pidp & 0xFF);	

			Serial3.write(pidd >> 8);	
			Serial3.write(pidd & 0xFF);	

			unsigned short status = 0;
	
			if (locked)
				status = status | 0b00000001;
			if (waitForLevel)
				status = status | 0b00000010;
			if (motor > (MOTOR_MAX * MOTOR_ALARM_PERCENT/100.0) || motor < -(MOTOR_MAX * MOTOR_ALARM_PERCENT/100.0))
				status = status | 0b00000100;
			if (board_angle < MIN_ANGLE || board_angle > MAX_ANGLE )
				status = status | 0b00001000;
		#if defined(TEMPERATURE_SENSORS) || defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
			if (temp1 > MAX_TEMP || temp2 > MAX_TEMP)
				status = status | 0b00010000;
		#endif
			Serial3.write(status >> 8);	
			Serial3.write(status & 0xFF);	
/*
			Serial3.write((uint8_t)(MOTOR_MAX >> 8));	
			Serial3.write(MOTOR_MAX & 0xFF);	

			Serial3.write((uint8_t)(MAX_ANGLE >> 8));	
			Serial3.write(MAX_ANGLE & 0xFF);	
*/
			Serial3.write((uint8_t)(current1 >> 8));	
			Serial3.write(current1 & 0xFF);	

			Serial3.write((uint8_t)(current2 >> 8));	
			Serial3.write(current2 & 0xFF);	

			Serial3.write((uint8_t)(MAX_TEMP >> 8));	
			Serial3.write(MAX_TEMP & 0xFF);	

			min_batt = MIN_BATTERY_4 * 10;
			Serial3.write((uint8_t)(min_batt >> 8));			
			Serial3.write(min_batt & 0xFF);		

			if (riderIsOn())
				Serial3.write(0xFF);			// rider Flag=true
			else
				Serial3.write(0x7F);			// rider Flag=false
			Serial3.write(0xFF);			// padding for future data
			
			Serial3.write(0xFF);			// padding for future data
			Serial3.write(0xFF);			// padding for future data

			}
	}
}
#endif
