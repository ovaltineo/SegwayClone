/*
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
*/

#include "Motors.h"

#if (defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(SABERTOOTH_CONTROLLER)) && !defined(MEGA)
	#include <SoftwareSerial.h>
#endif

#if defined(PWM_PWM_CONTROLLER) || defined(PWM_DIR_CONTROLLER) || defined(PWM_SHIELD_CONTROLLER) || defined(L298N_CONTROLLER) || defined(OSMC_CONTROLLER)
	void setPwmFrequency(byte mode) {
		if (mode >=1 && mode <=5)
		{
	#if defined(MEGA) && !defined(PWM_SHIELD_CONTROLLER)
			// set PWM frequency for pins 2,3,5
			TCCR3B = TCCR3B & 0b11111000 | mode;
			// set PWM frequency for pins 6,7,8
			TCCR4B = TCCR4B & 0b11111000 | mode;
	#endif
	#if defined(MEGA) && defined(PWM_SHIELD_CONTROLLER)
			// set PWM frequency for pins 11,12
			TCCR1B = TCCR1B & 0b11111000 | mode;
			// set PWM frequency for pins 9,10
			TCCR2B = TCCR2B & 0b11111000 | mode;
			// set PWM frequency for pins 2,3,5
			TCCR3B = TCCR3B & 0b11111000 | mode;
	#endif
	#ifndef MEGA
			// set PWM frequency for pins 9,10
			TCCR1B = TCCR1B & 0b11111000 | mode;
			// set PWM frequency for pins 3,11
			TCCR2B = TCCR2B & 0b11111000 | mode;
	#endif
		}
	}
#endif

#ifdef PWM_PWM_15KHZ_CONTROLLER
void setPwmFrequency(byte mode) 
{
	if (mode >=1 && mode <=5)
	{
	
		// set PWM frequency for pins 2,3,5
		TCCR3B = TCCR3B & 0b11111000 | mode;
		// set PWM frequency for pins 6,7,8
		TCCR4B = TCCR4B & 0b11111000 | mode;
		// set PWM frequency for pins 11,12
		TCCR1B = TCCR1B & 0b11111000 | mode;
		// set PWM frequency for pins 9,10
		TCCR2B = TCCR2B & 0b11111000 | mode;

/*
      // init 16bit timer 4
      TCCR3B &= ~(1<<WGM43);
      TCCR3B &= ~(1<<WGM42);
      TCCR3A |= (1<<WGM41); // phase correct mode, TOP=0x1FF
      TCCR3A &= ~(1<<WGM40);


      // init 16bit timer 4
      TCCR4B &= ~(1<<WGM43);
      TCCR4B &= ~(1<<WGM42);
      TCCR4A |= (1<<WGM41); // phase correct mode, TOP=0x1FF
      TCCR4A &= ~(1<<WGM40);

      
      TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
      TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
      TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
	  
      TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
      TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B
      TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
*/	  
	}
}
#endif

#ifdef L298N_CONTROLLER
	void initMotors(void)
	{
		pinMode(LEFT_ENA_PIN, OUTPUT);		// set left motor pin to output
		pinMode(LEFT_IN1_PIN, OUTPUT);
		pinMode(LEFT_IN2_PIN, OUTPUT);
		pinMode(RIGHT_ENB_PIN, OUTPUT);     // set right motor pins to output
		pinMode(RIGHT_IN3_PIN, OUTPUT);
		pinMode(RIGHT_IN4_PIN, OUTPUT);		

		digitalWrite(LEFT_ENA_PIN, LOW);
		digitalWrite(RIGHT_ENB_PIN, LOW);
		
		digitalWrite(LEFT_IN1_PIN, HIGH);
		digitalWrite(LEFT_IN2_PIN, LOW);
		digitalWrite(RIGHT_IN3_PIN, HIGH);
		digitalWrite(RIGHT_IN4_PIN, LOW);
		
	}


	void controlMotors(long speedL, long speedR) {
		if (speedL<0) 
		{
			speedL = -speedL;
			digitalWrite(LEFT_IN1_PIN, LOW);
			digitalWrite(LEFT_IN2_PIN, HIGH);
		}
		else
		{
			digitalWrite(LEFT_IN1_PIN, HIGH);
			digitalWrite(LEFT_IN2_PIN, LOW);
		}
		analogWrite(LEFT_ENA_PIN, speedL);

		if (speedR<0) 
		{
			speedR = -speedR;
			digitalWrite(RIGHT_IN3_PIN, LOW);
			digitalWrite(RIGHT_IN4_PIN, HIGH);
		}
		else
		{
			digitalWrite(RIGHT_IN3_PIN, HIGH);
			digitalWrite(RIGHT_IN4_PIN, LOW);
		}
		analogWrite(RIGHT_ENB_PIN, speedR);
	}
#endif

#ifdef PWM_SHIELD_CONTROLLER
	void initMotors(void)
	{
		setPwmFrequency(HZ_3906);				// set PWM frequency to 3.906 kHz
		pinMode(LEFT_ENABLE_PIN, OUTPUT);
		pinMode(LEFT_DISABLE_PIN, OUTPUT);
		pinMode(LEFT_BACKWARD_PIN, OUTPUT);		// set left motor pin to output
		pinMode(LEFT_FORWARD_PIN, OUTPUT);		// set left motor pin to output
		pinMode(RIGHT_ENABLE_PIN, OUTPUT);
		pinMode(RIGHT_DISABLE_PIN, OUTPUT);
		pinMode(RIGHT_BACKWARD_PIN, OUTPUT);	// set right motor pin to output
		pinMode(RIGHT_FORWARD_PIN, OUTPUT);		// set right motor pin to output
		
		digitalWrite(LEFT_FORWARD_PIN, HIGH);
		digitalWrite(LEFT_BACKWARD_PIN, HIGH);
		digitalWrite(RIGHT_FORWARD_PIN, HIGH);
		digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
		
		digitalWrite(LEFT_ENABLE_PIN, HIGH);
		digitalWrite(RIGHT_ENABLE_PIN, HIGH);
		digitalWrite(LEFT_DISABLE_PIN, LOW);
		digitalWrite(RIGHT_DISABLE_PIN, LOW);
	}


	void controlMotors(long speedL, long speedR) {
		if (speedL<0) 
		{
			speedL = -speedL;
			digitalWrite(LEFT_FORWARD_PIN, HIGH);
			analogWrite(LEFT_BACKWARD_PIN, 255-speedL);
		}
		else
		{
			digitalWrite(LEFT_BACKWARD_PIN, HIGH);
			analogWrite(LEFT_FORWARD_PIN, 255-speedL);
		}

		if (speedR<0) 
		{
			speedR = -speedR;
			digitalWrite(RIGHT_FORWARD_PIN, HIGH);
			analogWrite(RIGHT_BACKWARD_PIN, 255-speedR);
		}
		else
		{
			digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
			analogWrite(RIGHT_FORWARD_PIN, 255-speedR);
		}
	}
#endif


#ifdef PWM_PWM_CONTROLLER
	void initMotors(void)
	{
		setPwmFrequency(HZ_3906);				// set PWM frequency to 3.906 kHz
		pinMode(MOTOR_ENABLE_PIN, OUTPUT);
		pinMode(LEFT_BACKWARD_PIN, OUTPUT);		// set left motor pin to output
		pinMode(LEFT_FORWARD_PIN, OUTPUT);		// set left motor pin to output
		pinMode(RIGHT_BACKWARD_PIN, OUTPUT);	// set right motor pin to output
		pinMode(RIGHT_FORWARD_PIN, OUTPUT);		// set right motor pin to output
		analogWrite(LEFT_FORWARD_PIN, 0);		// stop left motor
		analogWrite(LEFT_BACKWARD_PIN, 0);		// stop left motor
		analogWrite(RIGHT_FORWARD_PIN, 0);		// stop right motor
		analogWrite(RIGHT_BACKWARD_PIN, 0);		// stop right motor
		digitalWrite(LEFT_FORWARD_PIN, LOW);
		digitalWrite(LEFT_BACKWARD_PIN, LOW);
		digitalWrite(RIGHT_FORWARD_PIN, LOW);
		digitalWrite(RIGHT_BACKWARD_PIN, LOW);
		digitalWrite(MOTOR_ENABLE_PIN, HIGH);
	}


	void controlMotors(long speedL, long speedR) {
		if (speedL<0) 
		{
			speedL = -speedL;
			analogWrite(LEFT_FORWARD_PIN, 0);
			analogWrite(LEFT_BACKWARD_PIN, speedL);
		}
		else
		{
			analogWrite(LEFT_BACKWARD_PIN, 0);
			analogWrite(LEFT_FORWARD_PIN, speedL);
		}

		if (speedR<0) 
		{
			speedR = -speedR;
			analogWrite(RIGHT_FORWARD_PIN, 0);
			analogWrite(RIGHT_BACKWARD_PIN, speedR);
		}
		else
		{
			analogWrite(RIGHT_BACKWARD_PIN, 0);
			analogWrite(RIGHT_FORWARD_PIN, speedR);
		}
	}
#endif


#ifdef OSMC_CONTROLLER
	void initMotors(void)
	{
		setPwmFrequency(HZ_3906);				// set PWM frequency to 3.906 kHz
		pinMode(MOTOR_DISABLE_PIN, OUTPUT);
		pinMode(LEFT_BACKWARD_PIN, OUTPUT);		// set left motor pin to output
		pinMode(LEFT_FORWARD_PIN, OUTPUT);		// set left motor pin to output
		pinMode(RIGHT_BACKWARD_PIN, OUTPUT);	// set right motor pin to output
		pinMode(RIGHT_FORWARD_PIN, OUTPUT);		// set right motor pin to output
		analogWrite(LEFT_FORWARD_PIN, 0);		// stop left motor
		analogWrite(LEFT_BACKWARD_PIN, 0);		// stop left motor
		analogWrite(RIGHT_FORWARD_PIN, 0);		// stop right motor
		analogWrite(RIGHT_BACKWARD_PIN, 0);		// stop right motor
		digitalWrite(LEFT_FORWARD_PIN, LOW);
		digitalWrite(LEFT_BACKWARD_PIN, LOW);
		digitalWrite(RIGHT_FORWARD_PIN, LOW);
		digitalWrite(RIGHT_BACKWARD_PIN, LOW);
		digitalWrite(MOTOR_DISABLE_PIN, LOW);
	}


	void controlMotors(long speedL, long speedR) {
		if (speedL<0) 
		{
			speedL = -speedL;
			analogWrite(LEFT_FORWARD_PIN, 0);
			analogWrite(LEFT_BACKWARD_PIN, speedL);
		}
		else
		{
			analogWrite(LEFT_BACKWARD_PIN, 0);
			analogWrite(LEFT_FORWARD_PIN, speedL);
		}

		if (speedR<0) 
		{
			speedR = -speedR;
			analogWrite(RIGHT_FORWARD_PIN, 0);
			analogWrite(RIGHT_BACKWARD_PIN, speedR);
		}
		else
		{
			analogWrite(RIGHT_BACKWARD_PIN, 0);
			analogWrite(RIGHT_FORWARD_PIN, speedR);
		}
	}
#endif


#ifdef PWM_PWM_15KHZ_CONTROLLER
	void initMotors(void)
	{
		setPwmFrequency(HZ_31250);				// set PWM frequency to 31250/2 = 15625 kHz (divide by 2 when using TOP=0x3FF)
		pinMode(MOTOR_ENABLE_PIN, OUTPUT);
		pinMode(LEFT_BACKWARD_PIN, OUTPUT);		// set left motor pin to output
		pinMode(LEFT_FORWARD_PIN, OUTPUT);		// set left motor pin to output
		pinMode(RIGHT_BACKWARD_PIN, OUTPUT);	// set right motor pin to output
		pinMode(RIGHT_FORWARD_PIN, OUTPUT);		// set right motor pin to output
		digitalWrite(MOTOR_ENABLE_PIN, HIGH);

		// init 16bit timer 3
		TCCR3B &= ~(1<<WGM33);
		TCCR3B |= (1<<WGM32);
		TCCR3A |= (1<<WGM31); // fast PWM, TOP=0x3FF
		TCCR3A |= (1<<WGM30);


		// init 16bit timer 4
		TCCR4B &= ~(1<<WGM43);
		TCCR4B |= (1<<WGM42);
		TCCR4A |= (1<<WGM41); // fast PWM, TOP=0x3FF
		TCCR4A |= (1<<WGM40);



		TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B
		TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
		TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A

		TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
//		TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
//		TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
		
	}

	

	void controlMotors(long speedL, long speedR) 
	{
		if (speedL<0) 
		{
			speedL = -speedL;
			OCR4A = 0;	// pin 6
			OCR3A = speedL; //  pin 5
		}
		else
		{
			OCR3A = 0;	// pin 5
			OCR4A = speedL; //  pin 6
		}

		if (speedR<0) 
		{
			speedR = -speedR;
			OCR3C = 0;	// pin 3
			OCR3B = speedR; //  pin 2
		}
		else
		{
			OCR3B = 0;	// pin 2
			OCR3C = speedR; //  pin 3
		}
                      
			
//        OCR4B = motor[4]<<3; //  pin 7
//        OCR4C = motor[5]<<3; //  pin 8
//        OCR2B = motor[6]>>3; //  pin 9
//        OCR2A = motor[7]>>3; //  pin 10

	}

#endif



#ifdef PWM_DIR_CONTROLLER
	void initMotors(void)
	{
		setPwmFrequency(HZ_3906);				// set PWM frequency to 3.906 kHz
		pinMode(MOTOR_ENABLE_PIN, OUTPUT);
		pinMode(LEFT_PWM_PIN, OUTPUT);			// set left pwm pin to output
		pinMode(LEFT_DIR_PIN, OUTPUT);			// set left direction pin to output
		pinMode(RIGHT_PWM_PIN, OUTPUT);			// set right pwm pin to output
		pinMode(RIGHT_DIR_PIN, OUTPUT);			// set right direction pin to output
		digitalWrite(LEFT_PWM_PIN, LOW);		// stop left motor
		digitalWrite(RIGHT_PWM_PIN, LOW);		// stop right motor
		digitalWrite(MOTOR_ENABLE_PIN, HIGH);
	}


	void controlMotors(long speedL, long speedR) {
		if (speedL<0) 
		{
			speedL = -speedL;
			digitalWrite(LEFT_DIR_PIN, LOW);
		}
		else
		{
			digitalWrite(LEFT_DIR_PIN, HIGH);
		}
		analogWrite(LEFT_PWM_PIN, speedL);

		if (speedR<0) 
		{
			speedR = -speedR;
			digitalWrite(RIGHT_DIR_PIN, LOW);
		}
		else
		{
			digitalWrite(RIGHT_DIR_PIN, HIGH);
		}
		analogWrite(RIGHT_PWM_PIN, speedR);
	}
#endif

#if defined(ROBOCLAW_CONTROLLER) || defined (ROBOCLAW_ENCODER_CONTROLLER)
	#ifndef MEGA
	SoftwareSerial Serial1(SOFT_RX_PIN, SOFT_TX_PIN); // RX, TX
	#endif

	int readCurrents(void)
	{
		byte receive[5];
		byte send[2];
		send[0] = ROBOCLAW_ADDRESS;
		send[1] = READ_CURRENTS;
		Serial1.write(send, sizeof(send));
		
		byte timeout = 30; //set timeout to 30x100us or 3ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (!timeout)
			return -1;
		unsigned char command=Serial1.read();	// read current1 MSB
		current1 = command << 8;

		timeout = 20; //set timeout to 2ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (!timeout)
			return -1;
		command=Serial1.read();					// read current1 LSB
		current1 = current1 + command;

		timeout = 10; //set timeout to 1ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (!timeout)
			return -1;
		command=Serial1.read();					// read current2 MSB
		current2 = command << 8;

		timeout = 10; //set timeout to 2ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (!timeout)
			return -1;
		command=Serial1.read();					// read current2 LSB
		current2 = current2 + command;
		
		
		timeout = 10; //set timeout to 1ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (timeout)
			command=Serial1.read();				// read checksum
		return current2;
		
	}


	float readTemperature(void)
	{
		byte receive[3];
		byte send[2];
		send[0] = ROBOCLAW_ADDRESS;
		send[1] = READ_TEMP;
		Serial1.write(send, sizeof(send));
		
		byte timeout = 30; //set timeout to 30x100us or 3ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (!timeout)
			return -1;
		unsigned char command=Serial1.read();	// read temp MSB
		int temp = command << 8;

		timeout = 20; //set timeout to 2ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (!timeout)
			return -1;
		command=Serial1.read();					// read temp LSB
		temp = temp + command;

		timeout = 10; //set timeout to 1ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (timeout)
			command=Serial1.read();				// read checksum
		return temp/10.0;
		
	}

	float readVoltage(void)
	{
		//return 24.68;
		byte receive[3];
		byte send[2];
		send[0] = ROBOCLAW_ADDRESS;
		send[1] = READ_VOLT;
		Serial1.write(send, sizeof(send));
		
		byte timeout = 30; //set timeout to 3ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (!timeout)
			return -1;
		unsigned char command=Serial1.read();	// read voltage MSB
		int volt = command << 8;

		timeout = 30; //set timeout to 2ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (!timeout)
			return -1;
		command=Serial1.read();					// read voltage LSB
		volt = volt + command;

		timeout = 30; //set timeout to 1ms
		while (!Serial1.available() && timeout > 0)
		{
              delayMicroseconds(100);
              timeout--;
		} // wait
		if (timeout)
			command=Serial1.read();				// read checksum
		return volt/10.0;
		
	}

#endif

#ifdef ROBOCLAW_CONTROLLER
	void sendCommand7(byte command, int param1, unsigned short param2)
	{
		byte packet[7];
		unsigned short acceleration=32767;

		packet[0] = ROBOCLAW_ADDRESS;
		packet[1] = command;
		packet[2] = param1 >> 8; // duty MSB
		packet[3] = param1 & 0xFF; // duty LSB
		packet[4] = param2 >> 8; // accel MSB
		packet[5] = param2 & 0xFF; // accel LSB
		packet[6] = (packet[0]+packet[1]+packet[2]+packet[3]+packet[4]+packet[5]+packet[6]) & 0x7F;
		Serial1.write(packet, sizeof(packet));
	}
	
	
	void sendCommand5(byte command, int param)
	{
		byte packet[5];
		packet[0] = ROBOCLAW_ADDRESS;
		packet[1] = command;
		packet[2] = param >> 8;		// MSB
		packet[3] = param & 0xFF;	// LSB
		packet[4] = (packet[0]+packet[1]+packet[2]+packet[3]) & 0x7F;
		Serial1.write(packet, sizeof(packet));
	}

#ifdef ROBOCLAW_ACCELERATION
	void setSpeedMotor1(int speed)
	{
		sendCommand7(M1_ACCEL, speed, ROBOCLAW_ACCELERATION);
	}

	void setSpeedMotor2(int speed)
	{
		sendCommand7(M2_ACCEL, speed, ROBOCLAW_ACCELERATION);
	}
#else	
	void setSpeedMotor1(int speed)
	{
		sendCommand5(M1_SPEED, speed);
	}

	void setSpeedMotor2(int speed)
	{
		sendCommand5(M2_SPEED, speed);
	}
#endif

	void controlMotors(long speedL, long speedR) {
		setSpeedMotor1((int) speedL);
		setSpeedMotor2((int) speedR);
	}

	void initMotors(void)
	{
		Serial1.begin(38400);			// initialize serial port

		//stop motors							// in case enable pin is not used, stop motors without waiting
		controlMotors(0, 0);
		delay(2000);							// give Roboclaw time to boot up
		//stop motors
		controlMotors(0, 0);					// send stop command again, in case it wasn't ready before
	}
#endif

#ifdef ROBOCLAW_ENCODER_CONTROLLER
	void sendCommand(byte command, long param)
	{
		byte packet[7];
		packet[0] = ROBOCLAW_ADDRESS;
		packet[1] = command;
		packet[2] = param >> 24;	// MSB
		packet[3] = param >> 16;	// 2nd MSB
		packet[4] = param >> 8;		// 3rd MSB
		packet[5] = param & 0xFF;	// LSB
		packet[6] = (packet[0]+packet[1]+packet[2]+packet[3]+packet[4]+packet[5]) & 0x7F;
		Serial1.write(packet, sizeof(packet));
	}

	void setSpeedMotor1(long speed)
	{
		sendCommand(M1_SPEED, speed);
	}

	void setSpeedMotor2(long speed)
	{
		sendCommand(M2_SPEED, speed);
	}

	void controlMotors(long speedL, long speedR) {
		setSpeedMotor1(speedL);
		setSpeedMotor2(speedR);
	}

	void initMotors(void)
	{
		Serial1.begin(38400);			// initialize serial port

		//stop motors							// in case enable pin is not used, stop motors without waiting
		controlMotors(0L, 0L);
		delay(2000);							// give Roboclaw time to boot up
		//stop motors
		controlMotors(0L, 0L);					// send stop command again, in case it wasn't ready before
	}
#endif

#ifdef SABERTOOTH_CONTROLLER
	#ifndef MEGA
	SoftwareSerial Serial1(SOFT_RX_PIN, SOFT_TX_PIN); // RX, TX
	#endif

	void sendCommand(byte command, int param)
	{
		byte packet[4];
		packet[0] = SABERTOOTH_ADDRESS;
		packet[1] = command;
		packet[2] = param;
		packet[3] = (packet[0]+packet[1]+packet[2]) & 0x7F;
		Serial1.write(packet, sizeof(packet));
	}

	void setSpeedMotor1(int speed)
	{
		if (speed >= 0)
			sendCommand(M1_FORWARD, speed);
		else
			sendCommand(M1_BACKWARD, -speed);
	}

	void setSpeedMotor2(int speed)
	{
		if (speed >= 0)
			sendCommand(M2_FORWARD, speed);
		else
			sendCommand(M2_BACKWARD, -speed);
	}

	void initMotors(void)
	{
		Serial1.begin(9600);			// initialize serial port at 9600
		//stop motors
		controlMotors(0, 0);		
	}


	void controlMotors(long speedL, long speedR) {
		setSpeedMotor1((int) speedL);
		setSpeedMotor2((int) speedR);
	}
#endif
