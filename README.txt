Segway* Clone by "Ovaltineo" for Arduino

*Segway is a trademark of Segway Inc.

A self-balancing two wheeled vehicle is inherently dangerous and may cause serious injury or
death.  You are NOT allowed to use this code unless you agree that the author bears no responsibility
or liability, directly or indirectly, for any injury, death, or damage to property caused by the use
of this code.

This is a program for a self-balancing two wheeled vehicle which was popularized and pioneered by the Segway.
Segway is a trademark of Segway Inc which holds many patents for this type of vehicle.  The term 
"Segway Clone" is used very loosely -- the code here is not a copy, in any shape or form, of the Segway firmware.
The real Segway is robust, well-engineered and has many safety features.  This "Segway Clone" is not.

This code is comprised of original and open source code from other authors.  It uses well known concepts 
of balancing an inverted pendulum by using a complementary filter for combining accelerometer and gyro 
data to measure the angle of tilt and a PID algorithm to apply a counter-acting motor-to-wheel movement 
in an attempt to bring the angle to zero.

v1.0	Original release
v1.1	Fixed bugs in original release
v2.0	Added support for Uno/Nano by using SoftwareSerial library.
		Added support for Roboclaw motor controller.
		Added support for single MPU6050 board tilt and steer angle sensor.
V2.1	Changed Roboclaw command to use -1500 to 1500 range
		Changed PWM pins for non-Mega boards
		Fixed #include Motors.h for Linux 
v2.2	Changed main loop cycle to 12.5ms when using Roboclaw.  This is required to prevent serial buffer overrun at 38400bps.
		Fixed check for MOTOR_MAX.
		Fixed mixup of S1 and S2 pins for Roboclaw in README
		Changed default MOTOR_MAX to very low values.  These new values should be just enough to balance the machine and not
		  cause too much damage if something goes wrong.  Increase MOTOR_MAX to 255 for PWM and 1500 for Roboclaw only if
		  everything is thoroughly tested.
v2.3	Fixed Roboclaw packet serial checksum bug which JR found.
		All axis calibration data are now saved & restored to/from EEPROM.
		Added support for GY80 which uses ADXL345 accelerometer and L3G4200D gyro.
		Added sensor orientation in the README - sensors must be aligned such that the X axis is level and pointing to the front of the vehicle.
v2.4	Fixed another bug in the calibration routine.  Gyro min and max offsets are now reset before calibration.
		Added debug output of steer angle.

v2.5	Found a bug in reading calibration data from EEPROM.  Looks like an Arduino compiler bug, fixed by using parenthesis!

v2.6	Reversed sign of Y accelerometer to fix inverted steering angle.
		Used new PID and STEER_MULTIPLIER values when using Roboclaw - copied from JR's values.
		
v2.7	Added Sabertooth support. Added patterned alarm (copied from Multiwii). Added pushback (from Phaedrus). Added voltage sensor support.
		
v2.8	Fixed Sabertooth compilation error. Fixed VOLTAGE_CHECK bug. Added battery voltage level LED support.

v2.9	Fixed compilation error. Fixed LED output initialization.

v3.0	Added "linvor" HC06 bluetooth module support -- this is required to use Android application.  
		Added TC74 temperature sensor support.  Fixed compilation error when using POT_STEERING.
		
v3.1	Removed I component of PID coz it's set to zero anyway.  Change analog pin for POT_STEERING to A1 coz A3 is used for voltage sensor. Removed ENABLE_PIN for
		Roboclaw to free up port for Bluetooth module when used with UNO.  You can only use Bluetooth module and BTN7960 together if you have an Arduino MEGA.

v3.2	Fixed compilation bugs when BLUETOOTH or TEMPERATURE_SENSORS is enabled.  

v3.3	Changed LED pins when using BTN7960 or other PWM-PWM/PWM-DIR controllers with an UNO board.  See new schematic.

v3.4	Changed bluetooth module name to SEGWAY during calibration.  Added MOTOR_ENABLE_PIN to PWM-DIR controller.

v3.5	Fixed code that will sound the buzzer continuously at the end of calibration.

v3.6	Saved and retrieved P and D values to/from EEPROM.  Added tilt and steer offset for remote control via bluetooth.  Reduced refresh rate for SABERTOOTH_CONTROLLER.

v3.7	Fixed compile bug - EEPROM_PD_ADDR should be EEPROM_PD_SAVED_ADDR.

v3.8	Fixed "remote control" bug - local steering is now disabled only when CloneConsole Android app is on "remote control" screen.

v3.9	Fixed integer conversion error when using POT_STEERING.  Added Bluetooth command to initiate calibration.  During startup, P and D values are retrieved from 
		EEPROM only if BLUETOOTH is enabled.

v4.0	Fixed a few bugs with communication interface to CloneConsole Android app. Fixed intermittent "Error 11" on I2C bus.
		Added new configuration parameters BOARD_OFFSET_DIVIDER and STEER_OFFSET_DIVIDER which specify CloneConsole remote control sensitivity.

v4.1	Fixed compile error when using Bluetooth with non-MEGA boards.  Added MOTOR_LR_RATIO (defined in Motors.h) to adjust power output between left and right motors.

v4.2	Disabled alarm if voltage is below 5V (powered by USB).  Added INVERT_STEERING option to swap left and right steering.  Added FAN_PIN option to turn on fan output
		when temperature goes above TEMP_FAN_ON.
		
v4.3	Added support for PWM dual-channel motor shield from Elechouse - select #define PWM_SHIELD_CONTROLLER in SegwayClone.h

v4.4	Changed to inverse PWM for PWM_SHIELD_CONTROLLER.  Added LOCKED_BY_DEFAULT option - unlock using CloneConsole Android app.

v4.5	Fixed LOCK bug. Added ALARM_LOCKED beeper indicator.

v4.6	Fixed FAN output bug.  Added ROBOCLAW_ENCODER_CONTROLLER to support a Roboclaw with quadrature encoders.

v4.7	Fixed compile error and multiplied Pf & Df by 10 when using ROBOCLAW_ENCODER_CONTROLLER.  

v4.8	Fixed compile error when BLUETOOTH is defined but not TEMPERATURE_SENSORS.  

v4.9	Fixed compile error when MIN_BATTERY_4 is decimal.  

v5.0	Added alarm for I2C error - long beep, short beep, long beep.  
		"Calibrate" via CloneConsole is now allowed regardless of motor speed when using USB power only (must have voltage sensor).
		Fixed compile error when using ROBOCLAW_CONTROLLER or SABERTOOTH_CONTROLLER.
		
v5.1	Changed serial baud rate from 38400 to 9600 for SABERTOOTH_CONTROLLER.

v5.2	Bluetooth module is now supported with UNO + PWM controllers - connect bluetooth HC-06 TX to pin 5 and RX to pin 6.  You can't enable
		VOLTAGE_LED with this configuration.
		
v5.3	Added support for RIDER_SENSOR.  Define the RIDER_SENSOR digital input pin, KP_OFF and KD_OFF for your controller in SegwayClone.h. 
		Define MOTOR_MAX_OFF for your controller in Motors.h.

v5.4	Added support for low power L298N motor controller - select #define L298N_CONTROLLER in SegwayClone.h
		Added option for INVERT_RIDER_SENSOR.  This inverts the logic of the RIDER_SENSOR pin.
		
v5.5	Fixed L298N motor controller code by removing "setPwmFrequency(HZ_3906);"		

v5.6	Changed bluetooth baud rate to 19200 for non-Mega boards. Non-Mega users must first switch to CALIBRATE_MODE to re-program the HC-06 module.

v5.7	Added software low-pass filter for voltage sensor.
		Added support for HX711 weight sensor.  
		Added RIDER_SENSOR to telemetry sent to CloneConsole.
		Added baud rate detection with bluetooth module.

v5.8	Added internal pull-up of RIDER_SENSOR pin. This has the effect of inverting the RIDER_SENSOR logic.  
		Sensor must now pull pin to GND if rider is ON and leave it open or HIGH if rider is OFF.  
		Added internal pull-up of RX3 pin on MEGA board when using bluetooth module.

v5.9	Fixed MAX_MOTOR for Roboclaw with encoder.  Added support for forward/backward tilt control by defining POT_TILT_PIN.

v6.0	Added center adjustment for POT_STEERING.

v7.0	Added voltage and temperature sensing through Roboclaw serial commands.  
		Replaced standard blocking Wire library with non-blocking I2C module.  Hopefully, this fixes locking of the Arduino!!!
		Added timeout when waiting for data from Bluetooth interface.
		Added support for WTV020 Voice module on Serial port #2.
		
v7.1	Gyro and accelerometer are now both used to compute steering angle (more stable at high speed).
		Added SUBTRACT_BANK_ANGLE option to compensate for terrain banking.
		
v7.2	Fixed inverted steering angle gyro and bank angle gyro.
		Added support for OSMC (Open Source Motor Controller) without MOB (Modular OSMC Brain) - #define OSMC_CONTROLLER.
		Added option for exponential steering response - #define EXPONENTIAL_STEER_DIVIDER

v7.3	Added option for acceleration with Roboclaw - #define ROBOCLAW_ACCELERATION in Motors.h

v7.4	Fixed compile problem by adding missing ALPHA_SLOW and SUBTRACT_BANK_ANGLE value in SegwayClone.h

v7.5	Support for new Roboclaw firmware which uses CRC for checksum (v4.1.11 and newer) -  use #define ROBOCLAW_CRC_CONTROLLER 
		or #define_CRC_ENCODER_CONTROLLER

v7.6	Added battery voltage decrease compensation.  Added CRC check for voltage, temperature, and current with Roboclaw controllers.

v7.7	Fixed CRC check for voltage, temperature, and current with Roboclaw controllers.

v7.8	Added alarm for rider sensor.  Fixed LED pins for Mega with L298N_CONTROLLER.
		
***********************************************************************************************************************************		
		
This code has the following features:
	Support for the following Arduino board types:
		Mega - required for Bluetooth + Android App
		Uno
		Nano
	Support for the following tilt sensor:
		MPU6050
		GY80 (ADXL345 accelerometer and L3G4200D gyro)
	Support for the following steering sensors:
		MPU6050
		GY80 (ADXL345 accelerometer and L3G4200D gyro)
		Potentiometer - connect to A1
	Support for the following motor controllers:
		PWM dual pin+enable, e.g. BTN7960/BTS9760.  You need an Arduino MEGA if you also want to install the Bluetooth module.
		PWM single pin+direction
		Roboclaw in packet serial mode
		Sabertooth in packet serial mode
		PWM 50A dual-channel motor shield.  http://www.elechouse.com/elechouse/index.php?main_page=product_info&products_id=2179
	Support for "linvor" HC06 bluetooth module.  
	Support for TC74 temperature sensors.  Default addresses are for TC74A0 and TC74A2.

There are three options with the MPU6050:
	Single sensor to measure board tilt only. Use this if you want to use a potentiometer as the steering sensor.
	   The I2C address for MPU6050 is 0x68 (AD0 low).
	Single sensor to measure board tilt and steering angle. You should mount the sensor on the steering shaft.
	   The I2C address for MPU6050 is 0x68 (AD0 low).
	Dual sensor - one to measure board tilt and a second to measure steering angle. One sensor is mounted on the 
	   board, and another sensor is mounted in the steering shaft. The I2C address for MPU6050 is 0x68 (AD0 low) 
	   for board tilt and 0x69 (AD0 high) for steering.

I am using two BTN7960 motor controllers, but the code should work with most motor controllers with dual PWM inputs.
	If you are using an Arduino Mega:
		Right motor PWM is output on digital pins 2 & 3. Swap the pins if the right motor is turning in the wrong direction.
		Left motor PWM is output on digital pins 5 & 6. Swap the pins if the left motor is turning in the wrong direction.
		Motor enable (connect to both left and right) output is on digital pin 7.
	else if you are using a UNO/Nano:
		Right motor PWM is output on digital pins 9 & 10. Swap the pins if the right motor is turning in the wrong direction.
		Left motor PWM is output on digital pins 3 & 11. Swap the pins if the left motor is turning in the wrong direction.
		Motor enable (connect to both left and right) output is on digital pin 7.
	
I have now added support for motor controllers with a single PWM and direction inputs.
	If you are using an Arduino Mega:
		Right motor PWM is output on digital pin 2
		Right motor DIR is output on digital pin 3		Do not swap with PWM pin. Swap the right controller motor output pins if the right motor is turning in the wrong direction.
		Left motor PWM is output on digital pin 5
		Left motor DIR is output on digital pin 6		Do not swap with PWM pin. Swap the left controller motor output pins if the left motor is turning in the wrong direction.
		Motor enable (connect to both left and right) output is on digital pin 7.
	else if you are using a UNO/Nano:
		Right motor PWM is output on digital pin 9
		Right motor DIR is output on digital pin 10		Do not swap with PWM pin. Swap the right controller motor output pins if the right motor is turning in the wrong direction.
		Left motor PWM is output on digital pin 3
		Left motor DIR is output on digital pin 11		Do not swap with PWM pin. Swap the left controller motor output pins if the left motor is turning in the wrong direction.
		Motor enable (connect to both left and right) output is on digital pin 7.
	
I have now added support for the Roboclaw motor controller. The Roboclaw must be configured for Packet Serial mode, address 0x80, 
and 38400 baud. There is no enable pin for the Roboclaw.  Hence, you must have KILL button that connects S3 to ground, or even better, kills the battery.

To use Roboclaw with a Mega, connect S1 to pin 18 (TX1), and S2 to pin 19 (RX1).  To use Roboclaw with an Uno/Nano, 
connect S1 to pin 6 (soft serial TX), and S2 to pin 5 (soft serial RX).   

I have also added support for the Sabertooth motor controller. The Sabertooth must be configured for Packet Serial mode, address 0x80, 
and 9600 baud. There is no enable pin for the Sabertooth.  Hence, you must have KILL button that connects S2 to ground, or even better, kills the battery.

To use Sabertooth with a Mega, connect S1 to pin 18 (TX1).  To use Sabertooth with an Uno/Nano, 
connect S1 to pin 6 (soft serial TX).  

I have tried to write a hardware abstraction layer so that all you need to do is replace MPU6050.ino with
your own module if using a different sensor.  You can also modify or replace motor.ino with your own module if 
using a non-supported motor controller.

Here are the hardware-dependent routines:
	void initBoardSensor(boolean calibrate)
	void initSteerSensor(boolean calibrate)
	float getBoardAccel(void)
	float getBoardGyro(void)
	float getSteerAccel(void)
	void initMotors(void)
	void controlMotors(int speedL, int speedR)

P is read from potentiometer on Analog pin 0 or from static constant KP.
D is read from potentiometer on Analog pin 2 or from static constant KD.

Beeper output is on digital pin 12.  Connect a small 5V beeper to this pin.  Beeper output is now patterned, depending on type of alarm:
    BOARD MAX_ANGLE    short long long
    BATTERY CRITICAL   long long long    <pause>
    BATTERY VERY LOW   long long         <pause>
    BATTERY LOW        long              <pause>
    MOTOR LIMIT        short short short
	I2C ERROR          long short long   <pause>
	LOCKED             short short       <pause>

Battery voltage (max 28.4 volts) can now be monitored by adding a resistor voltage divider across pin A3.  Like this: Battery----22K----A3----4.7K----Gnd.
You must uncomment #define VOLTAGE_CHECK in SegwayClone.h to enable this. LED Voltage indicator can be enabled if you uncomment #define VOLTAGE_LED in SegwayClone.h.
Place a Green LED+resistor on D9, Yellow LED+resistor on D10, and Red LED + resistor on D11.  MIN_BATTERY voltage levels can be set in Alarms.h.  LED display is like this:
	GREEN			above MIN_BATTERY1
	GREEN+YELLOW	below MIN_BATTERY1
	YELLOW			below MIN_BATTERY2
	YELLOW+RED		below MIN_BATTERY3
	RED				below MIN_BATTERY4

This code now incorporates Phaedrus' pushback algorithm. Pushback can be enabled if you uncomment #define PUSHBACK in SegwayClone.h.  Parameters for the 
pushback can be modified in Pushback.h.  

*** IMPORTANT **
You must edit SegwayClone.h to match your hardware. Uncomment one #define for STEERING and CONTROLLER.  
When first used, add/uncomment #define MODE_CALIBRATE and load the program to Arduino.  Level the board and 
center the steering and reset the Arduino.  There will be a 1 second beep before calibration and a continuous
beep after calibration.  Remove/comment #define MODE_CALIBRATE and load program to Arduino.  The beeper will 
sound until the board is levelled between +/- 2 degrees. When it stops beeping, the Segway clone will attempt 
to balance.

Sensors must be aligned such that the X axis is level and pointing to the front of the vehicle.
