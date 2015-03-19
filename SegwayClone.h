/*
MODE_CALIBRATE		- calibrate the sensors and save the data to EEPROM.  You must level the board, reset the Arduino, and  wait for the continuous tone.
MODE_RUN			- loads calibration data from the EEPROM.  Tone will sound until board is level between +-2 degrees.  Then it is ready to ride.

POT_STEERING		- use a potentiometer on A1 for steering and an MPU6050 for board tilt
MPU6050_X2_STEERING	- use two MPU6050s, one for board tilt and a second one for steering
MPU6050_X1_STEERING	- one MPU6050. It is used to measure board tilt and steering
GY80_STEERING		- one GY80. It is used to measure board tilt and steering

PWM_PWM_CONTROLLER	- use dual pin PWM + enable pin motor controller, eg BTN7960
PWM_DIR_CONTROLLER	- use single pin PWM + direction pin motor controller  
ROBOCLAW_CONTROLLER	- use Roboclaw motor controller.  Roboclaw must be configured for packet serial mode 38400 baud with address 0x80
SABERTOOTH_CONTROLLER - use Sabertooth motor controller.  Sabertooth must be configured for packet serial mode 38400 baud with address 0x80
PWN_SHIELD_CONTROLLER - use Elechouse dual-channel motor shield.  http://www.elechouse.com/elechouse/index.php?main_page=product_info&products_id=2179
L298N_CONTROLLER	- use L298N motor controller with ENA, IN1, IN2, IN3, IN4, and ENB pins 
OSMC_CONTROLLER	    - use OSMC motor controller (without MOB) with ALI, BLI, and DIS pins 

CHECK_VOLTAGE		- define if voltage sensor (22K + 4.7K voltage divider resistor ) is connected to A3.  22K is in series with battery voltage, 4.7K goes to ground.
					  Battery----22K----A3----4.7K----Gnd

PUSHBACK			- define this if pushback above specified motor output is desired.  Change pushback parameters in Pushback.h.

BLUETOOTH			- define this if a HC06 "linvor" bluetooth module is attached to Serial3 (on Mega) or pin 7 RX & pin 8 TX (on UNO)

TEMPERATURE_SENSORS	- define this if TC74A0 and TC74A2 are attached to I2C bus

LOCKED_BY_DEFAULT	- define this if you want clone to be locked at power-up.  Needs CloneConsole Android app to unlock.

RIDER_SENSOR		- define this if rider sensor/switch is available. Sensor must pull this pin to GND/LOW when rider is ON, and leave it open or pull HIGH when rider is OFF.

INVERT_RIDER_SENSOR - This will invert the logic of the RIDER_SENSOR. RIDER_SENSOR pin is HIGH when rider is ON, and LOW when rider is OFF
*/

/*
Uncomment one MODE below
*/
//#define MODE_CALIBRATE
#define MODE_RUN

/*
Uncomment one STEERING type below
*/
//#define POT_STEERING
#define MPU6050_X2_STEERING
//#define MPU6050_X1_STEERING
//#define GY80_STEERING

//#define EXPONENTIAL_STEER_DIVIDER	7	// turns steering from linear to exponential - this specifies the angle where response is higher than linear

/*
Uncomment only if steer MPU6050 is vertically oriented
*/
//#define STEER_Z_GYRO

/*
Uncomment only if using MPU6050_X2 and you want to compensate for terrain banking
*/
#define SUBTRACT_BANK_ANGLE			1.0		// defining this will subtract board bank angle from steer angle.  Value (0 to 1) defines how much bank is used.

/*
Uncomment only if steering left and right are inverted
*/
//#define INVERT_STEERING

/*
Uncomment one CONTROLLER type below
*/
//#define PWM_PWM_CONTROLLER
//#define PWM_DIR_CONTROLLER
//#define ROBOCLAW_CONTROLLER
//#define ROBOCLAW_ENCODER_CONTROLLER
//#define SABERTOOTH_CONTROLLER
//#define PWM_SHIELD_CONTROLLER
//#define L298N_CONTROLLER
#define OSMC_CONTROLLER

/*
Uncomment if you want to check battery voltage.  See note above for resistor divider.
*/
//#define VOLTAGE_CHECK

/*
Uncomment if you want LED Voltage indicator.  You need VOLTAGE_CHECK with this.
*/
//#define VOLTAGE_LED

/*
Uncomment if you want Pushback
*/
//#define PUSHBACK

/*
Uncomment if you have a HC06 "linvor" bluetooth module
*/
//#define BLUETOOTH

/*
Uncomment if you have TC74 temperature sensors - TC74A0 and TC74A2 default addresses in TC74.h
*/
//#define TEMPERATURE_SENSORS

/*
Uncomment if you want the clone to be locked during startup.  It can only be unlocked via CloneConsole Android app.
*/
//#define LOCKED_BY_DEFAULT

/*
Uncomment if you have a rider sensor - sensor must pull this pin to GND/LOW when rider is ON, and leave it open or HIGH when rider is OFF.
Initial values are loaded from KP_OFF, KD_OFF, and MOTOR_MAX_OFF.
*/
//#define RIDER_SENSOR	8

/*
Uncomment if RIDER_SENSOR above has reverse logic, ie HIGH when rider is ON and LOW when rider is OFF
*/
//#define INVERT_RIDER_SENSOR

/*
Uncomment if you have a HX711 weight sensor instead of digital RIDER_SENSOR. HX711_DOUT and HX711_SCK define which pins are 
connected to HX711.  If weight is greater than HX711_MIN, then rider is ON, else rider is OFF.
Initial values are loaded from KP_OFF, KD_OFF, and MOTOR_MAX_OFF.
*/
//#define HX711_DOUT	A6
//#define HX711_SCK		A7
//#define HX711_MIN		50
//#define HX711_REFERENCE_WEIGHT 940	// optional weight of rider where static P & D are based on.  If defined, P & D are computed proportional to rider's weight


//#define FAN_PIN			13				// optional output for MOSFET or relay fan switch

#define BUZZER_PIN			12
#define POT_STEERING_PIN	A1				
#define P_PIN				A0				// optional pot for adjusting P value (Proportional)
#define D_PIN				A2				// optional pot for adjusting D value (Derivative)
//#define POT_TILT_PIN	A4				// optional pot for adjusting forward/backward tilt

#ifdef VOLTAGE_CHECK
	#define VOLTAGE_PIN			A3
//	#define VOLTAGE_PIN			A5			// ovaltine's clone uses this
	#define VOLTAGE_DIVIDER		.176		// voltage divider: 4.7K/(22K + 4.7K) = .176
#endif

// The following 9 lines were borrowed/copied from Multiwii
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif
#if defined(__AVR_ATmega32U4__) || defined(TEENSY20)
  #define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif

#ifdef VOLTAGE_LED
	#if defined(PWM_SHIELD_CONTROLLER)
		#define LED_GREEN_PIN		A0
		#define LED_YELLOW_PIN		5
		#define LED_RED_PIN			6
	#elif ((defined(PWM_PWM_CONTROLLER) || defined(PWM_DIR_CONTROLLER)) && !defined(MEGA))
		#define LED_GREEN_PIN		4
		#define LED_YELLOW_PIN		5
		#define LED_RED_PIN			6
	#else
		#define LED_GREEN_PIN		9
		#define LED_YELLOW_PIN		10
		#define LED_RED_PIN			11
	#endif
#endif

typedef struct
{
	float x_accel;
	float y_accel;
	float z_accel;
	float x_gyro;
	float y_gyro;
	float z_gyro;

	int x_accel_offset;
	int y_accel_offset;
	int z_accel_offset;

	int x_gyro_offset;
	int y_gyro_offset;
	int z_gyro_offset;

	int x_gyro_offset_min;
	int x_gyro_offset_max;
	int y_gyro_offset_min;
	int y_gyro_offset_max;
	int z_gyro_offset_min;
	int z_gyro_offset_max;

	int address;
} sensorType;


#define	ALPHA 0.97					// used by complementary filter
#define	ALPHA_SLOW 0.997			// used by slow complementary filter


#if defined(ROBOCLAW_CONTROLLER)
	// use these values if using Roboclaw
	#define KP 3.9					// proportional constant when rider is ON
	#define KD 1.5   				// derivative constant when rider is ON
	#define STEER_MULTIPLIER 25     // adjust depending on your motor speed
	#define KP_OFF 1.8				// proportional constant when rider is OFF
	#define KD_OFF .75 				// derivative constant when rider is OFF
#endif
#if defined(ROBOCLAW_ENCODER_CONTROLLER)
	// use these values if using Roboclaw with encoder
	#define KP 10.9					// proportional constant when rider is ON
	#define KD 4.68   				// derivative constant when rider is ON
	#define STEER_MULTIPLIER 546     // adjust depending on your motor speed
	#define KP_OFF 5.45				// proportional constant when rider is OFF
	#define KD_OFF 2.34 				// derivative constant when rider is OFF
#endif
#if defined(SABERTOOTH_CONTROLLER)
	// use these values if using Sabertooth
	#define KP .35					// proportional constant when rider is ON
	#define KD .15   				// derivative constant when rider is ON
	#define STEER_MULTIPLIER 1.75   // adjust depending on your motor speed
	#define KP_OFF .17				// proportional constant when rider is OFF
	#define KD_OFF .07 				// derivative constant when rider is OFF
#endif
#if defined(PWM_PWM_CONTROLLER) || defined(PWM_DIR_CONTROLLER)  || defined(PWM_SHIELD_CONTROLLER) || defined(L298N_CONTROLLER) || defined(OSMC_CONTROLLER)
	// use these values if using PWM
	#define KP .7					// proportional constant when rider is ON
	#define KD .3   				// derivative constant when rider is ON
	#define STEER_MULTIPLIER 3.5    // adjust depending on your motor speed
	#define KP_OFF .35				// proportional constant when rider is OFF
	#define KD_OFF .15 				// derivative constant when rider is OFF
#endif

#define MAX_ANGLE  18			// adjust depending on your machine
#define MIN_ANGLE  -15			// adjust depending on your machine
//#define MAX_YAW_RATE_RIDER_OFF	45	// max degrees per second (with rider off) change in direction before safety shutdown
//#define MAX_YAW_RATE_RIDER_ON	90	// max degrees per second (with rider on) change in direction before safety shutdown

#define CALIBRATE_YES	true
#define CALIBRATE_NO	false

#define ACC_FILTER	0.05
#define VOLT_FILTER	0.8

//#define lowpassFilter(val, change, q) ((1-q) * val + (change * q)) 
#define lowpassFilter(val, change, q) (val + q * (change - val))

#if defined(PWM_PWM_CONTROLLER) || defined(PWM_DIR_CONTROLLER) || defined(PWM_SHIELD_CONTROLLER) || defined (L298N_CONTROLLER) || defined(L298N_CONTROLLER)
	// bluetooth software serial pins for non-mega boards with PWM controllers
	#define SOFT_RX_PIN3	5			// connect to bluetooth module TX pin
	#define SOFT_TX_PIN3	6			// connect to bluetooth module RX pin
#else	
	// bluetooth software serial pins for non-mega boards with serial controllers
	#define SOFT_RX_PIN3	7			// connect to bluetooth module TX pin
	#define SOFT_TX_PIN3	8			// connect to bluetooth module RX pin
#endif	

#define STEER_OFFSET_DIVIDER	-70.0		// bluetooth remote control steer offset (X) divider.  Higher value makes X movement less sensitive. Negative value reverses the direction.

#ifdef POT_TILT_PIN
	#define BOARD_OFFSET_DIVIDER	1.0		// pot tilt board offset divider.  Higher value makes pot tilt movement less sensitive.  Negative value reverses the direction.
#else
	#define BOARD_OFFSET_DIVIDER	-70.0		// bluetooth remote control board offset (Y) divider.  Higher value makes Y movement less sensitive.  Negative value reverses the direction.
#endif

#define POT_STEERING_ADJUST		0	// modify this to get the POT_STEERING angle centered to zero


//#define SERIAL_WTV020			// WTV020 Voice module on Serial port 2

