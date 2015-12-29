/*
This is Ovaltineo's Roboclaw-test SegwayClone.h.  I must have forgotten to rename this before zipping the release.  
Please rename SegwayClone.h.generic to SegwayClone.h and use that instead.
*/

/*
Uncomment one MODE below
*/
//#define MODE_CALIBRATE
//#define MODE_RUN

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
#define STEER_Z_GYRO

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
//#define PWM_PWM_15KHZ_CONTROLLER
//#define PWM_DIR_CONTROLLER
//#define ROBOCLAW_CONTROLLER
//#define ROBOCLAW_ENCODER_CONTROLLER
#define ROBOCLAW_CRC_CONTROLLER
//#define ROBOCLAW_CRC_ENCODER_CONTROLLER
//#define SABERTOOTH_CONTROLLER
//#define PWM_SHIELD_CONTROLLER
//#define L298N_CONTROLLER

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
#define BLUETOOTH

/*
Uncomment if you have TC74 temperature sensors - TC74A0 and TC74A2 default addresses in TC74.h
*/
//#define TEMPERATURE_SENSORS

/*
Uncomment if you want the clone to be locked during startup.  It can only be unlocked via CloneConsole Android app.
*/
//#define LOCKED_BY_DEFAULT

/*
Uncomment if you have a rider sensor - sensor must pull this pin HIGH when rider is ON, and pull it LOW when rider is OFF
Initial values are loaded from KP_OFF, KD_OFF, and MOTOR_MAX_OFF.
*/
//#define RIDER_SENSOR	8

/*
Uncomment if RIDER_SENSOR above has reverse logic, ie LOW when rider is ON and HIGH when rider is OFF
*/
//#define INVERT_RIDER_SENSOR


//#define FAN_PIN			13				// optional output for MOSFET or relay fan switch

#define BUZZER_PIN			12
#define POT_STEERING_PIN	A1				
#define P_PIN				A0				// optional pot for adjusting P value (Proportional)
#define D_PIN				A2				// optional pot for adjusting D value (Derivative)
//#define POT_TILT_PIN	A4				// optional pot for adjusting forward/backward tilt

#ifdef VOLTAGE_CHECK
//	#define VOLTAGE_PIN			A3
	#define VOLTAGE_PIN			A5			// ovaltine's clone uses this
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


#define	ALPHA 0.97				// used by complementary filter
#define	ALPHA_SLOW 0.997				// used by slow complementary filter

#if defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER)
	// use these values if using Roboclaw
//	#define KP 0.96					// proportional constant when rider is ON
//	#define KD 1.2   				// derivative constant when rider is ON
//	#define STEER_MULTIPLIER 25     // liner - adjust depending on your motor speed
//	#define KP_OFF 0.67				// proportional constant when rider is OFF
//	#define KD_OFF .72 				// derivative constant when rider is OFF
	#define KP 15					// proportional constant when rider is ON
	#define KD 20   				// derivative constant when rider is ON
	#define STEER_MULTIPLIER 500    // liner - adjust depending on your motor speed
	#define KP_OFF 8				// proportional constant when rider is OFF
	#define KD_OFF 12 				// derivative constant when rider is OFF
#endif
#if defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
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
#if defined(PWM_PWM_CONTROLLER) || defined(PWM_DIR_CONTROLLER)  || defined(PWM_SHIELD_CONTROLLER) || defined(L298N_CONTROLLER)
	// use these values if using PWM
	#define KP .7					// proportional constant when rider is ON
	#define KD .3   				// derivative constant when rider is ON
	#define STEER_MULTIPLIER 3.5    // adjust depending on your motor speed
	#define KP_OFF .35				// proportional constant when rider is OFF
	#define KD_OFF .15 				// derivative constant when rider is OFF
#endif
#if defined(PWM_PWM_15KHZ_CONTROLLER)
	// use these values if using PWM
	#define KP 2.8					// proportional constant when rider is ON
	#define KD 1.2   				// derivative constant when rider is ON
	#define STEER_MULTIPLIER 14		// adjust depending on your motor speed
	#define KP_OFF 1.4				// proportional constant when rider is OFF
	#define KD_OFF .6 				// derivative constant when rider is OFF
#endif

#define MAX_ANGLE  18			// adjust depending on your machine
#define MIN_ANGLE  -15			// adjust depending on your machine
//#define MAX_YAW_RATE_RIDER_OFF	90	// max degrees per second (with rider off) change in direction before safety shutdown
//#define MAX_YAW_RATE_RIDER_ON	180	// max degrees per second (with rider on) change in direction before safety shutdown

#define CALIBRATE_YES	true
#define CALIBRATE_NO	false

#define ACC_FILTER	0.05
#define VOLT_FILTER     0.8
//#define lowpassFilter(val, change, q) ((1-q) * val + (change * q))
#define lowpassFilter(val, change, q) (val + q * (change - val))

#if defined(PWM_PWM_CONTROLLER) || defined(PWM_DIR_CONTROLLER) || defined(PWM_SHIELD_CONTROLLER) || defined (L298N_CONTROLLER)
	// bluetooth software serial pins for non-mega boards with PWM controllers
	#define SOFT_RX_PIN3	5			// connect to bluetooth module TX pin
	#define SOFT_TX_PIN3	6			// connect to bluetooth module RX pin
#else	
	// bluetooth software serial pins for non-mega boards with serial controllers
	#define SOFT_RX_PIN3	7			// connect to bluetooth module TX pin
	#define SOFT_TX_PIN3	8			// connect to bluetooth module RX pin
#endif	

#define STEER_OFFSET_DIVIDER	-70.0		// bluetooth remote control steer offset (X) divider.  Higher value makes X movement less sensitive.
#ifdef POT_TILT_PIN
	#define BOARD_OFFSET_DIVIDER	1.0		// pot tilt board offset divider.  Higher value makes pot tilt movement less sensitive.  Negative value reverses the direction.
#else
	#define BOARD_OFFSET_DIVIDER	-70.0		// bluetooth remote control board offset (Y) divider.  Higher value makes Y movement less sensitive.  Negative value reverses the direction.
#endif

#define POT_STEERING_ADJUST		0	// modify this to get the POT_STEERING angle centered to zero

#define HX711_DOUT	A7
#define HX711_SCK	A6
#define HX711_MIN	50			// min weight before IS_RIDER_ON returns true
#define HX711_REFERENCE_WEIGHT 940	// optional weight of rider where static P & D are based on.  If defined, P & D are computed proportional to rider's weight

#define SERIAL_WTV020			// WTV020 Voice module on Serial port 2

