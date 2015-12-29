// MOTOR_LR_RATIO defines the speed ratio between left and right motors.  Should be 1.0 if motor speeds are identical. 
// Should be > 1 (eg 1.1) if left motor is slower and < 1 (eg 0.9) if left motor is faster.
#define MOTOR_LR_RATIO	1.0

#if defined(PWM_PWM_CONTROLLER) || defined(PWM_DIR_CONTROLLER) || defined(PWM_SHIELD_CONTROLLER) || defined(L298N_CONTROLLER) || defined(PWM_PWM_15KHZ_CONTROLLER) || defined(OSMC_CONTROLLER)
	// PWM frequency selector
	#define HZ_31250		1
	#define	HZ_3906			2
	#define HZ_488			3
	#define HZ_122			4
	#define HZ_30			5
#endif

#ifdef L298N_CONTROLLER
	#define MOTOR_MAX		252				// Don't use 255 (100% duty cycle) as MAX for most PWM controllers -- they don't like it
	#define MOTOR_MAX_OFF	100				// MOTOR_MAX when rider sensor is OFF

	#define LEFT_ENA_PIN	3
	#define LEFT_IN1_PIN	11
	#define LEFT_IN2_PIN	2

	#define RIGHT_ENB_PIN	9
	#define RIGHT_IN3_PIN	10
	#define RIGHT_IN4_PIN	7
#endif

#ifdef PWM_SHIELD_CONTROLLER
//	#define MOTOR_MAX		252				// Don't use 255 (100% duty cycle) as MAX for most PWM controllers -- they don't like it
	#define MOTOR_MAX		30
	#define MOTOR_MAX_OFF	30				// MOTOR_MAX when rider sensor is OFF

	#define RIGHT_BACKWARD_PIN	11
	#define RIGHT_FORWARD_PIN	3
	#define RIGHT_ENABLE_PIN	2
	#define RIGHT_DISABLE_PIN	4

	#define LEFT_BACKWARD_PIN	10
	#define LEFT_FORWARD_PIN	9
	#define LEFT_ENABLE_PIN		8
	#define LEFT_DISABLE_PIN	7
#endif

#ifdef PWM_PWM_CONTROLLER
	#define MOTOR_MAX		252				// Don't use 255 (100% duty cycle) as MAX for most PWM controllers -- they don't like it
//	#define MOTOR_MAX		30
	#define MOTOR_MAX_OFF	30				// MOTOR_MAX when rider sensor is OFF

	// PWM pins for motor
	#ifdef MEGA	
		#define RIGHT_BACKWARD_PIN	2
		#define RIGHT_FORWARD_PIN	3
		#define LEFT_BACKWARD_PIN	5
		#define LEFT_FORWARD_PIN	6
		#define MOTOR_ENABLE_PIN	7
	#else
		#define RIGHT_BACKWARD_PIN	9
		#define RIGHT_FORWARD_PIN	10
		#define LEFT_BACKWARD_PIN	3
		#define LEFT_FORWARD_PIN	11
		#define MOTOR_ENABLE_PIN	7
	#endif
#endif

#ifdef PWM_PWM_15KHZ_CONTROLLER
	#define MOTOR_MAX		0x3FE			// Don't use 0x3FF (100% duty cycle) as MAX for most PWM controllers -- they don't like it
//	#define MOTOR_MAX		30
	#define MOTOR_MAX_OFF	120				// MOTOR_MAX when rider sensor is OFF

	// PWM pins for motor
	#ifdef MEGA	
		#define RIGHT_BACKWARD_PIN	2
		#define RIGHT_FORWARD_PIN	3
		#define LEFT_BACKWARD_PIN	5
		#define LEFT_FORWARD_PIN	6
		#define MOTOR_ENABLE_PIN	7
	#endif

	// PWM pins for motor using MEGA with broken pins 2&3
	#ifdef MEGA_BROKEN
		#define RIGHT_BACKWARD_PIN	7
		#define RIGHT_FORWARD_PIN	8
		#define LEFT_BACKWARD_PIN	5
		#define LEFT_FORWARD_PIN	6
		#define MOTOR_ENABLE_PIN	4
	#endif
#endif

#ifdef PWM_DIR_CONTROLLER
//	#define MOTOR_MAX		252				// Don't use 255 (100% duty cycle) as MAX for most PWM controllers -- they don't like it
	#define MOTOR_MAX		30
	#define MOTOR_MAX_OFF	30				// MOTOR_MAX when rider sensor is OFF

	// PWM and DIR pins for motor
	#ifdef MEGA	
		#define RIGHT_PWM_PIN		2
		#define RIGHT_DIR_PIN		3
		#define LEFT_PWM_PIN		5
		#define LEFT_DIR_PIN		6
		#define MOTOR_ENABLE_PIN	7
	#else	
		#define RIGHT_PWM_PIN		9
		#define RIGHT_DIR_PIN		10
		#define LEFT_PWM_PIN		3
		#define LEFT_DIR_PIN		11
		#define MOTOR_ENABLE_PIN	7
	#endif
#endif

#ifdef OSMC_CONTROLLER
//	#define MOTOR_MAX		252				// Don't use 255 (100% duty cycle) as MAX for most PWM controllers -- they don't like it
	#define MOTOR_MAX		30
	#define MOTOR_MAX_OFF	30				// MOTOR_MAX when rider sensor is OFF

	// PWM pins for motor
	#ifdef MEGA	
		#define RIGHT_BACKWARD_PIN	2
		#define RIGHT_FORWARD_PIN	3
		#define LEFT_BACKWARD_PIN	5
		#define LEFT_FORWARD_PIN	6
		#define MOTOR_DISABLE_PIN	7
	#else
		#define RIGHT_BACKWARD_PIN	9
		#define RIGHT_FORWARD_PIN	10
		#define LEFT_BACKWARD_PIN	3
		#define LEFT_FORWARD_PIN	11
		#define MOTOR_DISABLE_PIN	7
	#endif
#endif

#if defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER)
//	#define MOTOR_MAX		1500
//	#define MOTOR_MAX_OFF	250				// MOTOR_MAX when rider sensor is OFF
	#define MOTOR_MAX		32767
	#define MOTOR_MAX_OFF	8192				// MOTOR_MAX when rider sensor is OFF

	#define ROBOCLAW_ADDRESS 0x80

	// ROBOCLAW packet serial commands
	#define M1_SPEED		32
	#define M2_SPEED		33
	#define READ_TEMP		82
	#define READ_VOLT		24
	#define READ_CURRENTS	49
	#define M1_ACCEL		52
	#define M2_ACCEL		53

	//#define ROBOCLAW_ACCELERATION 32767		// optional acceleration, values 0-65535
	
	#ifndef MEGA
		// software serial pins for non-mega boards
		#define SOFT_RX_PIN	5			// connect to S2 pin
		#define SOFT_TX_PIN	6			// connect to S1 pin
	#endif
#endif

#if defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
//	#define MOTOR_MAX		40000
	#define MOTOR_MAX		10000
	#define MOTOR_MAX_OFF	10000		// MOTOR_MAX when rider sensor is OFF
 
	#define ROBOCLAW_ADDRESS 0x80

	// ROBOCLAW packet serial commands
	#define M1_SPEED	35
	#define M2_SPEED	36
	#define READ_TEMP	82
	#define READ_VOLT	24
	#define READ_CURRENTS	49

	#ifndef MEGA
		// software serial pins for non-mega boards
		#define SOFT_RX_PIN	5			// connect to S2 pin
		#define SOFT_TX_PIN	6			// connect to S1 pin
	#endif
#endif

#ifdef SABERTOOTH_CONTROLLER
//	#define MOTOR_MAX		127
	#define MOTOR_MAX		20
	#define MOTOR_MAX_OFF	20			// MOTOR_MAX when rider sensor is OFF

	#define SABERTOOTH_ADDRESS 0x80

	// ROBOCLAW packet serial commands
	#define M1_FORWARD	0
	#define M1_BACKWARD	1
	#define M2_FORWARD	4
	#define M2_BACKWARD	5
	#define SET_BAUD	15
	#define BAUD_38400	4

	#ifndef MEGA
		// software serial pins for non-mega boards
		#define SOFT_RX_PIN	5			// connect to S2 pin
		#define SOFT_TX_PIN	6			// connect to S1 pin
	#endif
#endif

#define MOTOR_ALARM_PERCENT 80
