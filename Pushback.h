#if defined(PWM_PWM_CONTROLLER) || defined (PWM_DIR_CONTROLLER) || defined (PWM_SHIELD_CONTROLLER) || defined (L298N_CONTROLLER) || defined(OSMC_CONTROLLER)
	#define kick_trigger 100 // the motor speed at which to initiate a kick event
	#define kick_trigger_off 50 // the motor speed at which to stop a kick event
	#define kick_max 20 // the maximum allowed boost level
	#define kick_slope_up 10 // slope of upward ramp
	#define kick_slope_down 1 // slope of downrard ramp
#endif
#if defined(PWM_PWM_15KHZ_CONTROLLER)
	#define kick_trigger 400 // the motor speed at which to initiate a kick event
	#define kick_trigger_off 200 // the motor speed at which to stop a kick event
	#define kick_max 80 // the maximum allowed boost level
	#define kick_slope_up 40 // slope of upward ramp
	#define kick_slope_down 4 // slope of downrard ramp
#endif
#if defined(ROBOCLAW_CONTROLLER) || defined(ROBOCLAW_CRC_CONTROLLER)
	#define kick_trigger 600 // the motor speed at which to initiate a kick event
	#define kick_trigger_off 295 // the motor speed at which to stop a kick event
	#define kick_max 120 // the maximum allowed boost level
	#define kick_slope_up 60 // slope of upward ramp
	#define kick_slope_down 6 // slope of downrard ramp
#endif
#if defined(ROBOCLAW_ENCODER_CONTROLLER) || defined(ROBOCLAW_CRC_ENCODER_CONTROLLER)
	#define kick_trigger 15600 // the motor speed at which to initiate a kick event
	#define kick_trigger_off 7800 // the motor speed at which to stop a kick event
	#define kick_max 3120 // the maximum allowed boost level
	#define kick_slope_up 1560 // slope of upward ramp
	#define kick_slope_down 156 // slope of downrard ramp
#endif
#if defined(SABERTOOTH_CONTROLLER)
	#define kick_trigger 50 // the motor speed at which to initiate a kick event
	#define kick_trigger_off 25 // the motor speed at which to stop a kick event
	#define kick_max 10 // the maximum allowed boost level
	#define kick_slope_up 5 // slope of upward ramp
	#define kick_slope_down 1 // slope of downrard ramp
#endif

#define kick_term_angle 0 // angle at which to cut short top plateau
#define kick_hold_time1 300 // max number of cycles to stay at kick_max
#define kick_hold_time2 200 // number of cycles to lock out after a kick event ends
