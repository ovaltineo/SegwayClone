// Pushback routine
// -----------------------------
//
// By www.diyelectriccar.com user "Phaedrus".
// Open Source / Public Domain
// 
// Modified by "Ovaltineo"
// for use on DIY Segway clone

#include "Pushback.h"

static boolean kick_flag = false; // set true at start of a kick event (the trigger)
static int kick_level = 0; // the level of kick, ranges 0 to kick_max
static int kick_stage = 1; // 1: ramping up, 2: hold, 3:ramping down, 4:lock out another kick
static int kick_timer = 0; // count of time to stay at top of ramp

int processPushback(int motor_out)
{
	if ((motor_out >= kick_trigger) && not kick_flag) 
	{
		kick_flag = true;
		kick_level = 0;
		kick_stage = 1;
	} 

	if (kick_flag) // *** we are in kick mode ***
	{
		alarmArray[ALARM_MAX_MOTOR] = 1;
		if (kick_stage == 1) // ramp up
		{
			kick_level = kick_level + kick_slope_up;
			if (kick_level > kick_max) 
				kick_level = kick_max;
			if (kick_level == kick_max) 
			{
				kick_stage = 2; 
				kick_timer = 0; 
			}
		}
		else if (kick_stage == 2) // hold at max
		{
			kick_timer = kick_timer + 1;
			if ((kick_timer >= kick_hold_time1) || (motor < kick_trigger_off)) 
			{
				kick_stage = 3; 
				kick_timer = 0; 
			}
		}
		else if (kick_stage == 3) // ramp down 
		{
			kick_level = kick_level - kick_slope_down;
			if (kick_level < 0)
				kick_level = 0;
			if (kick_level == 0) 
			{
				kick_stage = 4; 
				kick_timer = 0; 
			}
		}
		else if (kick_stage == 4) // lockout interval before kicking again
		{
			kick_timer = kick_timer + 1;
			if (kick_timer >= kick_hold_time2) 
			{ 
				kick_stage = 0; 
				kick_flag = false; 
			}
		}
	}
	return kick_level;
}