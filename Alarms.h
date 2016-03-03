/*
AlarmArray
0: toggle
1: failsafe
2: noGPS
3: beeperOn
4: pMeter
5: runtime
6: vBat
7: confirmation
8: Acc
9: I2C Error
*/
/*
Resources:
0: onboard LED
1: Buzzer
2: PL GREEN
3: PL BLUE
4: PL RED
*/
#define BUZZERPIN_ON	digitalWrite(BUZZER_PIN, HIGH)
#define BUZZERPIN_OFF	digitalWrite(BUZZER_PIN, LOW)

#define ALARM_REVERSE		0
#define	ALARM_MAX_ANGLE		1
#define	ALARM_MAX_MOTOR		2
#define	ALARM_BATTERY		3
#define	ALARM_READY			4
#define ALARM_TEMP			5
#define ALARM_LOCKED		6
#define ALARM_I2C			7
#define ALARM_LEVEL			8
#define ALARM_RIDER			9

#define VOICE_BATTERY_1		101
#define VOICE_BATTERY_2		102
#define VOICE_BATTERY_3		103
#define VOICE_BATTERY_4		104
#define VOICE_MAX_ANGLE		105
#define VOICE_MAX_MOTOR		106
#define VOICE_MAX_TEMP		107
#define VOICE_I2C			108
#define VOICE_NOT_LEVEL		109
#define VOICE_LOCKED		110
#define VOICE_RIDER_ON		111
#define VOICE_RIDER_OFF		112

#define ALARM_GAP_MILLIS	4000

#define	RESOURCE_BUZZER		0
#define	RESOURCE_GREEN		1
#define	RESOURCE_BLUE		2
#define	RESOURCE_RED		3

#define MAX_TEMP		80
#define TEMP_FAN_ON		30
#define TEMP_FAN_OFF	25

#define MIN_BATTERY_1	24.0
#define MIN_BATTERY_2	23.0
#define MIN_BATTERY_3	22.0
#define MIN_BATTERY_4	21.0

