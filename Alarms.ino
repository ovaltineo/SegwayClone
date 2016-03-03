// Alarm Pattern Handling
// -----------------------------
//
// From Multiwii
// Open Source / Public Domain
// 
// Modified by "Ovaltineo"
// for use on DIY Segway clone

static unsigned long lastAlarmTime[11] = {0,0,0,0,0,0,0,0,0,0,0};
static uint8_t cycleDone[5]={0,0,0,0,0}, 
               resourceIsOn[5] = {0,0,0,0,0};
static uint32_t LastToggleTime[5] ={0,0,0,0,0};
static int16_t  i2c_errors_count_old = 0;

static uint8_t SequenceActive[5]={0,0,0,0,0};

uint8_t isBuzzerON() { return resourceIsOn[RESOURCE_BUZZER]; } // returns true while buzzer is buzzing; returns 0 for silent periods


void initAlarm()
{
	pinMode(BUZZER_PIN, OUTPUT);	// set buzzer pin as output
	digitalWrite(BUZZER_PIN, LOW);	// turn off buzzer

#ifdef VOLTAGE_LED
	pinMode(LED_GREEN_PIN, OUTPUT);	// set LED pin as output
	pinMode(LED_YELLOW_PIN, OUTPUT);	// set LED pin as output
	pinMode(LED_RED_PIN, OUTPUT);	// set LED pin as output
#endif
#ifdef SERIAL_WTV020
	Serial2.begin(115200);
#endif
}

void shortBeep()
{
 	BUZZERPIN_ON;
	delay(300);
	BUZZERPIN_OFF;
	delay(100);
}

void longBeep()
{
 	BUZZERPIN_ON;
	delay(1000);
	BUZZERPIN_OFF;
	delay(200);
}


void alarmHandler(){
	alarmPatternComposer();
}

void alarmPatternComposer(){ 
  static char resource = 0;
  
  // patternDecode(length1,length2,length3,beeppause,endpause,loop)
    resource = RESOURCE_BUZZER;																		//buzzer selected
    if (alarmArray[ALARM_MAX_ANGLE] == 1 || alarmArray[ALARM_LEVEL] == 1) 		patternDecode(resource,50,200,200,50,50);	// max angle
    else if (alarmArray[ALARM_BATTERY] == 4)	patternDecode(resource,200,200,200,50,1000);		//vbat critical
    else if (alarmArray[ALARM_BATTERY] == 3)	patternDecode(resource,200,200,0,50,1000);			//vbat very low
    else if (alarmArray[ALARM_BATTERY] == 2)	patternDecode(resource,200,0,0,50,1000);			//vbat low
    else if (alarmArray[ALARM_TEMP] == 1)		patternDecode(resource,100,100,1000,50,50);			//over temperature
    else if (alarmArray[ALARM_MAX_MOTOR] == 1)	patternDecode(resource,50,50,50,50,50);				//motor limit
    else if (alarmArray[ALARM_REVERSE] == 1)	patternDecode(resource,50,0,0,50,1100);				// reversing
    else if (alarmArray[ALARM_LOCKED] == 1)		patternDecode(resource,50,50,0,50,1100);			// locked
    else if (alarmArray[ALARM_I2C] == 1)		patternDecode(resource,200,50,200,50,1100);			// I2C ERROR
    else if (alarmArray[ALARM_RIDER] == 1)	patternDecode(resource,50,0,0,50,1000);					//rider on
    else if (alarmArray[ALARM_RIDER] == 2)	patternDecode(resource,200,0,0,50,1000);				//rider off
    else if (SequenceActive[(uint8_t)resource] == 1) patternDecode(resource,0,0,0,0,0);				// finish last sequence if not finished yet
    else turnOff(resource);																			// turn off the resource
#ifdef VOLTAGE_LED
	if (alarmArray[ALARM_BATTERY] == 0)			patternLED(1,0,0);  //G
	else if (alarmArray[ALARM_BATTERY] == 1)	patternLED(1,1,0);  //G, Y
	else if (alarmArray[ALARM_BATTERY] == 2)	patternLED(0,1,0);  //Y
	else if (alarmArray[ALARM_BATTERY] == 3)	patternLED(0,1,1);  //Y, R
	else if (alarmArray[ALARM_BATTERY] == 4)	patternLED(0,0,1);  //R
#endif
#ifdef  SERIAL_WTV020
	unsigned long currMillis;

	currMillis = millis();
	if (alarmArray[ALARM_RIDER] == 1 && currMillis-lastAlarmTime[ALARM_RIDER] > ALARM_GAP_MILLIS)
	{
		Serial2.write(VOICE_RIDER_ON);
		lastAlarmTime[ALARM_RIDER] = currMillis;
	} else if (alarmArray[ALARM_RIDER] == 2 && currMillis-lastAlarmTime[ALARM_RIDER] > ALARM_GAP_MILLIS)
	{
		Serial2.write(VOICE_RIDER_OFF);
		lastAlarmTime[ALARM_RIDER] = currMillis;
	}
	if (alarmArray[ALARM_LEVEL] == 1 && currMillis-lastAlarmTime[ALARM_LEVEL] > ALARM_GAP_MILLIS)
	{
		Serial2.write(VOICE_NOT_LEVEL);
		lastAlarmTime[ALARM_LEVEL] = currMillis;
	}
	if (alarmArray[ALARM_MAX_ANGLE] == 1 && currMillis-lastAlarmTime[ALARM_MAX_ANGLE] > ALARM_GAP_MILLIS)
	{
		Serial2.write(VOICE_MAX_ANGLE);
		lastAlarmTime[ALARM_MAX_ANGLE] = currMillis;
	}
	if (alarmArray[ALARM_TEMP] == 1 && currMillis-lastAlarmTime[ALARM_TEMP] > ALARM_GAP_MILLIS)
	{
		Serial2.write(VOICE_MAX_TEMP);
		lastAlarmTime[ALARM_TEMP] = currMillis;
	}
	if (alarmArray[ALARM_MAX_MOTOR] == 1 && currMillis-lastAlarmTime[ALARM_MAX_MOTOR] > ALARM_GAP_MILLIS)
	{
		Serial2.write(VOICE_MAX_MOTOR);
		lastAlarmTime[ALARM_MAX_MOTOR] = currMillis;
	}
	if (alarmArray[ALARM_LOCKED] == 1 && currMillis-lastAlarmTime[ALARM_LOCKED] > ALARM_GAP_MILLIS)
	{
		Serial2.write(VOICE_LOCKED);
		lastAlarmTime[ALARM_LOCKED] = currMillis;
	}
	if (alarmArray[ALARM_I2C] == 1 && currMillis-lastAlarmTime[ALARM_I2C] > ALARM_GAP_MILLIS)
	{
		Serial2.write(VOICE_I2C);
		lastAlarmTime[ALARM_I2C] = currMillis;
	}
	if (alarmArray[ALARM_BATTERY] != 0 && currMillis-lastAlarmTime[ALARM_BATTERY] > ALARM_GAP_MILLIS)
	{
		if (alarmArray[ALARM_BATTERY] == 4)		Serial2.write(VOICE_BATTERY_4);			//vbat critical
		else if (alarmArray[ALARM_BATTERY] == 3)	Serial2.write(VOICE_BATTERY_3);			//vbat very low
		else if (alarmArray[ALARM_BATTERY] == 2)	Serial2.write(VOICE_BATTERY_2);			//vbat low
		else if (alarmArray[ALARM_BATTERY] == 1)	Serial2.write(VOICE_BATTERY_1);			//vbat a little low
		lastAlarmTime[ALARM_BATTERY] = currMillis;
	}
#endif
}

#ifdef VOLTAGE_LED
void patternLED(int g, int y, int r)
{
	digitalWrite(LED_GREEN_PIN, g);
	digitalWrite(LED_YELLOW_PIN, y);
	digitalWrite(LED_RED_PIN, r);
}
#endif

void patternDecode(uint8_t resource,uint16_t first,uint16_t second,uint16_t third,uint16_t cyclepause, uint16_t endpause){
  static uint16_t pattern[5][5];
  static uint8_t icnt[5] = {0,0,0,0,0};
  
  if(SequenceActive[resource] == 0){
    SequenceActive[resource] = 1; 
    pattern[resource][0] = first; 
    pattern[resource][1] = second;
    pattern[resource][2] = third;
    pattern[resource][3] = endpause;
    pattern[resource][4] = cyclepause;
  }
  if(icnt[resource] <3 ){
    if (pattern[resource][icnt[resource]] != 0){
      setTiming(resource,pattern[resource][icnt[resource]],pattern[resource][4]);
     }
  }
  else if (LastToggleTime[resource] < (millis()-pattern[resource][3]))  {  //sequence is over: reset everything
    icnt[resource]=0;
    SequenceActive[resource] = 0;                               //sequence is now done, cycleDone sequence may begin
    alarmArray[ALARM_MAX_MOTOR] = 0;                                //reset toggle bit
    alarmArray[ALARM_MAX_ANGLE] = 0;                                //reset toggle bit
    alarmArray[ALARM_REVERSE] = 0;                                //reset toggle bit
    alarmArray[ALARM_BATTERY] = 0;                                //reset toggle bit
    alarmArray[ALARM_TEMP] = 0;                                //reset toggle bit
    alarmArray[ALARM_LOCKED] = 0;                                //reset toggle bit
    alarmArray[ALARM_I2C] = 0;                                //reset toggle bit
    alarmArray[ALARM_LEVEL] = 0;                                //reset toggle bit
    alarmArray[ALARM_RIDER] = 0;                                //reset toggle bit
    turnOff(resource);   
    return;
  }
  if (cycleDone[resource] == 1 || pattern[resource][icnt[resource]] == 0) {            //single on off cycle is done
    if (icnt[resource] < 3) {
      icnt[resource]++;
    }
    cycleDone[resource] = 0;
    turnOff(resource);    
  }  
}

void turnOff(uint8_t resource){
  if (resource == RESOURCE_BUZZER) {
    if (resourceIsOn[RESOURCE_BUZZER]) {
      BUZZERPIN_OFF;
      resourceIsOn[RESOURCE_BUZZER] = 0;
    }
  }
}


/********************************************************************/
/****                   Global Resource Handling                 ****/
/********************************************************************/

  void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause){
    if (!resourceIsOn[resource] && (millis() >= (LastToggleTime[resource] + pause))&& pulse != 0) {
      resourceIsOn[resource] = 1;      
      toggleResource(resource,1);
      LastToggleTime[resource]=millis();      
    } else if ( (resourceIsOn[resource] && (millis() >= LastToggleTime[resource] + pulse) ) || (pulse==0 && resourceIsOn[resource]) ) {
      resourceIsOn[resource] = 0;
      toggleResource(resource,0);
      LastToggleTime[resource]=millis();
      cycleDone[resource] = 1;     
    } 
  } 
 
  void toggleResource(uint8_t resource, uint8_t activate){
     switch(resource) {     
          case RESOURCE_BUZZER:
            if (activate == 1) {BUZZERPIN_ON;}
            else BUZZERPIN_OFF;
            break; 
      }
      return;
  }


