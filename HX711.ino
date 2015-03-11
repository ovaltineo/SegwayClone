#ifdef HX711_DOUT
#include <HX711.h>

HX711 hx711(HX711_DOUT, HX711_SCK);


boolean checkHX711(void)
{
	static boolean last_result = false;
	static boolean get_weight = false;
	static unsigned long start_millis = 0;
	static unsigned long curr_millis = 0;
	if (hx711.is_ready())
	{
		weight = hx711.get_units();
		if (!last_result && (weight >= HX711_MIN))	// check if rider has just hopped on
		{
			get_weight = true;						// set flag to sample the weight after 3 seconds
			start_millis = millis();				// remember the time now
		}
		else if (weight < HX711_MIN)
			get_weight = false;
			
		last_result = weight >= HX711_MIN;  // return TRUE if weight > minimum threshold
	}
	if (get_weight)									// do we need to sample the weight?
	{
			curr_millis = millis();
			if ((curr_millis - start_millis) >= 3000)	// check if 3 seconds has elapsed since rider has hopped on
			{
				sample_weight = weight;					// get sample weight
				get_weight = false;						// turn off the flag
			}
	}
	return last_result;
}

void initHX711(void)
{
Serial.println("Start init HX711");
	hx711.set_scale(2280.f);            // this value is obtained by calibrating the scale with known weights; see the README for details
	hx711.tare();				        // reset the scale to 0
Serial.println("End init HX711");
}
#endif
