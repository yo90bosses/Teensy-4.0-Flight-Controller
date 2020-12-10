




//#include <FastLED.h>



#define NUM_LEDS 15 
#define DATA_PIN 0


/*void fadeall(CRGB *leds) { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(250); } }




void ledloop(CRGB *leds) {

    static uint8_t hue = 0;
	Serial.print("x");
	// First slide the led in one direction
	for(int i = 0; i < NUM_LEDS; i++) {
		// Set the i'th led to red 
		leds[i] = CHSV(hue++, 255, 255);
		// Show the leds
		FastLED.show(); 
		// now that we've shown the leds, reset the i'th led to black
		// leds[i] = CRGB::Black;
		fadeall(leds);
		// Wait a little bit before we loop around and do it again
		delay(10);
	}
	Serial.print("x");

	// Now go in the other direction.  
	for(int i = (NUM_LEDS)-1; i >= 0; i--) {
		// Set the i'th led to red 
		leds[i] = CHSV(hue++, 255, 255);
		// Show the leds
		FastLED.show();
		// now that we've shown the leds, reset the i'th led to black
		// leds[i] = CRGB::Black;
		fadeall(leds);
		// Wait a little bit before we loop around and do it again
		delay(10);
	}

}*/



/*void LEDRuntime(void * pvParameters) {

    CRGB leds[NUM_LEDS];

    LEDS.addLeds<WS2812,DATA_PIN,RGB>(leds,NUM_LEDS);
	LEDS.setBrightness(84);

    while (true) {


        for(int i = (NUM_LEDS)-1; i >= 0; i--) {
            // Set the i'th led to red 
            leds[i] = CHSV(92, 255, byte((globalSystemSetting.totalForce)*255.0f));
            
        }

        FastLED.show();

        delay(10);


    }

}*/