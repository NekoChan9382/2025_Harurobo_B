#include <FastLED.h>
#define NUM_LEDS 31
#define DATA_PIN 5
CRGB leds[NUM_LEDS];


bool gmx = false;
pinMode(6,INPUT_PULLUP); 

void setup() {
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(6) == 0){
    gmx = 1;
  }
  if(gmx){
    for(int i = 0;i < 32;i++){
      leds[i] = CRGB(0,0,255);
      FastLED.show();
      delay(20);
      leds[i] = CRGB::Black;
      FastLED.show();
      delay(20);
    }
  }
}
