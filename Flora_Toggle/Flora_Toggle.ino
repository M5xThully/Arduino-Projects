#define PINforControl   6 // pin connected to the small NeoPixels strip
#define NUMBER_PIXELS   188 // number of LEDs on second strip

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER_PIXELS, PINforControl, NEO_GRB + NEO_KHZ800);
//
#include <math.h>

#define N_PIXELS  188  // Number of pixels in strand
#define MIC_PIN   9  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define SAMPLE_WINDOW   5  // Sample window for average level
#define PEAK_HANG 16 //Time of pause before peak dot falls
#define PEAK_FALL .2 //Rate of falling peak dot
#define INPUT_FLOOR 100 //Lower range of analogRead input
#define INPUT_CEILING 500 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)
//

unsigned long patternInterval = 20 ; // time between steps in the pattern
unsigned long lastUpdate = 0 ; // for millis() when last update occoured
unsigned long intervals [] = { 20, 20, 20, 3, 10 } ; // speed for each pattern
const byte button = 10; // pin to connect button switch to between pin and ground
int pixels[NUMBER_PIXELS]; //array for lightning

byte peak = 16;      // Peak level of column; used for falling dots
unsigned int sample;

byte dotCount = 0;  //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot

void setup() {
  strip.begin(); // This initializes the NeoPixel library.
  strip.setBrightness(30); //adjust brightness here
  wipe(); // wipes the LED buffers
  pinMode(button, INPUT_PULLUP); // change pattern button
 //populates pixel array with random blue values for lightning storm
  for (int i=0; i<strip.numPixels();i++){
    int rc = random(0,128);
    pixels[i]=rc;
    }
}

void loop() {
  static int pattern = 0, lastReading;
  int reading = digitalRead(button);
  if(lastReading == HIGH && reading == LOW){
    pattern++ ; // change pattern number
    if(pattern > 4) pattern = 0; // wrap round if too big
    patternInterval = intervals[pattern]; // set speed for this pattern
    wipe(); // clear out the buffer 
    delay(50); // debounce delay
  }
  lastReading = reading; // save for next time

if(millis() - lastUpdate > patternInterval) updatePattern(pattern);
}

void updatePattern(int pat){ // call the pattern currently being created
  switch(pat) {
    case 0:
        strip.setBrightness(30); //adjust brightness here
        voiceCoil();
        break;
    case 1: 
        strip.setBrightness(27); //adjust brightness here
        rainbowCycle();
        break;
  }  
}

void voiceCoil() {
  unsigned long startMillis = millis(); // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;
  // collect data for length of sample window (in mS)
  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    sample = analogRead(MIC_PIN);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

  // Serial.println(peakToPeak);

  /*
    //Fill the strip with rainbow gradient
    for (int i = 0; i <= strip.numPixels() - 1; i++) {
      strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, 30, 150)));
    }
  */

  //Fill the strip with rainbow gradient
  for (int i = 0; i <= strip.numPixels() / 2; i++) {
    // left - works
    strip.setPixelColor(strip.numPixels() / 2 - i, Wheel(map(i, 0, strip.numPixels() / 2, 30, 150)));
    // right - maybe fixme
    strip.setPixelColor(i + strip.numPixels() / 2, Wheel(map(i, 0, strip.numPixels() / 2, 30, 150)));
  }

  //Scale the input logarithmically instead of linearly
  c = fscale(INPUT_FLOOR, INPUT_CEILING, strip.numPixels() / 2, 0, peakToPeak, 2);

  if (c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= strip.numPixels() / 2) { // Fill partial column with off pixels
    // left - okay
    drawLine(0, c, strip.Color(0, 0, 0));
    // right - maybe fixed?
    drawLine(strip.numPixels() - c, strip.numPixels(), strip.Color(0, 0, 0));
  }

  // Set the peak dot to match the rainbow gradient
  y = strip.numPixels() / 2 - peak;

  // left - okay
  //strip.setPixelColor(strip.numPixels() / 2 - (y - 1), Wheel(map(y, 0, strip.numPixels() / 2, 30, 150)));
  // right - maybe fixme
  //strip.setPixelColor(strip.numPixels() / 2 + y - 1, Wheel(map(y, 0, strip.numPixels() / 2, 30, 150)));

  strip.show();

  // Frame based peak dot animation
  if (dotHangCount > PEAK_HANG) { //Peak pause length
    if (++dotCount >= PEAK_FALL) { //Fall rate
      peak++;
      dotCount = 0;
    }
  }
  else {
    dotHangCount++;
  }
}

void rainbowCycle() { // modified from Adafruit example to make it a state machine
  static uint16_t j=0;
    for(int i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
  j++;
  if(j >= 256*5) j=0;
  lastUpdate = millis(); // time for next change to the display
}

void theaterChaseRainbow() { // modified from Adafruit example to make it a state machine
  static int j=0, q = 0;
  static boolean on = true;
  // for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
  //  for (int q=0; q < 3; q++) {
     if(on){
            for (int i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
             }
     }
      else {
           for (int i=0; i < strip.numPixels(); i=i+3) {
               strip.setPixelColor(i+q, 0);        //turn every third pixel off
                 }
      }
     on = !on; // toggel pixelse on or off for next time
      strip.show(); // display
      q++; // update the q variable
      if(q >=3 ){ // if it overflows reset it and update the J variable
        q=0;
        j++;
        if(j >= 256) j = 0;
      }
  lastUpdate = millis(); // time for next change to the display    
}

void wipe(){ // clear all LEDs
     for(int i=0;i<strip.numPixels();i++){
       strip.setPixelColor(i, strip.Color(0,0,0)); 
       }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 170 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


void lightning(){
  for (int i=0; i<strip.numPixels();i++){
      strip.setPixelColor(i,0,0,pixels[i]);   
  int lightning = random(1,200);
  if (lightning == 4){
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
  }
    if (pixels[i]%2==0){
      pixels[i]+=2;
    }else{
      pixels[i]-=2;
    }
    if (pixels[i]<1) pixels[i]=2;
    if (pixels[i]>128) pixels[i]=127;
    strip.show();
   }
   lastUpdate = millis();
}

void lightningRed(){
  for (int i=0; i<strip.numPixels();i++){
      strip.setPixelColor(i,pixels[i],0,0);   
  int lightning = random(1,200);
  if (lightning == 4){
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
  }
    if (pixels[i]%2==0){
      pixels[i]+=2;
    }else{
      pixels[i]-=2;
    }
    if (pixels[i]<1) pixels[i]=2;
    if (pixels[i]>128) pixels[i]=127;
    strip.show();
   }
   lastUpdate = millis();
}

void lightningGreen(){
  for (int i=0; i<strip.numPixels();i++){
      strip.setPixelColor(i,0,pixels[i],0);   
  int lightning = random(1,200);
  if (lightning == 4){
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
    strip.setPixelColor(i,255,255,255);
    strip.show();
    delay(3);
  }
    if (pixels[i]%2==0){
      pixels[i]+=2;
    }else{
      pixels[i]-=2;
    }
    if (pixels[i]<1) pixels[i]=2;
    if (pixels[i]>128) pixels[i]=127;
    strip.show();
   }
   lastUpdate = millis();
}

//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  for (int i = from; i <= to; i++) {
    strip.setPixelColor(i, c);
  }
}


float fscale( float originalMin, float originalMax, float newBegin, float
              newEnd, float inputValue, float curve) {

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
    Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution
    Serial.println();
  */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}


