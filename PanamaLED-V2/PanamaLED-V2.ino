// --------------------------------
// -- VU Meter - Scott's version --
// --------------------------------

#include <FastLED.h>
#include <EEPROM.h>
#include <JC_Button.h>

# define MIC_GAIN 10                // Gain on mic signal
# define LEFT_OUT_PIN 2             // Left channel data out pin to LEDs [6]
# define RIGHT_OUT_PIN 4            // Right channel data out pin to LEDs [5]
# define LEFT_IN_PIN A0             // Left aux in signal [A5]
# define RIGHT_IN_PIN A0            // Right aux in signal [A4]
# define BTN_PIN D3                 // Push button on this pin [3]
# define DEBOUNCE_MS 20             // Number of ms to debounce the button [20]
# define LONG_PRESS 500             // Number of ms to hold the button to count as long press [500]
# define N_PIXELS 34                // Number of pixels in each string [24]
# define MAX_MILLIAMPS 500          // Maximum current to draw [500]
# define COLOR_ORDER GRB            // Colour order of LED strip [GRB]
# define LED_TYPE WS2812B           // LED string type [WS2812B]
# define NOISE 1                    // Noise/hum/interference in aux signal [10]
# define SAMPLES 60                 // Length of buffer for dynamic level adjustment [60]
# define TOP (N_PIXELS + 2)         // Allow dot to go slightly off scale [(N_PIXELS + 2)]
# define PEAK_FALL 20               // Rate of peak falling dot [20]
# define N_PIXELS_HALF (N_PIXELS / 2)
# define PATTERN_TIME 10            // Seconds to show eaach pattern on auto [10]
# define STEREO false               // If true, L&R channels are independent. If false, both L&R outputs display same data from L audio channel [false]

uint8_t volCountLeft = 0;           // Frame counter for storing past volume data
int volLeft[SAMPLES];               // Collection of prior volume samples
int lvlLeft = 0;                    // Current "dampened" audio level
int minLvlAvgLeft = 0;              // For dynamic adjustment of graph low & high
int maxLvlAvgLeft = 512;

uint8_t volCountRight = 0;          // Frame counter for storing past volume data
int volRight[SAMPLES];              // Collection of prior volume samples
int lvlRight = 0;                   // Current "dampened" audio level
int minLvlAvgRight = 0;             // For dynamic adjustment of graph low & high
int maxLvlAvgRight = 512;

CRGB ledsLeft[N_PIXELS];
CRGB ledsRight[N_PIXELS];

uint8_t myhue = 0;

void vu4(int center_mode, uint8_t channel);
void vu5(int center_mode, uint8_t channel);
void vu6(bool is_centered, uint8_t channel);
void vu7(bool show_background);
void vu8();
void vu9();
void vu10();
void balls();
void fire();
void juggle();
void ripple(boolean show_background);
void sinelon();
void twinkle();
void rainbow(uint8_t rate);

#define NUM_VISUALS 14

// --------------------
// --- Button Stuff ---
// --------------------
uint8_t state = 0;
int buttonPushCounter = 0;
bool autoChangeVisuals = true;
Button modeBtn(BTN_PIN, DEBOUNCE_MS);


void incrementButtonPushCounter() {
  buttonPushCounter = ++buttonPushCounter % NUM_VISUALS;
  EEPROM.write(1, buttonPushCounter);
}

bool genesis_mode;

void setup() {
  delay(1000); // power-up safety delay

  pinMode(D1, OUTPUT);
  digitalWrite(D1, LOW); 

  pinMode(D3, INPUT);
  genesis_mode = digitalRead(D3);

  if (genesis_mode) {
    setup_genesis();
    return;
  } 
  
  FastLED.addLeds < LED_TYPE, LEFT_OUT_PIN, COLOR_ORDER > (ledsLeft, N_PIXELS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds < LED_TYPE, RIGHT_OUT_PIN, COLOR_ORDER > (ledsRight, N_PIXELS).setCorrection(TypicalLEDStrip);
  //FastLED.setBrightness(BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_MILLIAMPS);

  modeBtn.begin();
  Serial.begin(115200);

  buttonPushCounter = (int)EEPROM.read(1) % NUM_VISUALS; // load previous setting
  //buttonPushCounter = 0;
  Serial.print("Starting pattern ");
  Serial.println(buttonPushCounter);
}

void loop() {

  if (genesis_mode) {
     loop_genesis();
     return; 
  }
  
  // Read button
  modeBtn.read(); 
  switch (state) {
    case 0:                
      if (modeBtn.wasReleased()) {
        incrementButtonPushCounter();
        autoChangeVisuals = false;
        Serial.print("Short press, pattern ");
        Serial.println(buttonPushCounter);
      }
      else if (modeBtn.pressedFor(LONG_PRESS))
        state = 1;
      break;
    
    case 1:
      if (modeBtn.wasReleased()) {
        state = 0;
        Serial.print("Long press, auto, pattern ");
        Serial.println(buttonPushCounter);
        autoChangeVisuals = true;
      }
      break;
  }
  
  // Switch pattern if on auto
  if(autoChangeVisuals){
    EVERY_N_SECONDS(PATTERN_TIME) { 
      incrementButtonPushCounter();
      Serial.print("Auto, pattern ");
      Serial.println(buttonPushCounter); 
    }
  }

  // Run selected pattern
  switch (buttonPushCounter) {
//    case 0:
//      vu4(1, 0);
//      if (STEREO) vu4(1, 1);
//      else copyLeftToRight();
//      break;
  
    case 0:
      vu4(2, 0);
      if (STEREO) vu4(2, 1);
      else copyLeftToRight();
      break;
  
//    case 2:
//      vu5(1, 0);
//      if (STEREO) vu5(1, 1);
//      else copyLeftToRight();
//      break;
  
    case 1:
      vu5(2, 0);
      if (STEREO) vu5(2, 1);
      else copyLeftToRight();
      break;
  
    case 2:
      vu6(false, 0);
      if (STEREO) vu6(false, 1);
      else copyLeftToRight();
      break;
      
    case 3:
      vu7(true);
      copyLeftToRight();
      break;
  
    case 4:
      vu8();
      copyLeftToRight();
      break;
  
    case 5:
      vu9();
      break;
  
    case 6:
      vu10();
      break;
    
 //   case 9:
 //     vu7(false);
 //     copyLeftToRight();
 //     break;
  
    case 7:
      twinkle();
      break;
  
    case 8:
      sinelon();
      break;
  
    case 9:
      balls();
      break;
  
    case 10:
      juggle();
      break;
  
    case 11:
      fire();
      break;
  
    case 12:
      ripple(false);
      break;
  
    case 13:
      rainbow(10);
      break;
  }
}

// ------------------
// -- VU functions --
// ------------------

int readLeft() {
    int n = analogRead(LEFT_IN_PIN); // Raw reading from left line in

    // low pass filter to find DC value
    static int dc_offset = 512;
    static int acc = 512 * 256;
    acc = acc - dc_offset + n;
    dc_offset = acc >> 8;
//Serial.println(n);
//Serial.print("\t");
//Serial.println((n - dc_offset) * MIC_GAIN);
    return (n - dc_offset) * MIC_GAIN; // Center on zero  
}

int readRight() {
    int n = analogRead(RIGHT_IN_PIN); // Raw reading from mic

    // low pass filter to find DC value
    static int dc_offset = 512;
    static int acc = 512 * 256;
    acc = acc - dc_offset + n;
    dc_offset = acc >> 8;

    return (n - dc_offset) * MIC_GAIN; // Center on zero  
}

uint16_t auxReading(uint8_t channel) {

  int n = 0;
  uint16_t height = 0;

  // on ESP8266, called at about 400Hz
  
  if(channel == 0) {
    n = abs(readLeft()); // Center on zero
    n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
    lvlLeft = ((lvlLeft * 7) + n) >> 3; // "Dampened" reading else looks twitchy (>>3 is divide by 8)
    volLeft[volCountLeft] = n; // Save sample for dynamic leveling
    volCountLeft = ++volCountLeft % SAMPLES;
    // Calculate bar height based on dynamic min/max levels (fixed point):
    height = TOP * (lvlLeft - minLvlAvgLeft) / (long)(maxLvlAvgLeft - minLvlAvgLeft);
  }
  
  else {
    n = abs(readRight()); // Center on zero
    n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
    lvlRight = ((lvlRight * 7) + n) >> 3; // "Dampened" reading (else looks twitchy)
    volRight[volCountRight] = n; // Save sample for dynamic leveling
    volCountRight = ++volCountRight % SAMPLES;
    // Calculate bar height based on dynamic min/max levels (fixed point):
    height = TOP * (lvlRight - minLvlAvgRight) / (long)(maxLvlAvgRight - minLvlAvgRight);
  }

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = constrain(height, 0, TOP);
  Serial.println(height);
  return height;
}

void copyLeftToRight() {
  for (uint8_t i = 0; i < N_PIXELS; i++) {
    ledsRight[i] = ledsLeft[i];
  }
}

/*
 * Function for dropping the peak
 */
uint8_t peakLeft, peakRight;
void dropPeak(uint8_t channel) {
  
  static uint8_t dotCountLeft, dotCountRight;
 
  if(channel == 0) {
    if(++dotCountLeft >= PEAK_FALL) { //fall rate 
      if(peakLeft > 0) peakLeft--;
      dotCountLeft = 0;
    }
  } else {
    if(++dotCountRight >= PEAK_FALL) { //fall rate 
      if(peakRight > 0) peakRight--;
      dotCountRight = 0;
    }
  }
}

/*
 * Function for averaging the sample readings
 */
void averageReadings(uint8_t channel) {

  uint16_t minLvl, maxLvl;

  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if(channel == 0) {
    minLvl = maxLvl = volLeft[0];
    for (int i = 1; i < SAMPLES; i++) {
      if (volLeft[i] < minLvl) minLvl = volLeft[i];
      else if (volLeft[i] > maxLvl) maxLvl = volLeft[i];
    }
    if ((maxLvl - minLvl) < TOP / 4) maxLvl = minLvl + TOP / 4;
    
    minLvlAvgLeft = (minLvlAvgLeft * 63 + minLvl) >> 6; // Dampen min/max levels
    maxLvlAvgLeft = (maxLvlAvgLeft * 63 + maxLvl) >> 6; // (fake rolling average)
  }

  else {
    minLvl = maxLvl = volRight[0];
    for (int i = 1; i < SAMPLES; i++) {
      if (volRight[i] < minLvl) minLvl = volRight[i];
      else if (volRight[i] > maxLvl) maxLvl = volRight[i];
    }
    if ((maxLvl - minLvl) < TOP / 4) maxLvl = minLvl + TOP / 4;
    minLvlAvgRight = (minLvlAvgRight * 63 + minLvl) >> 6; // Dampen min/max levels
    maxLvlAvgRight = (maxLvlAvgRight * 63 + maxLvl) >> 6; // (fake rolling average)
  }
}
