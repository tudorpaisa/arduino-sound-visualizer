// A basic everyday NeoPixel strip test program.

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include <CircularBuffer.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    6

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 60

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


const int sampleWindow = 30; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
const double THRESHOLD = 0.20;
long globalFirstPixelHue = 0;
const long globalMaxFirstPixelHue = 65535;//5*65536;
const int globalPixelIncrement = 8;
int globalBrightness = 255;
float globalFloatBrightness = 1.0f;
const double globalThresholdBuffer = 0.02;
const double globalNoiseLevel = 0.05;
const int MAX_BRIGHTNESS = 50;
const int BIGHTNESS_DECREASE = 20;
const int BUFFER_SIZE = 128;
CircularBuffer<double, BUFFER_SIZE> voltageBuffer;



// setup() function -- runs once at startup --------------------------------

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  // init voltageBuffer to 0;
  for (int i=0; i<100; i++) {
    voltageBuffer.push(0.0);
  }

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(MAX_BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
  Serial.begin(9600);
}


// loop() function -- runs repeatedly as long as board is on ---------------

void loop() {

  // get voltage
  double voltage = getVoltage();
  // Serial.println(voltage);
  updateVoltageBuffer(voltage - globalNoiseLevel);
  double averageVoltage = getAverageVoltage();
  // color
  if (voltage - globalNoiseLevel >= averageVoltage + globalThresholdBuffer) {
    globalBrightness = 255;
    // globalFloatBrightness = 1.0f;
    // rainbowFill(0);
    intColorBrightnessFill();

  }
  else {
    decreaseGlobalBrightness();
    // decreaseGlobalFloatBrightness();
    // rainbowBrightnessFill(0);
    intColorBrightnessFill();
  }
  // Serial.println(globalFirstPixelHue);

  // Fill along the length of the strip in various colors...
  // colorWipe(strip.Color(255,   0,   0), 50); // Red
  // colorWipe(strip.Color(  0, 255,   0), 50); // Green
  // colorWipe(strip.Color(  0,   0, 255), 50); // Blue

  // Do a theater marquee effect in various colors...
  // theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
  // theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
  // theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness

  // rainbow(10);             // Flowing rainbow cycle along the whole strip
  // theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
}

// get voltage
double getVoltage()
{
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(0);
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
   double volts = (peakToPeak * 5.0) / 1024;  // convert to volts

   return volts;
}

double getAverageVoltage()
{
  double sum = 0.0;
  for (int i=0; i<100; i++) {
    sum += voltageBuffer[i];
  }
  return sum / BUFFER_SIZE;
}

void updateVoltageBuffer(double volts)
{
  voltageBuffer.push(volts);
}

void rainbowFill(int wait) {
  if (globalFirstPixelHue >= globalMaxFirstPixelHue) {
    globalFirstPixelHue = 0;
  }
  strip.rainbow(globalFirstPixelHue);
  // Above line is equivalent to:
  // strip.rainbow(firstPixelHue, 1, 255, 255, true);
  strip.show(); // Update strip with new contents
  globalFirstPixelHue += 256;
  delay(wait);  // Pause for a moment
}

void rainbowBrightnessFill(int wait) {
  if (globalFirstPixelHue >= globalMaxFirstPixelHue) {
    globalFirstPixelHue = 0;
  }
  strip.rainbow(globalFirstPixelHue, 1, 255, globalBrightness, true);
  // Above line is equivalent to:
  // strip.rainbow(firstPixelHue, 1, 255, 255, true);
  strip.show(); // Update strip with new contents
  globalFirstPixelHue += 256;
  delay(wait);  // Pause for a moment
}


void intColorFill() {
  if (globalFirstPixelHue >= globalMaxFirstPixelHue) {
    globalFirstPixelHue = 0;
  }
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, globalFirstPixelHue + i);         //  Set pixel's color (in RAM)
  }
  // strip.setBrightness(MAX_BRIGHTNESS);
  strip.show();
  globalFirstPixelHue += globalPixelIncrement;
}

void intColorBrightnessFill() {
  if (globalFirstPixelHue >= globalMaxFirstPixelHue) {
    globalFirstPixelHue = 0;
  }
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, strip.ColorHSV(globalFirstPixelHue+(i*globalPixelIncrement), 255, globalBrightness));         //  Set pixel's color (in RAM)
  }
  // strip.setBrightness((int)((float)MAX_BRIGHTNESS * globalFloatBrightness));
  strip.show();
  globalFirstPixelHue += globalPixelIncrement;
}

void decreaseGlobalBrightness() {
  int newBrightness = globalBrightness - BIGHTNESS_DECREASE;
  if (newBrightness > 0) {
    globalBrightness = newBrightness;
  }
  else {
    globalBrightness = 0;
  }
}

void decreaseGlobalFloatBrightness() {
  int newBrightness = globalFloatBrightness - 0.005f;
  if (newBrightness > 0.0f) {
    globalFloatBrightness = newBrightness;
  }
  else {
    globalFloatBrightness = 0.0f;
  }
}


// Some functions of our own for creating animated effects -----------------

void colorFill(uint32_t color) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
  strip.show();
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
