#include "arduinoFFT.h"
#include <Adafruit_NeoPixel.h>
#include <CircularBuffer.h>
#include <math.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 9990.0; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

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

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];
unsigned int sampleCounter = 0;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

// My global stuff
struct RGBColors {
  float red;
  float green;
  float blue;
};

struct HSVColors {
  uint16_t h;
  uint8_t s;
  uint8_t v;
};

const int sampleWindow = 30; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
const double THRESHOLD = 0.20;
int globalBrightness = 255;
const double globalThresholdBuffer = 0.02;
const double globalNoiseLevel = 0.05;

uint16_t globalPixelHue = 0;
const uint16_t GLOBAL_PIXEL_INCREMENT = 48;

const int BRIGHTNESS_DECREASE = 20;
const int MAX_BRIGHTNESS_UPDATE_THRESHOLD = 8;
const int BUFFER_SIZE = 128;
double volts = 0.0;
double averageVoltage = 0.0;
CircularBuffer<double, BUFFER_SIZE> voltageBuffer;

const float LOW_FF = 16.35f;
const float HIGH_FF = 32.7f;
const float LOW_NM = 740.0f;
const float HIGH_NM = 380.0f;
const float C = 299792458000000000.0f;
struct RGBColors colors;
struct HSVColors pendingHsvColors;
struct HSVColors hsvColors = { 65536, 255, 255 };

// RGB consts
const float MAX_INTENSITY = 255;


int intensity[LED_COUNT] = { };

void setup()
{
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  // init voltageBuffer to 0;
  for (int i=0; i<BUFFER_SIZE; i++) {
    voltageBuffer.push(0.0);
  }

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(128); // Set BRIGHTNESS to about 1/5 (max = 255)

  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}

void loop()
{
  // getColor();
  getVoltage();
  updateVoltageBuffer();
  getAverageVoltage();
  updateLights();

  // colorFill(strip.ColorHSV((uint16_t)hsvColors.h, (uint8_t)hsvColors.s, (uint8_t)hsvColors.v));
}


void getColor() {
  /*SAMPLING*/
  // microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }

  // Serial.println("windowing");
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  // Serial.println("computing");
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  // Serial.println("complex");
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

  // Serial.println("peak");
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  // Serial.println(x);
  // Serial.println("normlization");
  float y = normalizeConvert(x);

  // Serial.println("rgb");
  colors = nmToRgb(y);
  // Serial.println("hsv");
  hsvColors = rgbToHsv(colors);
}

void updateLights()
{
  // Serial.println("getting color");
  // getColor();
  // Serial.println("got color");
  // Serial.println(globalBrightness);
  if (volts - globalNoiseLevel >= averageVoltage + globalThresholdBuffer)
  {
    if (globalBrightness <= MAX_BRIGHTNESS_UPDATE_THRESHOLD)
    {
      getColor();
    }
    globalBrightness = 255;
    globalPixelHue = hsvColors.h;
    // getColor();
  }
  else
  {
    decreaseGlobalBrightness();
  }
  colorFillIncreasing();
}

void decreaseGlobalBrightness() {
  int newBrightness = globalBrightness - BRIGHTNESS_DECREASE;
  if (newBrightness > 0) {
    globalBrightness = newBrightness;
  }
  else {
    globalBrightness = 0;
  }
}

float toNm(float hz) {
  return C / hz;
}

float scaledownHz(float hz) {
  if (hz <= 10.0f) return 0.0f;
  if ((hz >= LOW_FF) && (hz <= HIGH_FF)) return hz;
  return scaledownHz(hz / 2.0f);
}

float scaleupHz(float hz) {
  if (hz <= 10.0f) return 0.0f;
  if ((hz >= LOW_FF) && (hz <= HIGH_FF)) return hz;
  return scaleupHz(hz * 2.0f);
}

float scaleHz(float hz) {
  if (hz < LOW_FF) return scaleupHz(hz);
  return scaledownHz(hz);
}

float normalizeConvert(float hz) {
  float low_hz_nm = toNm(HIGH_FF);
  float high_hz_nm = toNm(LOW_FF);

  return (toNm(scaleHz(hz)) - low_hz_nm) / (high_hz_nm - low_hz_nm) * (HIGH_NM - LOW_NM) + LOW_NM;
}

RGBColors nmToRgb(float nm) {
  float gamma = 0.8f;
  float factor, red, green, blue;

  if ((nm >= 380.0f) && (nm < 440.0f)) {
    red = -(nm - 440.0f) / (440.0f - 380.0f);
    green = 0.0f;
    blue = 1.0f;
  }
  else if ((nm >= 440.0f) && (nm < 490.0f)) {
    red = 0.0f;
    green = (nm - 440.0f) / (490.0f - 440.0f);
    blue = 1.0f;
  }
  else if ((nm >= 490.0f) && (nm < 510.0f)) {
    red = 0.0f;
    green = 1.0f;
    blue = -(nm - 510.0f) / (510.0f - 490.0f);
  }
  else if ((nm >= 510.0f) && (nm < 580.0f)) {
    red = (nm - 510.0f) / (580.0f - 510.0f);
    green = 1.0f;
    blue = 0.0f;
  }
  else if((nm >= 580.0f) && (nm < 645.0f)) {
    red = 1.0f;
    green = -(nm - 645.0f) / (645.0f - 580.0f);
    blue = 0.0f;
  } 
  else if ((nm >= 645.0f) && (nm < 781.0f)) {
    red = 1.0f;
    green = 0.0f;
    blue = 0.0f;
  }
  else {
    red = 0.0f;
    green = 0.0f;
    blue = 0.0f;
  }

  if ((nm >= 380) && (nm < 420)) {
    factor = 0.3f + 0.7f * (nm - 380.0f) / (420.0f - 380.0f);
  }
  else if ((nm >= 420) && (nm < 701)) {
    factor = 1.0f;
  }
  else if ((nm >= 701) && (nm < 781)) {
    factor = 0.3f + 0.7f * (780.0f - nm) / (780.0f - 700.0f);
  }
  else {
    factor = 0.0f;
  }


  if (red != 0) {
    red = round(MAX_INTENSITY * pow(red * factor, gamma));
  }
  if (green != 0) {
    green = round(MAX_INTENSITY * pow(green * factor, gamma));
  }
  if (blue != 0) {
    blue = round(MAX_INTENSITY * pow(blue * factor, gamma));
  }

  if ((red == 0.0f) && (green = 0.0f) && (blue == 0.0f)) {

  }
  struct RGBColors colors = {(int)red, (int)green, (int)blue};

  return colors; 
}

HSVColors rgbToHsv(RGBColors rgb) {
  // Code stressfully taken from:
  // https://stackoverflow.com/a/6930407
  HSVColors hsv;
  unsigned char rgbMin, rgbMax;
  float h;

  rgbMin = rgb.red < rgb.green ? (rgb.red < rgb.blue ? rgb.red : rgb.blue) : (rgb.green < rgb.blue ? rgb.green : rgb.blue);
  rgbMax = rgb.red > rgb.green ? (rgb.red > rgb.blue ? rgb.red : rgb.blue) : (rgb.green > rgb.blue ? rgb.green : rgb.blue);
  
  hsv.v = rgbMax;
  if (hsv.v == 0)
  {
      hsv.h = 0;
      hsv.s = 0;
      return hsv;
  }

  hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
  if (hsv.s == 0)
  {
      hsv.h = 0;
      return hsv;
  }

  if (rgbMax == rgb.red)
      // h = 0 + 43 * (rgb.green - rgb.blue) / (rgbMax - rgbMin);
      h = (rgb.green - rgb.blue) / (rgbMax - rgbMin);
  else if (rgbMax == rgb.green)
      // h = 85 + 43 * (rgb.blue - rgb.red) / (rgbMax - rgbMin);
      h = 2.0 + (rgb.blue - rgb.red) / (rgbMax - rgbMin);
  else
      // h = 171 + 43 * (rgb.red - rgb.green) / (rgbMax - rgbMin);
      h = 4.0 + (rgb.red - rgb.green) / (rgbMax - rgbMin);
  h *= 60.0f;

  if (h < 0.0) h += 360.0f;

  // hsv.h = (uint16_t)((h/255.0f) * 65535.0f);  // fuck me... why does Neopixel have to use uint16!?
  hsv.h = (uint16_t)((h / 360.0f) * 65535.0f);  // fuck me... why does Neopixel have to use uint16!?
  return hsv;
}

void colorFill(uint32_t color) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }

  strip.show();
}

void colorFillIncreasing() {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, strip.ColorHSV((globalPixelHue + (i * GLOBAL_PIXEL_INCREMENT)), hsvColors.s, globalBrightness));         //  Set pixel's color (in RAM)
  }

  strip.show();
  globalPixelHue += GLOBAL_PIXEL_INCREMENT;
}

void incrementSampleCounter()
{
  ++sampleCounter;
  if (sampleCounter >= samples)
  {
    sampleCounter = 0;
  }
}

// get voltage
void getVoltage()
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
        // Serial.println(sampleCounter);
        // vReal[sampleCounter] = sample;
        // // vImag[sampleCounter] = 0;
        // incrementSampleCounter();

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
   volts = (peakToPeak * 5.0) / 1024;  // convert to volts
}

void getAverageVoltage()
{
  double sum = 0.0;
  for (int i=0; i<100; i++) {
    sum += voltageBuffer[i];
  }
  averageVoltage = sum / BUFFER_SIZE;
}

void updateVoltageBuffer()
{
  voltageBuffer.push(volts - globalNoiseLevel);
}
