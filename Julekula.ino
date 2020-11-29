#include "Adafruit_BMP085_soft.h"
#include <Adafruit_BMP085.h>
#include <FastLED.h>
#include "Fire2012.h"
#include "ADXL345.h"


#define LED_PIN 2
#define BUTTON_PIN 0
#define PIXEL_PIN 3
#define NUM_LEDS 7
#define COLOR_ORDER GRB
#define CHIPSET WS2812B
#define BRIGHTNESS_MAX 160
#define BRIGHTNESS_SPEED 2

// Wiring
#define SDA_0 4
#define SCL_0 5
#define SDA_1 13
#define SCL_1 12
// sda 11 scl 12
#define SDA_2 14
#define SCL_2 16
#define IMU_SDA 4
#define IMU_SCL 5

#define HPF_FC 0.1
#define LPF_FC 0.01

// Number of standard deviations for detection
#define DETECT_THRES 4.0f

#define HPF_RC 1.0/(6.28318530718 * HPF_FC)
#define LPF_RC 1.0/(6.28318530718 * LPF_FC)

// Time of inactivity before going to sleep (seconds)
#define SLEEP_TIME 120
#define MIN_TO_US (60UL * 1000000UL)

Adafruit_BMP085 bmp0; //Adafruit_BMP085_soft bmp0(SDA_0, SCL_0);
ADXL345 adxl;
Adafruit_BMP085_soft bmp1(SDA_1, SCL_1);
Adafruit_BMP085_soft bmp2(SDA_2, SCL_2);
int32_t p_zero0, p_zero1, p_zero2;
float var_zero0, var_zero1, var_zero2;
CRGB leds[NUM_LEDS];
CRGBPalette16 gPal;
int brightness = 0;


float highPass(float in, float in_last, float out_last, float dt, float RC){
  float alpha = RC / (RC + dt);
  return alpha * out_last + alpha * (in - in_last);
}

float lowPass(float in, float out_last, float dt, float RC){
  float alpha = dt / (RC + dt);
  return alpha * in + (1-alpha) * out_last;
}

void adxlSetup(int sda, int scl){
  adxl.powerOn(sda, scl);

  // New code
  adxl.set_bw(ADXL345_BW_6);
  adxl.setLowPower(1); 
  adxl.setActivityAc(0);
  adxl.setInterruptLevelBit(1);
  adxl.setRangeSetting(2);

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(17); //62.5mg per increment
  
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);

  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT, ADXL345_INT1_PIN );
}

void adxlIntAct(){
  //register interupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
}

void adxlIntOff(){
  //register interupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 0);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 0);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  0);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   0);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 0);
}

void setup() {
  // Init ADXL
  adxlSetup(SDA_0, SCL_0);
  adxlIntOff();
  if(adxl.triggered(adxl.getInterruptSource(), ADXL345_ACTIVITY))
    Serial.println("ADXL activity");
  //adxlIntAct();

  // Init serial
  Serial.begin(115200);

  // Set LED pin to output and turn on LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Set the button pin to input with pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Init pressure sensor 0
  if (!bmp0.begin(SDA_0, SCL_0, BMP085_ULTRAHIGHRES))
    Serial.println("Could not find a valid BMP085 sensor, check wiring for sensor 0!");
  else
    Serial.println("BMP085 sensor 0 found!");

  // Init pressure sensor 1
  if (!bmp1.begin(BMP085_ULTRAHIGHRES))
    Serial.println("Could not find a valid BMP085 sensor, check wiring for sensor 1!");
  else
    Serial.println("BMP085 sensor 1 found!");

  // Init pressure sensor 2
  if (!bmp2.begin(BMP085_ULTRAHIGHRES))
    Serial.println("Could not find a valid BMP085 sensor, check wiring for sensor 2!");
  else
    Serial.println("BMP085 sensor 2 found!");

  // Calibrate sensors
  Serial.println("Calibrating pressure sensors!");
  float pressure0 = 0.0f;
  float pressure1 = 0.0f;
  float pressure2 = 0.0f;
  float max0 = 0.0f;
  float max1 = 0.0f;
  float max2 = 0.0f;

  int n_meas = 10;
  for(int i = 0;i < n_meas; i++){
    int32_t p_0 = (float)bmp0.readPressure();
    int32_t p_1 = (float)bmp1.readPressure();
    int32_t p_2 = (float)bmp2.readPressure();
    pressure0 += p_0/((float)n_meas);
    pressure1 += p_1/((float)n_meas);
    pressure2 += p_2/((float)n_meas);
    max0 = (p_0 > max0) ? p_0 : max0;
    max1 = (p_1 > max1) ? p_1 : max1;
    max2 = (p_2 > max2) ? p_2 : max2;

    delay(50);
  }
  var_zero0 = max0 - pressure0; 
  var_zero1 = max1 - pressure1;
  var_zero2 = max2 - pressure2;
  // Calculate calibration values
  p_zero0 = (int32_t)pressure0;
  p_zero1 = (int32_t)pressure1;
  p_zero2 = (int32_t)pressure2;
  Serial.println("Calibration done!");

  // Turn off LED
  digitalWrite(LED_PIN, HIGH);

  // Initialize LED strip and color palette
  FastLED.addLeds<CHIPSET, PIXEL_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(brightness);
  gPal = CRGBPalette16( CRGB::Black, CRGB::Red, CRGB::Yellow, CRGB::White);

  // HEader for the arduino serial plotter
  Serial.println("P0_diff,P0_lim,P1_diff,P1_lim,P2_diff,P2_lim, button");

}

void loop() {
    float dt = 0.1f;
    static float p0_last = (float)bmp0.readPressure();
    static float p1_last = (float)bmp1.readPressure();
    static float p2_last = (float)bmp2.readPressure();
    static float hpf0_last = 0.0f;
    static float hpf1_last = 0.0f;
    static float hpf2_last = 0.0f;
    static float lpf0_last = 2.0 * var_zero0;
    static float lpf1_last = 2.0 * var_zero1;
    static float lpf2_last = 2.0 * var_zero2;
    static long last_detection = millis();

    // Serial.print("Temperature = ");
    // Serial.print(bmp0.readTemperature());
    // Serial.println(" *C");
    int32_t pressure0 = bmp0.readPressure();
    int32_t pressure1 = bmp1.readPressure();
    int32_t pressure2 = bmp2.readPressure();

    float out_0 = highPass((float)pressure0, p0_last, hpf0_last, dt, HPF_RC);
    float out_1 = highPass((float)pressure1, p1_last, hpf1_last, dt, HPF_RC);
    float out_2 = highPass((float)pressure2, p2_last, hpf2_last, dt, HPF_RC);
    float var_0 = lowPass(out_0 * out_0, lpf0_last, dt, LPF_RC);
    float var_1 = lowPass(out_1 * out_1, lpf1_last, dt, LPF_RC);
    float var_2 = lowPass(out_2 * out_2, lpf2_last, dt, LPF_RC);

    // Last variables
    p0_last = (float)pressure0;
    p1_last = (float)pressure1;
    p2_last = (float)pressure2;
    hpf0_last = out_0;
    hpf1_last = out_1;
    hpf2_last = out_2;
    // Don't use extreme values in filter update
    if(DETECT_THRES * sqrt(var_0) > out_0)
      lpf0_last = var_0;
    if(DETECT_THRES * sqrt(var_1) > out_1)
      lpf1_last = var_1;
    if(DETECT_THRES * sqrt(var_2) > out_2)
      lpf2_last = var_2;

    // Debug prints for plotting in the arduino serial plotter
    Serial.print(out_0);
    Serial.print(",");
    Serial.print(DETECT_THRES * sqrt(var_0));
    Serial.print(",");
    Serial.print(out_1);
    Serial.print(",");
    Serial.print(DETECT_THRES * sqrt(var_1));
    Serial.print(",");
    Serial.print(out_2);
    Serial.print(",");
    Serial.print(DETECT_THRES * sqrt(var_2));
    Serial.print(",");
    Serial.print(!digitalRead(BUTTON_PIN));
    Serial.println();

    bool detection = (((DETECT_THRES * sqrt(var_0)) < out_0) || ((DETECT_THRES * sqrt(var_1)) < out_1) || ((DETECT_THRES * sqrt(var_2)) < out_2));

    // Show the detection with the LED
    if(detection)
      digitalWrite(LED_PIN, LOW);
    else
      digitalWrite(LED_PIN, HIGH);

    // LED animation
    static bool bright_up = true;
    if(detection || (brightness > 0))
    {
      // Set time for last activity
      last_detection = millis();
      // LED stuff
      random16_add_entropy( random(1000000));
      FastLED.setBrightness( brightness );
      // Control dimming up and down
      if(brightness >= BRIGHTNESS_MAX)
        bright_up = false;
      if (brightness <= 0)
        bright_up = true;
      if(bright_up)
        brightness += BRIGHTNESS_SPEED;
      else
        brightness -= BRIGHTNESS_SPEED;
      // Run fire simulation
      FastLED.show(); // display this frame
      Fire2012WithPalette(gPal, leds); // run simulation frame, using palette colors
    }
    else if (brightness <= 0)
      FastLED.clear(true);

    if((millis() - last_detection) > (SLEEP_TIME * 1000)){
      // Go to sleep
      Serial.println("Going to sleep!");
      if(adxl.triggered(adxl.getInterruptSource(), ADXL345_ACTIVITY))
        Serial.println("ADXL activity");
      adxlIntAct();
      ESP.deepSleep(20 * MIN_TO_US, WAKE_RF_DISABLED);
    }else{
      // Keep going
      delay((long)(dt*1000.0));
    }
      



}

// TODO: Read temperature as well? More data is always better I guess..
