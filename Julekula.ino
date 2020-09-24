#include "Adafruit_BMP085_soft.h"
#include <Adafruit_BMP085.h>


#define LED_PIN 2

#define SDA_0 4
#define SCL_0 5
#define SDA_1 13
#define SCL_1 12
// sda 11 scl 12

Adafruit_BMP085 bmp0;
Adafruit_BMP085_soft bmp1;
int32_t p_zero0, p_zero1;

  
void setup() {
  // Init serial
  Serial.begin(115200);

  // Set LED pin to output and turn on LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Init pressure sensor 0
  if (!bmp0.begin(SDA_0, SCL_0, BMP085_ULTRAHIGHRES))
	  Serial.println("Could not find a valid BMP085 sensor, check wiring for sensor 0!");
  else
    Serial.println("BMP085 sensor 0 found!");

  // Init pressure sensor 1
  if (!bmp1.begin(SDA_1, SCL_1, BMP085_ULTRAHIGHRES))
	  Serial.println("Could not find a valid BMP085 sensor, check wiring for sensor 1!");
  else
    Serial.println("BMP085 sensor 1 found!");

  // Calibrate sensors
  Serial.println("Calibrating pressure sensors!");
  float pressure0 = 0.0f;
  float pressure1 = 0.0f;
  int n_meas = 20;
  for(int i = 0;i < n_meas; i++){
    pressure0 += ((float)bmp0.readPressure())/((float)n_meas);
    pressure1 += ((float)bmp1.readPressure())/((float)n_meas);
    delay(100);
  }
  // Calculate calibration values
  p_zero0 = (int32_t)pressure0;
  p_zero1 = (int32_t)pressure1;
  Serial.print("Sensor 0 calibration value: ");
  Serial.println(p_zero0);
  Serial.print("Sensor 1 calibration value: ");
  Serial.println(p_zero1);
  Serial.println("Calibration done!");
  
  // Turn off LED
  digitalWrite(LED_PIN, HIGH);



}
  
void loop() {
    // TODO: do continuous calibration with a low pass filter with very long time constant

    // Serial.print("Temperature = ");
    // Serial.print(bmp0.readTemperature());
    // Serial.println(" *C");
    int32_t pressure0 = bmp0.readPressure();
    int32_t pressure1 = bmp1.readPressure();
    
    Serial.print("Pressure 0 = ");
    Serial.print(pressure0);
    Serial.println(" Pa");

    Serial.print("Pressure 1 = ");
    Serial.print(pressure1);
    Serial.println(" Pa");

    // Sensor 1 seems to give a diff on both readings while sensor 0 doesn't do anything..?
    // TODO: Maybe test sensor 0 by itself?
    // No, wire doesnt support two i2c buses: https://dronebotworkshop.com/multiple-i2c-bus/
    // Or this one: https://playground.arduino.cc/Main/SoftwareI2CLibrary/
    Serial.print("Diff pressures: ");
    Serial.print(pressure0 - p_zero0);
    Serial.print(" , ");
    Serial.print(pressure1 - p_zero1);
    Serial.println();

    if(((pressure0 - p_zero0) > 13) || ((pressure1 - p_zero1) > 13))
      digitalWrite(LED_PIN, LOW);
    else
      digitalWrite(LED_PIN, HIGH);
    
    Serial.println();
    delay(500);
}