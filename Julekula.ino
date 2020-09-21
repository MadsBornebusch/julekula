#include <Wire.h>
#include <Adafruit_BMP085.h>

/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor
  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391
  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here
#define LED_PIN 2

#define SDA_0 4
#define SCL_0 5
#define SDA_1 13
#define SCL_1 12
// sda 11 scl 12

Adafruit_BMP085 bmp0;
Adafruit_BMP085 bmp1;
int32_t p_zero0, p_zero1;

  
void setup() {
  // Init serial
  Serial.begin(115200);

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
  
  // Set LED pin to output
  pinMode(LED_PIN, OUTPUT);


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
    Serial.print("Diff pressures: ");
    Serial.print(pressure0 - p_zero0);
    Serial.print(" , ");
    Serial.print(pressure1 - p_zero1);
    Serial.println();

    if(((pressure0 - p_zero0) > 13) || ((pressure1 - p_zero1) > 13))
      digitalWrite(LED_PIN, LOW);
    else
      digitalWrite(LED_PIN, HIGH);

    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    // Serial.print("Altitude = ");
    // Serial.print(bmp0.readAltitude());
    // Serial.println(" meters");

    // Serial.print("Pressure at sealevel (calculated) = ");
    // Serial.print(bmp0.readSealevelPressure());
    // Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    // Serial.print("Real altitude = ");
    // Serial.print(bmp0.readAltitude(101500));
    // Serial.println(" meters");
    
    Serial.println();
    delay(500);
}