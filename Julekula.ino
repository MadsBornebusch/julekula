#include "Adafruit_BMP085_soft.h"
//#include <Adafruit_BMP085.h>


#define LED_PIN 2
#define BUTTON_PIN 0

#define SDA_0 4
#define SCL_0 5
#define SDA_1 13
#define SCL_1 12
// sda 11 scl 12

#define HPF_FC 0.1
#define LPF_FC 0.01

#define HPF_RC 1.0/(6.28318530718 * HPF_FC)
#define LPF_RC 1.0/(6.28318530718 * LPF_FC)


Adafruit_BMP085_soft bmp0(SDA_0, SCL_0);
Adafruit_BMP085_soft bmp1(SDA_1, SCL_1);
int32_t p_zero0, p_zero1;
float var_zero0, var_zero1;


float highPass(float in, float in_last, float out_last, float dt, float RC){
  float alpha = RC / (RC + dt);
  return alpha * out_last + alpha * (in - in_last);
}

float lowPass(float in, float out_last, float dt, float RC){
  float alpha = dt / (RC + dt);
  return alpha * in + (1-alpha) * out_last;
}

void setup() {
  // Init serial
  Serial.begin(115200);

  // Set LED pin to output and turn on LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Set the button pin to input
  pinMode(BUTTON_PIN, INPUT);

  // Init pressure sensor 0
  if (!bmp0.begin(BMP085_ULTRAHIGHRES))
    Serial.println("Could not find a valid BMP085 sensor, check wiring for sensor 0!");
  else
    Serial.println("BMP085 sensor 0 found!");

  // Init pressure sensor 1
  if (!bmp1.begin(BMP085_ULTRAHIGHRES))
    Serial.println("Could not find a valid BMP085 sensor, check wiring for sensor 1!");
  else
    Serial.println("BMP085 sensor 1 found!");

  // Calibrate sensors
  Serial.println("Calibrating pressure sensors!");
  float pressure0 = 0.0f;
  float pressure1 = 0.0f;
  float max0 = 0.0f;
  float max1 = 0.0f;

  int n_meas = 20;
  for(int i = 0;i < n_meas; i++){
    int32_t p_0 = (float)bmp0.readPressure();
    int32_t p_1 = (float)bmp1.readPressure();
    pressure0 += p_0/((float)n_meas);
    pressure1 += p_1/((float)n_meas);
    max0 = (p_0 > max0) ? p_0 : max0;
    max1 = (p_1 > max1) ? p_1 : max1;

    delay(100);
  }
  var_zero0 = max0 - pressure0; 
  var_zero1 = max1 - pressure1;
  // Calculate calibration values
  p_zero0 = (int32_t)pressure0;
  p_zero1 = (int32_t)pressure1;
  // Serial.print("Sensor 0 calibration value: ");
  // Serial.println(p_zero0);
  // Serial.print("Sensor 1 calibration value: ");
  // Serial.println(p_zero1);
  Serial.println("Calibration done!");

  // Turn off LED
  digitalWrite(LED_PIN, HIGH);


  Serial.println("P0_diff,P0_hpf,P1_diff,P1_hpf,button");

}

void loop() {
    float dt = 0.1f;
    static float p0_last = (float)bmp0.readPressure();
    static float p1_last = (float)bmp1.readPressure();
    static float hpf0_last = 0.0f;
    static float hpf1_last = 0.0f;
    static float lpf0_last = 2.0 * var_zero0;
    static float lpf1_last = 2.0 * var_zero1;

    // Serial.print("Temperature = ");
    // Serial.print(bmp0.readTemperature());
    // Serial.println(" *C");
    int32_t pressure0 = bmp0.readPressure();
    int32_t pressure1 = bmp1.readPressure();

    float out_0 = highPass((float)pressure0, p0_last, hpf0_last, dt, HPF_RC);
    float out_1 = highPass((float)pressure1, p1_last, hpf1_last, dt, HPF_RC);
    float var_0 = lowPass(out_0 * out_0, lpf0_last, dt, LPF_RC);
    float var_1 = lowPass(out_1 * out_1, lpf1_last, dt, LPF_RC);

    // Last variables
    p0_last = (float)pressure0;
    p1_last = (float)pressure1;
    hpf0_last = out_0;
    hpf1_last = out_1;
    // Don't use extreme values in filter update
    if(3.0f * sqrt(var_0) > out_0)
      lpf0_last = var_0;
    if(3.0f * sqrt(var_1) > out_1)
      lpf1_last = var_1;

    // Serial.print(pressure0); 
    // Serial.print(",");
    // Serial.print(pressure0 - p_zero0); 
    // Serial.print(",");
    Serial.print(out_0);
    Serial.print(",");
    Serial.print(3.0f * sqrt(var_0));
    Serial.print(",");

    // Serial.print(pressure1);
    // Serial.print(",");
    // Serial.print(pressure1 - p_zero1);
    // Serial.print(",");
    Serial.print(out_1);
    Serial.print(",");
    Serial.print(3.0f * sqrt(var_1));

    Serial.print(",");
    Serial.print(!digitalRead(BUTTON_PIN));
    Serial.println();
    
    if(((3.0f * sqrt(var_0)) < out_0) || ((3.0f * sqrt(var_1)) < out_1))
      digitalWrite(LED_PIN, LOW);
    else
      digitalWrite(LED_PIN, HIGH);

    delay((long)(dt*1000.0));
}
