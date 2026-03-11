#include <Arduino.h>
#include "../../main/led_driver.h"
#include "../../main/lux_sensor.h"
#include "../../main/calibrator.h"

// LDR calibration constants — keep in sync with main/main.ino
const float LDR_M = -0.8f;
const float LDR_B = 6.5f;

LedDriver  led;
LuxSensor  sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  led.init(15);
  sensor.init(A0, LDR_M, LDR_B);
}

void loop() {
  // Create calibrator using shared led & sensor objects
  Calibrator cal(led, sensor, 500, 11);

  float background = cal.measureBackground();
  Serial.print("Background LUX = ");
  Serial.println(background, 2);

  float gain = cal.measureStaticGain();
  Serial.print("Static gain G = ");
  Serial.print(gain, 2);
  Serial.println(" LUX/duty");

  // Wait before next sweep
  delay(5000);
}