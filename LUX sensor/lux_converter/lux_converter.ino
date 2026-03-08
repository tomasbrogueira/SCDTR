#include <Arduino.h>
#include "../../main/lux_sensor.h"

// LDR calibration constants — keep in sync with main/main.ino
const float LDR_M = -0.8f;
const float LDR_B = 6.5f;

LuxSensor sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  sensor.init(A0, LDR_M, LDR_B);

  // Header line for Arduino Serial Plotter
  Serial.println("Lux");
}

void loop() {
  float lux = sensor.readRaw();
  Serial.println(lux, 2);
  delay(1000);
}