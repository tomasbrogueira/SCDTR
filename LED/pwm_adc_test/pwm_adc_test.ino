#include <Arduino.h>
#include "../../main/led_driver.h"

const int LED_PIN = 15;
const int ADC_PIN = A0;

LedDriver led;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  analogReadResolution(12);
  led.init(LED_PIN);

  // Header line for Arduino Serial Plotter (tab-separated labels)
  Serial.println("Duty\tADC");
}

// Parses serial commands: "pwm <0-4096>", "pct <0-100>", "duty <0.0-1.0>"
void parseSerialCommand() {
  if (!Serial.available()) return;

  static char buf[32];
  int len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';

  // Find separator
  char *space = strchr(buf, ' ');
  if (!space) return;
  *space = '\0';
  float val = atof(space + 1);

  if (strcmp(buf, "pwm") == 0) {
    led.setPwm((int)val);
  } else if (strcmp(buf, "pct") == 0) {
    led.setPercentage(val);
  } else if (strcmp(buf, "duty") == 0) {
    led.setDutyCycle(val);
  }
}

void loop() {
  parseSerialCommand();

  int adc_val = analogRead(ADC_PIN);
  Serial.print(led.getDuty(), 4);
  Serial.print('\t');
  Serial.println(adc_val);

  delay(200);
}