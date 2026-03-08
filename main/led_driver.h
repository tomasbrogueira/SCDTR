#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include <Arduino.h>

// ===================== LED Driver =====================
// Encapsulates PWM output to the LED, tracking current duty cycle.
// Configures the RP2040 PWM hardware on construction.

class LedDriver {
private:
  int pin;
  int pwm_range;
  float duty;  // current duty cycle [0.0, 1.0]

public:
  // Configure PWM hardware and set LED off.
  // Call this in setup() after Serial.begin().
  //   pin          – GPIO pin for LED
  //   resolution   – PWM bit depth (e.g. 12)
  //   freq         – PWM frequency in Hz (e.g. 30000)
  LedDriver(int pin, int resolution = 12, int freq = 30000)
    : pin(pin), pwm_range(1 << resolution), duty(0.0f) {
    analogWriteResolution(resolution);
    analogWriteFreq(freq);
    analogWriteRange(pwm_range);
    analogWrite(pin, 0);
  }

  // Default constructor (no-op, must call init() later)
  LedDriver() : pin(-1), pwm_range(4096), duty(0.0f) {}

  // Deferred initialisation (useful for global objects)
  void init(int _pin, int resolution = 12, int freq = 30000) {
    pin = _pin;
    pwm_range = 1 << resolution;
    duty = 0.0f;
    analogWriteResolution(resolution);
    analogWriteFreq(freq);
    analogWriteRange(pwm_range);
    analogWrite(pin, 0);
  }

  // Set LED with raw PWM value [0, pwm_range]
  void setPwm(int pwm_value) {
    pwm_value = constrain(pwm_value, 0, pwm_range);
    analogWrite(pin, pwm_value);
    duty = (float)pwm_value / pwm_range;
  }

  // Set LED with duty cycle [0.0, 1.0]
  void setDutyCycle(float duty_cycle) {
    setPwm((int)(constrain(duty_cycle, 0.0f, 1.0f) * pwm_range));
  }

  // Set LED with percentage [0, 100]
  void setPercentage(float percent) {
    setDutyCycle(percent / 100.0f);
  }

  // Turn LED off
  void off() { setPwm(0); }

  // Getters
  float getDuty() const { return duty; }
};

#endif
