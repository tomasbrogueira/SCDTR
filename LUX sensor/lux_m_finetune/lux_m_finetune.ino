#include <Arduino.h>
#include "../../main/led_driver.h"
#include "../../main/lux_sensor.h"
#include "../../main/calibrator.h"

// Fine tuning under controlled behaviour of light inside the box.
// Sweeps LED duty cycle 0→100%, measures LUX at each step, then
// performs linear regression. Iteratively adjusts m to maximise
// R² (linearity of LUX vs duty cycle). Also reports background
// illuminance and static gain.

// Starting LDR calibration values — will be optimised by this sketch
float ldr_m = -0.8f;
float ldr_b = 6.5f;

// Sweep configuration
const int NUM_STEPS = 21;          // 0%, 5%, 10%, ..., 100%
const int SETTLE_MS = 500;         // RC filter settling time

// Arrays to hold sweep data
float duty_arr[NUM_STEPS];
float lux_arr[NUM_STEPS];

LedDriver  led;
LuxSensor  sensor;

// Runs a full duty-cycle sweep and fills duty_arr[] and lux_arr[]
void runSweep() {
  for (int i = 0; i < NUM_STEPS; i++) {
    float d = (float)i / (NUM_STEPS - 1);
    duty_arr[i] = d;
    led.setDutyCycle(d);
    delay(SETTLE_MS);
    lux_arr[i] = sensor.readRaw();
  }
  led.off();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  led.init(15);
  sensor.init(A0, ldr_m, ldr_b);
}

void loop() {
  // Run only once
  static bool done = false;
  if (done) return;
  done = true;

  Serial.println("=== LDR m/b Fine-Tuning ===");
  Serial.println("Starting sweep with initial m and b...\n");

  // --- Phase 1: sweep with current m ---
  runSweep();

  float slope, intercept, r2;
  Calibrator::ridgeRegression(duty_arr, lux_arr, NUM_STEPS, slope, intercept, r2);

  Serial.println("Duty\tLux");
  for (int i = 0; i < NUM_STEPS; i++) {
    Serial.print(duty_arr[i], 4);
    Serial.print('\t');
    Serial.println(lux_arr[i], 2);
  }

  Serial.print("\nInitial m = "); Serial.println(ldr_m, 4);
  Serial.print("Initial R² = "); Serial.println(r2, 6);

  // --- Phase 2: iterative optimisation of m ---
  float best_m = ldr_m;
  float best_r2 = r2;
  float delta = 0.05;

  for (int iter = 0; iter < 10; iter++) {
    // Try m - delta
    ldr_m = best_m - delta;
    sensor.setCalibration(ldr_m, ldr_b);
    runSweep();
    Calibrator::ridgeRegression(duty_arr, lux_arr, NUM_STEPS, slope, intercept, r2);
    if (r2 > best_r2) {
      best_r2 = r2;
      best_m = ldr_m;
      continue;
    }

    // Try m + delta
    ldr_m = best_m + delta;
    sensor.setCalibration(ldr_m, ldr_b);
    runSweep();
    Calibrator::ridgeRegression(duty_arr, lux_arr, NUM_STEPS, slope, intercept, r2);
    if (r2 > best_r2) {
      best_r2 = r2;
      best_m = ldr_m;
      continue;
    }

    // Neither direction improved — reduce step size
    delta *= 0.5;
    if (delta < 0.005) break;
  }

  ldr_m = best_m;
  sensor.setCalibration(ldr_m, ldr_b);

  // --- Final sweep with optimal m ---
  runSweep();
  Calibrator::ridgeRegression(duty_arr, lux_arr, NUM_STEPS, slope, intercept, r2);

  // --- Results ---
  Serial.println("\n=== Fine-Tuning Results ===");

  Serial.println("\nDuty\tLux");
  for (int i = 0; i < NUM_STEPS; i++) {
    Serial.print(duty_arr[i], 4);
    Serial.print('\t');
    Serial.println(lux_arr[i], 2);
  }

  Serial.print("\nOptimal m = ");       Serial.println(ldr_m, 4);
  Serial.print("Current b = ");         Serial.println(ldr_b, 4);
  Serial.print("R² = ");                Serial.println(r2, 6);

  float lux_background = lux_arr[0];
  Serial.print("Background LUX = ");    Serial.println(lux_background, 2);

  Serial.print("Static gain G = ");     Serial.print(slope, 2);
  Serial.println(" LUX/duty");

  Serial.println("\nCopy these values into your other sketches:");
  Serial.print("  const float LDR_M = ");  Serial.print(ldr_m, 4);  Serial.println(";");
  Serial.print("  const float LDR_B = ");  Serial.print(ldr_b, 4);  Serial.println(";");
}