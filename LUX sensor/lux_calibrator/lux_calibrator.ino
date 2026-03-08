#include <Arduino.h>

// One-shot calibration sketch.
// Place LDR under a known LUX source (default 500 LUX),
// then compute the intercept 'b' for the log-log LDR model:
//   log10(R) = m * log10(Lux) + b

const int ADC_PIN  = A0;
const int ADC_MAX  = 4095;
const float V_REF  = 3.3f;
const float R_FIXED = 10000.0f;
const float LDR_M  = -0.8f;
const float KNOWN_LUX = 500.0f;     // Reference LUX during calibration
const int NUM_SAMPLES = 50;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  analogReadResolution(12);

  // Take multiple samples to filter noise
  long adc_sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    adc_sum += analogRead(ADC_PIN);
    delay(10);
  }

  // Convert average ADC value to LDR resistance
  float avg_adc = adc_sum / (float)NUM_SAMPLES;
  if (avg_adc <= 0 || avg_adc >= ADC_MAX) {
    Serial.println("Error: ADC saturated or disconnected");
    return;
  }
  float voltage = (avg_adc / (float)ADC_MAX) * V_REF;
  float ldr_resistance = R_FIXED * (V_REF / voltage - 1.0f);

  // Calculate intercept b using known LUX reference
  float b = log10(ldr_resistance) - (LDR_M * log10(KNOWN_LUX));

  Serial.println("Calibration finished");
  Serial.print("Calibrated LDR_B: ");
  Serial.println(b, 4);
  Serial.println("\nCopy this value into your sketches:");
  Serial.print("  const float LDR_B = "); Serial.print(b, 4); Serial.println(";");
}

void loop() {
  // Empty because calibration only needs to run once
}