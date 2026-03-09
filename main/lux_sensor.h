#ifndef LUX_SENSOR_H
#define LUX_SENSOR_H

#include <Arduino.h>

// ===================== LUX Sensor =====================
// Encapsulates the LDR-based illuminance measurement.
// Model: log10(R) = m * log10(Lux) + b
//     Lux = K * ((ADC_MAX - adc) / adc)^p
//   where K = 10^(-b/m) * R_FIXED^(1/m), p = 1/m
//
// Includes median filter for noise reduction.

class LuxSensor {
private:
  int adc_pin;
  int adc_max;
  float R_fixed;

  float m;       // LDR log-log slope (negative, e.g. -0.8)
  float b;       // LDR log-log intercept
  float p_exp;   // 1/m  (pre-computed)
  float K;       // 10^(-b/m) * R_fixed^(1/m)  (pre-computed)

  int num_samples_log2;
  int num_samples;

  // Median filter state
  static const int MAX_MEDIAN_WINDOW = 15;
  int median_window;
  float median_buffer[MAX_MEDIAN_WINDOW];
  int median_idx;
  int median_count;

  void recomputeConstants() {
    p_exp = 1.0f / m;
    K = powf(10.0f, -b / m) * powf(R_fixed, 1.0f / m);
  }

  // Quickselect algorithm for finding the k-th smallest element
  float quickselect(float arr[], int left, int right, int k) {
    while (left < right) {
      int pivotIndex = left + (right - left) / 2;
      float pivotValue = arr[pivotIndex];
      
      // Swap pivot to the end
      arr[pivotIndex] = arr[right];
      arr[right] = pivotValue;
      
      int storeIndex = left;
      for (int i = left; i < right; i++) {
        if (arr[i] < pivotValue) {
          float temp = arr[storeIndex];
          arr[storeIndex] = arr[i];
          arr[i] = temp;
          storeIndex++;
        }
      }
      
      // Move pivot to its final place
      float temp = arr[storeIndex];
      arr[storeIndex] = arr[right];
      arr[right] = temp;
      
      if (k == storeIndex) {
        return arr[k];
      } else if (k < storeIndex) {
        right = storeIndex - 1;
      } else {
        left = storeIndex + 1;
      }
    }
    return arr[k];
  }

public:
  // Full constructor with all parameters
  LuxSensor(int adc_pin, float m, float b,
            float R_fixed = 10000.0f, int adc_resolution = 12,
            int num_samples_log2 = 4, int median_window_size = 5)
    : adc_pin(adc_pin), adc_max((1 << adc_resolution) - 1),
      R_fixed(R_fixed), m(m), b(b),
      num_samples_log2(num_samples_log2),
      num_samples(1 << num_samples_log2),
      median_window(median_window_size), median_idx(0), median_count(0)
  {
    if (median_window > MAX_MEDIAN_WINDOW) median_window = MAX_MEDIAN_WINDOW;
    analogReadResolution(adc_resolution);
    recomputeConstants();
  }

  // Default constructor (must call init() later)
  LuxSensor()
    : adc_pin(A0), adc_max(4095), R_fixed(10000.0f),
      m(-0.8f), b(6.5f), p_exp(0), K(0),
      num_samples_log2(4), num_samples(16),
      median_window(5), median_idx(0), median_count(0)
  {
  }

  // Deferred initialisation
  void init(int _adc_pin, float _m, float _b,
            float _R_fixed = 10000.0f, int adc_resolution = 12,
            int _num_samples_log2 = 4, int _median_window_size = 5) {
    adc_pin = _adc_pin;
    adc_max = (1 << adc_resolution) - 1;
    R_fixed = _R_fixed;
    m = _m;
    b = _b;
    num_samples_log2 = _num_samples_log2;
    num_samples = 1 << _num_samples_log2;
    
    median_window = _median_window_size;
    if (median_window > MAX_MEDIAN_WINDOW) median_window = MAX_MEDIAN_WINDOW;
    median_idx = 0;
    median_count = 0;

    analogReadResolution(adc_resolution);
    recomputeConstants();
  }

  // Raw LUX reading (hardware-oversampled, no software filter)
  float readRaw() {
    int adc_sum = 0;
    for (int i = 0; i < num_samples; i++) {
      adc_sum += analogRead(adc_pin);
    }
    float avg_adc = (float)(adc_sum >> num_samples_log2);
    if (avg_adc <= 0 || avg_adc >= adc_max) return 0.0f;
    return K * powf((adc_max - avg_adc) / avg_adc, p_exp);
  }

  // Reset median filter (call after calibration or large disturbance)
  void resetFilter() {
    median_count = 0;
    median_idx = 0;
  }

  // Median-filtered LUX reading using quickselect (reads ADC internally)
  float readMedianFiltered() {
    // Insert into circular buffer
    float raw = readRaw();
    median_buffer[median_idx] = raw;
    median_idx = (median_idx + 1) % median_window;
    if (median_count < median_window) {
      median_count++;
    }

    // Find median via quickselect on a copy
    float temp_arr[MAX_MEDIAN_WINDOW];
    for (int i = 0; i < median_count; i++) {
        temp_arr[i] = median_buffer[i];
    }
    int k = median_count / 2;
    return quickselect(temp_arr, 0, median_count - 1, k);
  }

  // Read raw averaged ADC value (no LUX conversion)
  float readRawAdc() {
    int adc_sum = 0;
    for (int i = 0; i < num_samples; i++) {
      adc_sum += analogRead(adc_pin);
    }
    return (float)(adc_sum >> num_samples_log2);
  }

  // Read voltage directly
  float readVoltage() {
    float avg_adc = readRawAdc();
    return avg_adc * 3.3f / adc_max; // Assuming 3.3V reference
  }

  // Update calibration constants (e.g. after m fine-tuning)
  void setCalibration(float _m, float _b) {
    m = _m;
    b = _b;
    recomputeConstants();
  }
};

#endif
