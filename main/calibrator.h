#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "led_driver.h"
#include "lux_sensor.h"

// ===================== Calibrator =====================
// Reusable calibration routines for the luminaire system
// Operates on injected LedDriver and LuxSensor references
//
// Provides:
//   - Background illuminance measurement (LED off)
//   - Static gain measurement via duty-cycle sweep + linear regression
//   - General-purpose linear regression utility

class Calibrator {
private:
  LedDriver  &led;
  LuxSensor  &sensor;
  int   settle_ms;
  int   gain_steps;

  static const int MAX_STEPS = 21;

public:
  Calibrator(LedDriver &led, LuxSensor &sensor,
             int settle_ms = 500, int gain_steps = 11)
    : led(led), sensor(sensor),
      settle_ms(settle_ms), gain_steps(gain_steps) {}

  // Measure ambient illuminance with LED off
  float measureBackground() {
    led.off();
    delay(settle_ms);
    float bg = sensor.readRaw();
    sensor.resetFilter();
    return bg;
  }

  // Sweep LED 0→100%, perform linear regression, return static gain (slope)
  // Also fills intercept_out if provided
  float measureStaticGain(float *intercept_out = nullptr) {
    int count = constrain(gain_steps, 2, MAX_STEPS);
    float duty[MAX_STEPS], lux[MAX_STEPS];

    for (int i = 0; i < count; i++) {
      float d = (float)i / (count - 1);
      duty[i] = d;
      led.setDutyCycle(d);
      delay(settle_ms);
      lux[i] = sensor.readRaw();
    }
    led.off();

    float slope, intercept, r2;
    ridgeRegression(duty, lux, count, slope, intercept, r2);

    if (intercept_out) *intercept_out = intercept;
    return slope;
  }

  // Ridge regression (closed-form): y = slope*x + intercept
  // Regularisation parameter lambda penalises large slope values,
  // improving numerical stability on noisy / collinear data.
  // The intercept is not regularised
  //
  // Closed-form for the design matrix X = [1, x]:
  //   A = X^T X + diag(0, lambda)
  //   beta = A^{-1} X^T y
  //
  // Expanded for 2×2:
  //   det = n*(Σx² + λ) - (Σx)²
  //   slope     = (n·Σxy - Σx·Σy) / det
  //   intercept = (Σy·(Σx² + λ) - Σx·Σxy) / det
  //
  // When lambda = 0 this reduces to ordinary least squares.
  static void ridgeRegression(const float *x, const float *y, int n,
                               float &slope, float &intercept, float &r_squared,
                               float lambda = 0.01f) {
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (int i = 0; i < n; i++) {
      sum_x  += x[i];
      sum_y  += y[i];
      sum_xy += x[i] * y[i];
      sum_x2 += x[i] * x[i];
    }

    // Closed-form inverse of the 2×2 ridge-augmented Gram matrix
    float denom = n * (sum_x2 + lambda) - sum_x * sum_x;
    if (fabsf(denom) < 1e-12f) {
      slope = 0; intercept = 0; r_squared = 0;
      return;
    }
    slope     = (n * sum_xy - sum_x * sum_y) / denom;
    intercept = (sum_y * (sum_x2 + lambda) - sum_x * sum_xy) / denom;

    // R² on the original (unpenalised) residuals
    float ss_res = 0, ss_tot = 0;
    float mean_y = sum_y / n;
    for (int i = 0; i < n; i++) {
      float predicted = slope * x[i] + intercept;
      ss_res += (y[i] - predicted) * (y[i] - predicted);
      ss_tot += (y[i] - mean_y) * (y[i] - mean_y);
    }
    r_squared = (ss_tot > 0.0f) ? 1.0f - ss_res / ss_tot : 0.0f;
  }

};

#endif
