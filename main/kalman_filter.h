#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// 2-state Kalman filter for the linear illuminance model:
//   lux = G * duty + background
// State vector x = [G, background]^T
// Observation:  y = [duty, 1] * x + noise
// Dynamics:     x_{k+1} = x_k + process noise  (random walk)

class KalmanFilter {
private:
  // State estimates
  float G;        // gain (lux per unit duty)
  float bg;       // background illuminance (lux)

  // Error covariance P (2x2 symmetric: P00, P01, P11)
  float P00, P01, P11;

  // Process noise variances
  float q_G;      // gain drift rate
  float q_bg;     // background drift rate

  // Measurement noise variance
  float R;

  // Initial covariance (for reset)
  static constexpr float P_INIT = 100.0f;

  // State clamping limits
  float G_min, G_max;
  float bg_min, bg_max;

public:
  KalmanFilter()
    : G(0), bg(0),
      P00(P_INIT), P01(0.0f), P11(P_INIT),
      q_G(1e-4f), q_bg(1e-2f), R(25.0f),
      G_min(0.0f), G_max(1e6f),
      bg_min(0.0f), bg_max(1e6f) {}

  void init(float G0, float bg0) {
    G = G0;
    bg = bg0;
    P00 = P_INIT;
    P01 = 0.0f;
    P11 = P_INIT;
  }

  // Run one predict+update step
  // duty: current LED duty cycle (known input)
  // lux:  measured illuminance (observation)
  void update(float duty, float lux) {
    // Predict: P_pred = P + Q (random walk, F = I)
    float Pp00 = P00 + q_G;
    float Pp01 = P01;
    float Pp11 = P11 + q_bg;

    // H = [duty, 1]
    // Innovation: y_tilde = lux - (G * duty + bg)
    float y_pred = G * duty + bg;
    float innovation = lux - y_pred;

    // S = H * P_pred * H^T + R
    float S = Pp00 * duty * duty + 2.0f * Pp01 * duty + Pp11 + R;

    // Guard against near-zero S (shouldn't happen with R > 0)
    if (S < 1e-10f) return;

    // Kalman gain: K = P_pred * H^T / S
    float K0 = (Pp00 * duty + Pp01) / S;
    float K1 = (Pp01 * duty + Pp11) / S;

    // State update
    G  += K0 * innovation;
    bg += K1 * innovation;

    // Clamp states to physical limits
    G  = constrain(G,  G_min,  G_max);
    bg = constrain(bg, bg_min, bg_max);

    // Joseph-form covariance update: P = (I-KH)*Pp*(I-KH)' + K*R*K'
    // More numerically stable than the simple form P = (I-KH)*Pp
    float I_KH00 = 1.0f - K0 * duty;
    float I_KH01 = -K0;
    float I_KH10 = -K1 * duty;
    float I_KH11 = 1.0f - K1;

    // Temp = (I-KH) * Pp
    float T00 = I_KH00 * Pp00 + I_KH01 * Pp01;
    float T01 = I_KH00 * Pp01 + I_KH01 * Pp11;
    float T10 = I_KH10 * Pp00 + I_KH11 * Pp01;
    float T11 = I_KH10 * Pp01 + I_KH11 * Pp11;

    // P = Temp * (I-KH)' + K*R*K'
    P00 = T00 * I_KH00 + T01 * I_KH10 + K0 * R * K0;
    P01 = T00 * I_KH01 + T01 * I_KH11 + K0 * R * K1;
    P11 = T10 * I_KH01 + T11 * I_KH11 + K1 * R * K1;
  }

  float getGain() const { return G; }
  float getBackground() const { return bg; }
  float getGainVariance() const { return P00; }
  float getBackgroundVariance() const { return P11; }
};

#endif
