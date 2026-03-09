#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <math.h>

class PIDController {
private:
  float Kp, Ki, Kd;
  float T;                // sampling period (seconds)
  float u_min, u_max;     // actuator limits
  float beta;             // set-point weighting for proportional term
  float alpha_d;          // derivative low-pass filter coefficient

  float integral;
  float prev_y;           // previous measurement (for derivative on measurement)
  float prev_ref;         // previous reference (for bumpless transfer)
  float prev_d_term;      // previous filtered derivative term
  float prev_u_unsat;     // previous unsaturated output (for anti-windup)
  float Kaw;              // cached anti-windup tracking gain

  bool feedback_on;
  bool feedforward_on;
  bool antiwindup_on;
  bool backcalculation_on;
  bool bumpless_on;

  float ff_gain;          // static gain G (LUX per unit duty)
  float ff_background;    // background LUX (feedforward offset)

  float saturate(float u) {
    if (u < u_min) return u_min;
    if (u > u_max) return u_max;
    return u;
  }

  void updateKaw() {
    if (Kd > 0.0f) Kaw = sqrtf(Ki / Kd);
    else if (Ki > 0.0f && Kp > 0.0f) Kaw = Ki / Kp;
    else Kaw = 1.0f;
  }

public:
  PIDController(float Kp, float Ki, float Kd, float T,
                float u_min = 0.0f, float u_max = 1.0f)
    : Kp(Kp), Ki(Ki), Kd(Kd), T(T),
      u_min(u_min), u_max(u_max),
      beta(0.75f), alpha_d(0.1f),
      integral(0.0f), prev_y(0.0f), prev_ref(0.0f),
      prev_d_term(0.0f), prev_u_unsat(0.0f), Kaw(1.0f),
      feedback_on(true), feedforward_on(false),
      antiwindup_on(true), backcalculation_on(true), bumpless_on(true),
      ff_gain(0.0f), ff_background(0.0f) {
    updateKaw();
  }

  PIDController() : PIDController(0, 0, 0, 0.01f) {}

  // Compute control output given reference and measurement
  float compute(float ref, float y) {
    float u_ff = 0.0f;
    if (feedforward_on && ff_gain > 0.0f) {
      u_ff = (ref - ff_background) / ff_gain;
    }

    if (!feedback_on) {
      prev_y = y;
      prev_ref = ref;
      return saturate(u_ff);
    }

    float error = ref - y;

    // Proportional with set-point weighting
    float p_term = Kp * (beta * ref - y);

    // Integral (forward Euler)
    integral += Ki * error * T;

    // Anti-windup: back-calculation (uses cached Kaw)
    if (antiwindup_on) {
      if (backcalculation_on) {
          float u_sat = saturate(prev_u_unsat);
          integral += Kaw * (u_sat - prev_u_unsat) * T;
      } else {
        // Clamp integrator so total output stays within actuator limits
        integral = constrain(integral, u_min - u_ff - p_term, u_max - u_ff - p_term);
      }
    }

    // Low-pass filtered derivative (skip entirely when Kd == 0)
    float d_term = 0.0f;
    if (Kd > 0.0f) {
      float d_raw = (T > 0.0f) ? -Kd * (y - prev_y) / T : 0.0f;
      d_term = alpha_d * d_raw + (1.0f - alpha_d) * prev_d_term;
      prev_d_term = d_term;
    }

    float u_fb = p_term + integral + d_term;
    float u_unsat = u_ff + u_fb;

    prev_y = y;
    prev_ref = ref;
    prev_u_unsat = u_unsat;

    return saturate(u_unsat);
  }

  void setGains(float _Kp, float _Ki, float _Kd) {
    setKp(_Kp);
    setKi(_Ki);
    setKd(_Kd);
  }

  void getGains(float &_Kp, float &_Ki, float &_Kd) const {
    _Kp = Kp; _Ki = Ki; _Kd = Kd;
  }

  void setBeta(float b) { beta = b; }
  float getBeta() const { return beta; }

  void setFeedback(bool on) { feedback_on = on; }
  bool getFeedback() const { return feedback_on; }

  void setFeedforward(bool on) { feedforward_on = on; }
  bool getFeedforward() const { return feedforward_on; }

  void setAntiWindup(bool on) { antiwindup_on = on; }
  bool getAntiWindup() const { return antiwindup_on; }

  void setBumpless(bool on) { bumpless_on = on; }
  bool getBumpless() const { return bumpless_on; }

  void setBackcalculation(bool on) { backcalculation_on = on; }
  bool getBackcalculation() const { return backcalculation_on; }

  void configureFeedforward(float G, float background) {
    ff_gain = G;
    ff_background = background;
  }

  // Bumpless transfer: adjust integrator so that the total feedback
  // output u_fb = p_term + integral + d_term stays continuous when
  // a gain changes.  p_term = Kp*(beta*ref - y), so a Kp change
  // produces delta_p = (Kp_new - Kp_old)*(beta*prev_ref - prev_y).
  // We absorb that into the integrator:  integral -= delta_p.
  void setKp(float _Kp) {
    float Kp_old = Kp;
    Kp = _Kp;
    if (bumpless_on) {
      integral -= (_Kp - Kp_old) * (beta * prev_ref - prev_y);
    }
    updateKaw();
  }

  // Ki bumpless: the running integral was accumulated with the old Ki.
  // Rescale so the contribution stays the same.
  void setKi(float _Ki) {
    if (bumpless_on && Ki != 0.0f && _Ki != 0.0f) {
      integral *= _Ki / Ki;
    }
    Ki = _Ki;
    updateKaw();
  }

  // Kd bumpless: rescale the filtered derivative term.
  void setKd(float _Kd) {
    if (bumpless_on && Kd != 0.0f && _Kd != 0.0f) {
      prev_d_term *= _Kd / Kd;
    }
    Kd = _Kd;
    updateKaw();
  }

  void reset() {
    integral = 0.0f;
    prev_y = 0.0f;
    prev_ref = 0.0f;
    prev_d_term = 0.0f;
    prev_u_unsat = 0.0f;
  }

  float getSamplingPeriod() const { return T; }
};

#endif
