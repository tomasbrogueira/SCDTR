#ifndef PERFORMANCE_TRACKER_H
#define PERFORMANCE_TRACKER_H

#include <Arduino.h>

// Performance metrics matching the project specification:
//   Energy:     E = Pmax * sum( d_k * T )                       [Joules]
//   Visibility: V = (1/N) * sum( max(0, ref_k - lux_k) )       [LUX]
//   Flicker:    F = (1/N) * sum( f_k )                          [s^-1]
//     where f_k = |d_k - d_{k-1}| + |d_{k-1} - d_{k-2}|
//           only when (d_k - d_{k-1})*(d_{k-1} - d_{k-2}) < 0  (sign change)

class PerformanceTracker {
private:
    double accumulated_energy;
    double accumulated_visibility_error;
    double accumulated_flicker;

    float duty_k1;              // d_{k-1}
    float duty_k2;              // d_{k-2}
    bool  has_k1;               // true after first sample  (duty_k1 is valid)
    bool  has_k2;               // true after second sample (duty_k2 is valid)

    unsigned long metric_samples;
    float sample_period;
    float max_power_w;

public:
    PerformanceTracker(float sample_period_s, float max_power_watts = 0.05f) 
        : accumulated_energy(0.0), accumulated_visibility_error(0.0), 
          accumulated_flicker(0.0),
          duty_k1(0.0f), duty_k2(0.0f),
          has_k1(false), has_k2(false),
          metric_samples(0),
          sample_period(sample_period_s), max_power_w(max_power_watts) {}

    void update(float current_duty, float lux_measured, float lux_reference) {
        // Energy: E = Pmax * sum( d_k * T )
        accumulated_energy += current_duty * max_power_w * sample_period;
        
        // Visibility error: V = (1/N) * sum( max(0, ref - measured) )
        float ve = lux_reference - lux_measured;
        if (ve > 0.0f) {
            accumulated_visibility_error += ve;
        }
        
        // Flicker (spec formula): detect oscillation via sign change
        //   f_k = |d_k - d_{k-1}| + |d_{k-1} - d_{k-2}|
        //         when (d_k - d_{k-1}) * (d_{k-1} - d_{k-2}) < 0
        if (has_k2) {
            float diff1 = current_duty - duty_k1;    // d_k - d_{k-1}
            float diff2 = duty_k1 - duty_k2;         // d_{k-1} - d_{k-2}
            if (diff1 * diff2 < 0.0f) {
                accumulated_flicker += fabsf(diff1) + fabsf(diff2);
            }
        }
        
        // Shift duty history
        duty_k2 = duty_k1;
        duty_k1 = current_duty;
        if (!has_k1) has_k1 = true;
        else if (!has_k2) has_k2 = true;

        metric_samples++;
    }

    // Total energy consumed [Joules]
    double getTotalEnergy() const {
        return accumulated_energy;
    }

    // Average visibility error [LUX]
    double getAverageVisibility() const {
        return (metric_samples > 0) ? (accumulated_visibility_error / metric_samples) : 0.0;
    }

    // Average flicker [s^-1]
    double getAverageFlicker() const {
        return (metric_samples > 0) ? (accumulated_flicker / metric_samples) : 0.0;
    }

    void reset() {
        accumulated_energy = 0.0;
        accumulated_visibility_error = 0.0;
        accumulated_flicker = 0.0;
        duty_k1 = 0.0f;
        duty_k2 = 0.0f;
        has_k1 = false;
        has_k2 = false;
        metric_samples = 0;
    }
};

#endif // PERFORMANCE_TRACKER_H