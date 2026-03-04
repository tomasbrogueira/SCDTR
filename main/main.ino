#include <Arduino.h>
#include "led_driver.h"
#include "lux_sensor.h"
#include "calibrator.h"
#include "pid_controller.h"
#include "kalman_filter.h"
#include "data_buffer.h"

// ===================== Pin definitions =====================
const int LED_PIN = 15;
const int ADC_PIN = A0;

// ===================== LDR calibration constants =====================
// log10(R) = m * log10(Lux) + b
// Update these after running lux_calibrator and lux_m_finetune
const float LDR_M = -0.8f;
const float LDR_B = 6.5f;  // 6.15 and 6.85 so we simply take the average

// ===================== Control loop timing =====================
const unsigned long SAMPLE_PERIOD_US = 10000; // 10 ms → 100 Hz
const float T = SAMPLE_PERIOD_US / 1e6f;      // 0.01 s

// ===================== Desk states =====================
enum DeskState { DESK_OFF = 0, DESK_LOW = 1, DESK_HIGH = 2 };
float lux_high = 50;
float lux_low  = 25;
DeskState desk_state = DESK_OFF;

// ===================== Runtime state =====================
float lux_background = 0.0;
float static_gain = 0.0;
float lux_reference = 0.0;
float lux_measured = 0.0;
bool control_active = false;
bool streaming = false;
unsigned long last_sample_us = 0;

// ===================== Performance metrics =====================
double accumulated_energy = 0.0;
double accumulated_visibility_error = 0.0;
double accumulated_flicker = 0.0;
float last_duty_cycle_for_flicker = 0.0;
unsigned long metric_samples = 0;

// ===================== Objects =====================
LedDriver   led;
LuxSensor   sensor;
Calibrator  *calibrator = nullptr;
PIDController pid;
KalmanFilter  kalman;
DataBuffer    dataBuffer;

// ===================== Desk state management =====================
void setDeskState(DeskState state) {
  desk_state = state;
  switch (state) {
    case DESK_HIGH: lux_reference = lux_high; control_active = true;  break;
    case DESK_LOW:  lux_reference = lux_low;  control_active = true;  break;
    case DESK_OFF:  lux_reference = 0.0;      control_active = false; led.off(); break;
  }
  pid.setReference(lux_reference);
}

// ===================== Calibration wrappers =====================
float doCalibrate() {
  bool was_active = control_active;
  control_active = false;
  lux_background = calibrator->measureBackground();
  control_active = was_active;
  return lux_background;
}

float doMeasureGain() {
  bool was_active = control_active;
  control_active = false;
  static_gain = calibrator->measureStaticGain();
  control_active = was_active;
  return static_gain;
}

// ===================== Serial command protocol =====================
static const int CMD_BUF_SIZE = 64;
static char cmd_buf[CMD_BUF_SIZE];

char stream_var = '\0'; // 'y' or 'u' or '\0' for none
int stream_desk = 0;

void parseSerialCommand() {
  if (!Serial.available()) return;

  int len = Serial.readBytesUntil('\n', cmd_buf, CMD_BUF_SIZE - 1);
  cmd_buf[len] = '\0';

  // Trim trailing whitespace / carriage return
  while (len > 0 && (cmd_buf[len-1] == '\r' || cmd_buf[len-1] == ' ')) {
    cmd_buf[--len] = '\0';
  }
  if (len == 0) return;

  char c0 = cmd_buf[0];

  // Helper variables
  char c1, c2, c3;
  int i;
  float val;
  char val_c;

  // --- Parse Table 1 Commands ---

  // u <i> <val> : Set duty cycle
  if (c0 == 'u' && sscanf(cmd_buf, "u %d %f", &i, &val) == 2) {
    if (i == 1 || i == 0) {
      control_active = false;
      led.setDutyCycle(val);
      Serial.println("ack");
    } else {
      Serial.println("err");
    }
  }
  // g u <i> : Get duty cycle
  else if (c0 == 'g' && sscanf(cmd_buf, "g u %d", &i) == 1) {
    if (i == 1 || i == 0) {
      Serial.print("u "); Serial.print(i); Serial.print(" "); Serial.println(led.getDuty(), 4);
    } else {
      Serial.println("err");
    }
  }
  // r <i> <val> : Set reference
  else if (c0 == 'r' && sscanf(cmd_buf, "r %d %f", &i, &val) == 2) {
    if (i == 1 || i == 0) {
      lux_reference = val;
      pid.setReference(lux_reference);
      control_active = true;
      Serial.println("ack");
    } else {
      Serial.println("err");
    }
  }
  // g r <i> : Get reference
  else if (c0 == 'g' && sscanf(cmd_buf, "g r %d", &i) == 1) {
    if (i == 1 || i == 0) {
      Serial.print("r "); Serial.print(i); Serial.print(" "); Serial.println(lux_reference, 2);
    } else {
      Serial.println("err");
    }
  }
  // g y <i> : Measure actual illuminance
  else if (c0 == 'g' && sscanf(cmd_buf, "g y %d", &i) == 1) {
    if (i == 1 || i == 0) {
      Serial.print("y "); Serial.print(i); Serial.print(" "); Serial.println(lux_measured, 2);
    } else {
      Serial.println("err");
    }
  }
  // g v <i> : Measure voltage
  else if (c0 == 'g' && sscanf(cmd_buf, "g v %d", &i) == 1) {
    if (i == 1 || i == 0) {
      Serial.print("v "); Serial.print(i); Serial.print(" "); Serial.println(sensor.readVoltage(), 4);
    } else {
      Serial.println("err");
    }
  }
  // o <i> <val> : Set occupancy ('o', 'l', 'h')
  else if (c0 == 'o' && sscanf(cmd_buf, "o %d %c", &i, &val_c) == 2) {
    if (i == 1 || i == 0) {
      if (val_c == 'o') { setDeskState(DESK_OFF); Serial.println("ack"); }
      else if (val_c == 'l') { setDeskState(DESK_LOW); Serial.println("ack"); }
      else if (val_c == 'h') { setDeskState(DESK_HIGH); Serial.println("ack"); }
      else { Serial.println("err"); }
    } else {
      Serial.println("err");
    }
  }
  // g o <i> : Get occupancy
  else if (c0 == 'g' && sscanf(cmd_buf, "g o %d", &i) == 1) {
    if (i == 1 || i == 0) {
      char state_c = 'o';
      if (desk_state == DESK_LOW) state_c = 'l';
      else if (desk_state == DESK_HIGH) state_c = 'h';
      Serial.print("o "); Serial.print(i); Serial.print(" "); Serial.println(state_c);
    } else {
      Serial.println("err");
    }
  }
  // a <i> <val> : Set anti-windup (0, 1)
  else if (c0 == 'a' && sscanf(cmd_buf, "a %d %f", &i, &val) == 2) {
    if (i == 1 || i == 0) {
      pid.setAntiWindup(val > 0.5f);
      Serial.println("ack");
    } else {
      Serial.println("err");
    }
  }
  // g a <i> : Get anti-windup
  else if (c0 == 'g' && sscanf(cmd_buf, "g a %d", &i) == 1) {
    if (i == 1 || i == 0) {
      Serial.print("a "); Serial.print(i); Serial.print(" "); Serial.println(pid.getAntiWindup() ? 1 : 0);
    } else {
      Serial.println("err");
    }
  }
  // f <i> <val> : Set feedback (0, 1)
  else if (c0 == 'f' && sscanf(cmd_buf, "f %d %f", &i, &val) == 2) {
    if (i == 1 || i == 0) {
      pid.setFeedback(val > 0.5f);
      Serial.println("ack");
    } else {
      Serial.println("err");
    }
  }
  // g f <i> : Get feedback
  else if (c0 == 'g' && sscanf(cmd_buf, "g f %d", &i) == 1) {
    if (i == 1 || i == 0) {
      Serial.print("f "); Serial.print(i); Serial.print(" "); Serial.println(pid.getFeedback() ? 1 : 0);
    } else {
      Serial.println("err");
    }
  }
  // g d <i> : Get external illuminance
  else if (c0 == 'g' && sscanf(cmd_buf, "g d %d", &i) == 1) {
    if (i == 1 || i == 0) {
      Serial.print("d "); Serial.print(i); Serial.print(" "); Serial.println(lux_background, 2);
    } else {
      Serial.println("err");
    }
  }
  // g p <i> : Get instant power
  else if (c0 == 'g' && sscanf(cmd_buf, "g p %d", &i) == 1) {
    if (i == 1 || i == 0) {
      // Approximate power using PWM duty cycle, multiply by max power (e.g. 0.05W)
      float power = led.getDuty() * 0.05f; 
      Serial.print("p "); Serial.print(i); Serial.print(" "); Serial.println(power, 4);
    } else {
      Serial.println("err");
    }
  }
  // g t <i> : Get elapsed time
  else if (c0 == 'g' && sscanf(cmd_buf, "g t %d", &i) == 1) {
    if (i == 1 || i == 0) {
      float t = millis() / 1000.0f;
      Serial.print("t "); Serial.print(i); Serial.print(" "); Serial.println(t, 2);
    } else {
      Serial.println("err");
    }
  }
  // g E <i> : Get average energy consumption
  else if (c0 == 'g' && sscanf(cmd_buf, "g E %d", &i) == 1) {
    if (i == 1 || i == 0) {
      Serial.print("E "); Serial.print(i); Serial.print(" "); Serial.println(accumulated_energy, 4);
    } else {
      Serial.println("err");
    }
  }
  // g V <i> : Get average visibility error
  else if (c0 == 'g' && sscanf(cmd_buf, "g V %d", &i) == 1) {
    if (i == 1 || i == 0) {
      double avg_vis = (metric_samples > 0) ? (accumulated_visibility_error / metric_samples) : 0.0;
      Serial.print("V "); Serial.print(i); Serial.print(" "); Serial.println(avg_vis, 4);
    } else {
      Serial.println("err");
    }
  }
  // g F <i> : Get average flicker error
  else if (c0 == 'g' && sscanf(cmd_buf, "g F %d", &i) == 1) {
    if (i == 1 || i == 0) {
      double avg_flicker = (metric_samples > 0) ? (accumulated_flicker / (metric_samples * T)) : 0.0;
      Serial.print("F "); Serial.print(i); Serial.print(" "); Serial.println(avg_flicker, 4);
    } else {
      Serial.println("err");
    }
  }
  // s <x> <i> : Start stream
  else if (c0 == 's' && sscanf(cmd_buf, "s %c %d", &val_c, &i) == 2) {
    if ((i == 1 || i == 0) && (val_c == 'y' || val_c == 'u')) {
      stream_var = val_c;
      stream_desk = i;
      // The stream output is handled in the main loop
    } else {
      Serial.println("err");
    }
  }
  // S <x> <i> : Stop stream
  else if (c0 == 'S' && sscanf(cmd_buf, "S %c %d", &val_c, &i) == 2) {
    if (i == 1 || i == 0) {
      if (stream_var == val_c) stream_var = '\0';
      Serial.println("ack");
    } else {
      Serial.println("err");
    }
  }
  // g b <x> <i> : Get buffer
  else if (c0 == 'g' && sscanf(cmd_buf, "g b %c %d", &val_c, &i) == 2) {
    if ((i == 1 || i == 0) && (val_c == 'y' || val_c == 'u')) {
      // For this implementation, we can just trigger buffer dump or format it.
      // E.g., print "b x i " then dump.
      Serial.print("b "); Serial.print(val_c); Serial.print(" "); Serial.print(i); Serial.print(" ");
      // dataBuffer implementation depends on the class. Here we just start the dump.
      dataBuffer.startDump(); 
      // The rest is handled in the main loop, although the format 'val1, val2' needs to be correct.
      // Assuming dataBuffer output can be adjusted if needed, or left as is for now.
    } else {
      Serial.println("err");
    }
  }
  // Unrecognized command
  else {
    Serial.println("err unknown command");
  }
}

// ===================== Setup & Loop =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialise hardware objects (deferred init for globals)
  led.init(LED_PIN);
  sensor.init(ADC_PIN, LDR_M, LDR_B, 10000.0f, 12, 4, 5); // 5 is the median_window_size

  // Calibrator needs references to led & sensor
  static Calibrator cal(led, sensor, 300, 11);
  calibrator = &cal;

  Serial.println("=== Luminaire Controller ===\n");

  Serial.print("Background... ");
  doCalibrate();
  Serial.print("bg="); Serial.println(lux_background, 2);

  Serial.print("Static gain... ");
  doMeasureGain();
  Serial.print("G="); Serial.print(static_gain, 2); Serial.println(" lux/duty");

  // Compute PID gains from static gain
  float Kp = (static_gain > 0) ? 1.0f / static_gain : 0.01f;
  float Ki = Kp * 2.0f;   // integral time ≈ 0.5 s
  float Kd = 0.0f;        // derivative not needed (RC filter provides damping)

  pid = PIDController(Kp, Ki, Kd, T, 0.0f, 1.0f);
  pid.setAntiWindup(true);
  pid.setFeedforward(true);
  pid.setFeedback(true);
  pid.configureFeedforward(static_gain, lux_background);

  // Seed Kalman filter with boot calibration values
  kalman.init(static_gain, lux_background);

  Serial.print("PID: Kp="); Serial.print(Kp, 6);
  Serial.print(" Ki="); Serial.print(Ki, 6);
  Serial.print(" Kd="); Serial.println(Kd, 6);

  Serial.println("\nGET:    g l|d|r|o|p|b|G|e|k");
  Serial.println("SET:    s r|d|o|H|L|p|b <val>");
  Serial.println("TOGGLE: c|f|w on|off");
  Serial.println("STREAM: S on|off");
  Serial.println("OTHER:  B | cal | gain\n");

  setDeskState(DESK_OFF);
  last_sample_us = micros();
}

void loop() {
  parseSerialCommand();

  // Non-blocking buffer dump (prints a chunk each iteration)
  if (dataBuffer.isDumping()) {
    dataBuffer.dumpChunk();
  }

  unsigned long now = micros();
  if (now - last_sample_us < SAMPLE_PERIOD_US) return;
  last_sample_us += SAMPLE_PERIOD_US;

  // Read sensor — raw for Kalman, filtered for PID
  float lux_raw = sensor.readRaw();
  lux_measured = sensor.readMedianFiltered(); // Using Quickselect median filter as per the professor's hint

  // Kalman filter: update G and background estimates with RAW data
  // (EMA-filtered data would corrupt the noise model)
  kalman.update(led.getDuty(), lux_raw);
  static_gain = kalman.getGain();
  lux_background = kalman.getBackground();
  pid.configureFeedforward(static_gain, lux_background);

  // Control
  if (control_active) {
    float u = pid.compute(lux_reference, lux_measured);
    led.setDutyCycle(u);
  }

  // Buffer
  dataBuffer.push(lux_measured, led.getDuty(), lux_reference);

  // Performance metrics update
  float current_duty = led.getDuty();
  float current_power_w = current_duty * 0.05f; // Approx max 50mW
  
  accumulated_energy += current_power_w * T; // Energy in Joules
  
  float ve = lux_reference - lux_measured;
  if(ve > 0) {
    accumulated_visibility_error += ve;
  }
  
  float duty_diff = current_duty - last_duty_cycle_for_flicker;
  if (duty_diff < 0) duty_diff = -duty_diff;
  accumulated_flicker += duty_diff;
  
  last_duty_cycle_for_flicker = current_duty;
  metric_samples++;

  // Stream
  if (stream_var != '\0') {
    Serial.print("s "); Serial.print(stream_var); Serial.print(" "); Serial.print(stream_desk); Serial.print(" ");
    if (stream_var == 'y') {
      Serial.print(lux_measured, 4);
    } else if (stream_var == 'u') {
      Serial.print(led.getDuty(), 4);
    }
    Serial.print(" "); Serial.println(millis());
  }
}