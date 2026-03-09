#include <Arduino.h>
#include "led_driver.h"
#include "lux_sensor.h"
#include "calibrator.h"
#include "pid_controller.h"
#include "data_buffer.h"
#include "performance_tracker.h"
#include "serial_command_parser.h"

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
// DeskState enum is defined in serial_command_parser.h
float lux_high = 50;
float lux_low  = 25;
DeskState desk_state = DESK_OFF;

// ===================== Runtime state =====================
float lux_background = 0.0;
float static_gain = 0.0;
float lux_reference = 0.0;
float lux_measured = 0.0;
bool control_active = false;
bool stream_y = false;
bool stream_u = false;
bool stream_r = false;
unsigned long last_sample_us = 0;

// ===================== Shared constants =====================
const float MAX_POWER_W = 0.05f;

// ===================== Performance metrics =====================
PerformanceTracker perf_tracker(T, MAX_POWER_W);

// ===================== Objects =====================
LedDriver   led;
LuxSensor   sensor;
Calibrator  *calibrator = nullptr;
PIDController pid;
DataBuffer    dataBuffer;
SerialCommandParser commandParser;

// ===================== Desk state management =====================
void setDeskState(DeskState state) {
  desk_state = state;
  switch (state) {
    case DESK_HIGH: lux_reference = lux_high; control_active = true;  break;
    case DESK_LOW:  lux_reference = lux_low;  control_active = true;  break;
    case DESK_OFF:  lux_reference = 0.0;      control_active = false; led.off(); break;
  }
}

// ===================== Calibration wrappers =====================
float doCalibrate() {
  bool was_active = control_active;
  control_active = false;
  lux_background = calibrator->measureBackground();
  last_sample_us = micros();  // prevent control burst after blocking delay
  control_active = was_active;
  return lux_background;
}

float doMeasureGain() {
  bool was_active = control_active;
  control_active = false;
  static_gain = calibrator->measureStaticGain();
  last_sample_us = micros();  // prevent control burst after blocking delay
  control_active = was_active;
  return static_gain;
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
  float Kp = (static_gain > 0) ? 0.75f / static_gain : 0.01f;
  float Ki = Kp * 2.0f;   // Ki/Kp = 2
  float Kd = 0.0f;        // derivative not needed (RC filter provides damping)

  pid = PIDController(Kp, Ki, Kd, T, 0.0f, 1.0f);
  pid.setAntiWindup(true);
  pid.setFeedforward(true);
  pid.setFeedback(true);
  pid.configureFeedforward(static_gain, lux_background);
  pid.setBeta(0.70f);

  Serial.print("PID: Kp="); Serial.print(Kp, 6);
  Serial.print(" Ki="); Serial.print(Ki, 6);
  Serial.print(" Kd="); Serial.println(Kd, 6);

  Serial.println("GET:    g <key> [<i>]  (l,r,b,G,e | y,u,o,d,p,E,V,F,...)");
  Serial.println("SET:    s H|L|p|b <val> | r|u|o <i> <val>");
  Serial.println("TOGGLE: c|f|w on|off");
  Serial.println("STREAM: s <y|u|r> <i> / S <y|u|r> <i>");
  Serial.println("OTHER:  B | cal | gain\n");

  setDeskState(DESK_OFF);
  last_sample_us = micros();
}

void loop() {
  commandParser.parse();

  // Non-blocking buffer dumps (prints a chunk each iteration)
  if (dataBuffer.isDumping()) {
    dataBuffer.dumpChunk();
  }
  if (dataBuffer.isVarDumping()) {
    dataBuffer.varDumpChunk();
  }

  unsigned long now = micros();
  if (now - last_sample_us < SAMPLE_PERIOD_US) return;
  last_sample_us += SAMPLE_PERIOD_US;

  lux_measured = sensor.readMedianFiltered();

  // Control
  if (control_active) {
    float u = pid.compute(lux_reference, lux_measured);
    led.setDutyCycle(u);
  }

  // Buffer
  dataBuffer.push(lux_measured, led.getDuty(), lux_reference);

  // Performance metrics update
  perf_tracker.update(led.getDuty(), lux_measured, lux_reference);

  // Stream (y, u, r can be active simultaneously)
  if (stream_y || stream_u || stream_r) {
    unsigned long now_ms = millis();
    if (stream_y) {
      Serial.print("s y 1 "); Serial.print(lux_measured, 4); Serial.print(" "); Serial.println(now_ms);
    }
    if (stream_u) {
      Serial.print("s u 1 "); Serial.print(led.getDuty(), 4); Serial.print(" "); Serial.println(now_ms);
    }
    if (stream_r) {
      Serial.print("s r 1 "); Serial.print(lux_reference, 4); Serial.print(" "); Serial.println(now_ms);
    }
  }
}