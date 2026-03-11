#ifndef SERIAL_COMMAND_PARSER_H
#define SERIAL_COMMAND_PARSER_H

#include <Arduino.h>
#include "led_driver.h"
#include "lux_sensor.h"
#include "pid_controller.h"
#include "data_buffer.h"
#include "performance_tracker.h"

// Desk state type (shared between main.ino and parser)
enum DeskState { DESK_OFF = 0, DESK_LOW = 1, DESK_HIGH = 2 };
extern void setDeskState(DeskState state);

extern float lux_background;
extern float static_gain;
extern float lux_reference;
extern float lux_measured;
extern bool control_active;
extern bool stream_y;
extern bool stream_u;
extern bool stream_r;
extern const float MAX_POWER_W;

// Global Objects
extern LedDriver   led;
extern LuxSensor   sensor;
extern PIDController pid;
extern DataBuffer    dataBuffer;
extern PerformanceTracker perf_tracker;

extern float doCalibrate();
extern float doMeasureGain();

class SerialCommandParser {
private:
    static const int CMD_BUF_SIZE = 64;
    char cmd_buf[CMD_BUF_SIZE];
    int cmd_len = 0;

public:
    // Non-blocking parse: reads all available characters without waiting.
    // Processes complete commands when '\n' or '\r' is received.
    void parse() {
        while (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (cmd_len == 0) continue;
                cmd_buf[cmd_len] = '\0';
                // Trim trailing spaces
                while (cmd_len > 0 && cmd_buf[cmd_len - 1] == ' ') {
                    cmd_buf[--cmd_len] = '\0';
                }
                if (cmd_len > 0) {
                    processCommand(cmd_len);
                }
                cmd_len = 0;
            } else if (cmd_len < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_len++] = c;
            }
        }
    }

private:
    void processCommand(int len) {
        // --- Single-word commands ---
        if (strcmp(cmd_buf, "cal") == 0) {
            Serial.print("Calibrating... ");
            doCalibrate();
            pid.configureFeedforward(static_gain, lux_background);
            Serial.print("bg="); Serial.println(lux_background, 2);
            return;
        }
        if (strcmp(cmd_buf, "gain") == 0) {
            Serial.print("Measuring... ");
            doMeasureGain();
            pid.configureFeedforward(static_gain, lux_background);
            Serial.print("G="); Serial.print(static_gain, 2); Serial.println(" lux/duty");
            return;
        }
        if (strcmp(cmd_buf, "B") == 0) {
            dataBuffer.startDump();
            return;
        }

        char c0 = cmd_buf[0];
        char *rest = (len >= 2) ? &cmd_buf[2] : nullptr;

        // --- GET Kp/Ki/Kd shortcuts: g Kp, g Ki, g Kd ---
        if (c0 == 'g' && rest) {
            if (strcmp(rest, "Kp") == 0) {
                float kp, ki, kd; pid.getGains(kp, ki, kd);
                Serial.print("Kp "); Serial.println(kp, 6); return;
            }
            if (strcmp(rest, "Ki") == 0) {
                float kp, ki, kd; pid.getGains(kp, ki, kd);
                Serial.print("Ki "); Serial.println(ki, 6); return;
            }
            if (strcmp(rest, "Kd") == 0) {
                float kp, ki, kd; pid.getGains(kp, ki, kd);
                Serial.print("Kd "); Serial.println(kd, 6); return;
            }
        }

        // --- GET shortcuts (no node ID): g <key> ---
        if (c0 == 'g' && len >= 3 && rest && strlen(rest) == 1) {
            char key = cmd_buf[2];
            switch (key) {
                case 'l': Serial.print("l "); Serial.println(lux_measured, 2); return;
                case 'r': Serial.print("r "); Serial.println(lux_reference, 2); return;
                case 'b': Serial.print("b "); Serial.println(lux_background, 2); return;
                case 'G': Serial.print("G "); Serial.println(static_gain, 2); return;
                case 'e': Serial.print("e "); Serial.println(lux_reference - lux_measured, 2); return;
                case 'm': Serial.print("m "); Serial.println(sensor.getM(), 4); return;
            }
            // Unknown key: fall through to Table 1 handlers below
        }

        // --- SET old style commands: s <key> <value(s)> ---
        if (c0 == 's' && rest && strlen(rest) > 0 && !(rest[0] == 'y' || rest[0] == 'u' || rest[0] == 'r')) { // exclude 's y/u/r <i>' (Stream)
            char key = rest[0];
            char *val_str = (strlen(rest) >= 2) ? &rest[2] : nullptr;

            // Individual gain setters with bumpless transfer: s Kp/Ki/Kd <val>
            if (strncmp(rest, "Kp ", 3) == 0) {
                pid.setKp(atof(rest + 3));
                float kp, ki, kd; pid.getGains(kp, ki, kd);
                Serial.print("Kp "); Serial.println(kp, 6); return;
            }
            if (strncmp(rest, "Ki ", 3) == 0) {
                pid.setKi(atof(rest + 3));
                float kp, ki, kd; pid.getGains(kp, ki, kd);
                Serial.print("Ki "); Serial.println(ki, 6); return;
            }
            if (strncmp(rest, "Kd ", 3) == 0) {
                pid.setKd(atof(rest + 3));
                float kp, ki, kd; pid.getGains(kp, ki, kd);
                Serial.print("Kd "); Serial.println(kd, 6); return;
            }

            switch (key) {
                case 'H': {
                    extern float lux_high;
                    extern DeskState desk_state;
                    if (!val_str) { Serial.println("err missing value"); break; }
                    lux_high = atof(val_str);
                    if (desk_state == DESK_HIGH) { lux_reference = lux_high; }
                    Serial.print("H "); Serial.println(lux_high, 2);
                    return;
                }
                case 'L': {
                    extern float lux_low;
                    extern DeskState desk_state;
                    if (!val_str) { Serial.println("err missing value"); break; }
                    lux_low = atof(val_str);
                    if (desk_state == DESK_LOW) { lux_reference = lux_low; }
                    Serial.print("L "); Serial.println(lux_low, 2);
                    return;
                }
                case 'p': {
                    if (!val_str) { Serial.println("err usage: s p <Kp> <Ki> <Kd>"); return; }
                    float kp, ki, kd;
                    char *tok1 = strtok(val_str, " ");
                    char *tok2 = strtok(nullptr, " ");
                    char *tok3 = strtok(nullptr, " ");
                    if (tok1 && tok2 && tok3) {
                        kp = atof(tok1); ki = atof(tok2); kd = atof(tok3);
                        pid.setGains(kp, ki, kd);
                        Serial.print("p "); Serial.print(kp, 6); Serial.print(' ');
                        Serial.print(ki, 6); Serial.print(' '); Serial.println(kd, 6);
                    } else { Serial.println("err usage: s p <Kp> <Ki> <Kd>"); }
                    return;
                }
                case 'b': {
                    if (!val_str) { Serial.println("err missing value"); return; }
                    pid.setBeta(atof(val_str));
                    Serial.print("beta "); Serial.println(pid.getBeta(), 4);
                    return;
                }
                case 'm': {
                    if (!val_str) { Serial.println("err missing value"); return; }
                    float new_m = atof(val_str);
                    sensor.setCalibration(new_m, sensor.getB());
                    Serial.print("m "); Serial.println(new_m, 4);
                    return;
                }
            }
        }

        // --- Toggle commands: c/f/w/b on|off ---
        if ((c0 == 'c' || c0 == 'f' || c0 == 'w' || c0 == 'b') && rest && (strcmp(rest, "on") == 0 || strcmp(rest, "off") == 0)) {
            bool on  = (strcmp(rest, "on") == 0);
            switch (c0) {
            case 'c':
                control_active = on;
                if (!on) { pid.reset(); }
                Serial.print("ctrl "); Serial.println(on ? "on" : "off");
                break;
            case 'f':
                pid.setFeedforward(on);
                Serial.print("ff "); Serial.println(on ? "on" : "off");
                break;
            case 'w':
                pid.setAntiWindup(on);
                Serial.print("aw "); Serial.println(on ? "on" : "off");
                break;
            case 'b':
                pid.setBumpless(on);
                Serial.print("bumpless "); Serial.println(on ? "on" : "off");
                break;
            }
            return;
        }

        int i;
        float val;
        char val_c;

        // u <i> <val> : Set duty cycle
        if (c0 == 'u' && sscanf(cmd_buf, "u %d %f", &i, &val) == 2) {
            if (i == 1 || i == 0) {
                control_active = false;
                led.setDutyCycle(val);
                Serial.println("ack");
            } else { Serial.println("err"); }
        }
        // g u <i> : Get duty cycle
        else if (c0 == 'g' && sscanf(cmd_buf, "g u %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("u "); Serial.print(i); Serial.print(" "); Serial.println(led.getDuty(), 4);
            } else { Serial.println("err"); }
        }
        // r <i> <val> : Set reference
        else if (c0 == 'r' && sscanf(cmd_buf, "r %d %f", &i, &val) == 2) {
            if (i == 1 || i == 0) {
                lux_reference = val;
                control_active = true;
                Serial.println("ack");
            } else { Serial.println("err"); }
        }
        // g r <i> : Get reference
        else if (c0 == 'g' && sscanf(cmd_buf, "g r %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("r "); Serial.print(i); Serial.print(" "); Serial.println(lux_reference, 2);
            } else { Serial.println("err"); }
        }
        // g y <i> : Measure actual illuminance
        else if (c0 == 'g' && sscanf(cmd_buf, "g y %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("y "); Serial.print(i); Serial.print(" "); Serial.println(lux_measured, 2);
            } else { Serial.println("err"); }
        }
        // g v <i> : Measure voltage
        else if (c0 == 'g' && sscanf(cmd_buf, "g v %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("v "); Serial.print(i); Serial.print(" "); Serial.println(sensor.readVoltage(), 4);
            } else { Serial.println("err"); }
        }
        // g A <i> : Read raw averaged ADC value
        else if (c0 == 'g' && sscanf(cmd_buf, "g A %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("A "); Serial.print(i); Serial.print(" "); Serial.println(sensor.readRawAdc(), 1);
            } else { Serial.println("err"); }
        }
        // o <i> <val> : Set occupancy
        else if (c0 == 'o' && sscanf(cmd_buf, "o %d %c", &i, &val_c) == 2) {
            if (i == 1 || i == 0) {
                if (val_c == 'o') { setDeskState(DESK_OFF); Serial.println("ack"); }
                else if (val_c == 'l') { setDeskState(DESK_LOW); Serial.println("ack"); }
                else if (val_c == 'h') { setDeskState(DESK_HIGH); Serial.println("ack"); }
                else { Serial.println("err"); }
            } else { Serial.println("err"); }
        }
        // g o <i> : Get occupancy
        else if (c0 == 'g' && sscanf(cmd_buf, "g o %d", &i) == 1) {
            extern DeskState desk_state;
            if (i == 1 || i == 0) {
                char state_c = 'o';
                if (desk_state == DESK_LOW) state_c = 'l';
                else if (desk_state == DESK_HIGH) state_c = 'h';
                Serial.print("o "); Serial.print(i); Serial.print(" "); Serial.println(state_c);
            } else { Serial.println("err"); }
        }
        // a <i> <val> : Set anti-windup
        else if (c0 == 'a' && sscanf(cmd_buf, "a %d %f", &i, &val) == 2) {
            if (i == 1 || i == 0) {
                pid.setAntiWindup(val > 0.5f);
                Serial.println("ack");
            } else { Serial.println("err"); }
        }
        // g a <i> : Get anti-windup
        else if (c0 == 'g' && sscanf(cmd_buf, "g a %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("a "); Serial.print(i); Serial.print(" "); Serial.println(pid.getAntiWindup() ? 1 : 0);
            } else { Serial.println("err"); }
        }
        // f <i> <val> : Set feedback
        else if (c0 == 'f' && sscanf(cmd_buf, "f %d %f", &i, &val) == 2) {
            if (i == 1 || i == 0) {
                pid.setFeedback(val > 0.5f);
                Serial.println("ack");
            } else { Serial.println("err"); }
        }
        // g f <i> : Get feedback Table 1
        else if (c0 == 'g' && sscanf(cmd_buf, "g f %d", &i) == 1 && len <= 6) {
            if (i == 1 || i == 0) {
                Serial.print("f "); Serial.print(i); Serial.print(" "); Serial.println(pid.getFeedback() ? 1 : 0);
            } else { Serial.println("err"); }
        }
        // g d <i> : Get external illuminance
        else if (c0 == 'g' && sscanf(cmd_buf, "g d %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("d "); Serial.print(i); Serial.print(" "); Serial.println(lux_background, 2);
            } else { Serial.println("err"); }
        }
        // g p <i> : Get instant power
        else if (c0 == 'g' && sscanf(cmd_buf, "g p %d", &i) == 1) {
            if (i == 1 || i == 0) {
                float power = led.getDuty() * MAX_POWER_W; 
                Serial.print("p "); Serial.print(i); Serial.print(" "); Serial.println(power, 4);
            } else { Serial.println("err"); }
        }
        // g t <i> : Get elapsed time
        else if (c0 == 'g' && sscanf(cmd_buf, "g t %d", &i) == 1) {
            if (i == 1 || i == 0) {
                float t = millis() / 1000.0f;
                Serial.print("t "); Serial.print(i); Serial.print(" "); Serial.println(t, 2);
            } else { Serial.println("err"); }
        }
        // g E <i> : Get average energy consumption
        else if (c0 == 'g' && sscanf(cmd_buf, "g E %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("E "); Serial.print(i); Serial.print(" "); Serial.println(perf_tracker.getTotalEnergy(), 4);
            } else { Serial.println("err"); }
        }
        // g V <i> : Get average visibility error
        else if (c0 == 'g' && sscanf(cmd_buf, "g V %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("V "); Serial.print(i); Serial.print(" "); Serial.println(perf_tracker.getAverageVisibility(), 4);
            } else { Serial.println("err"); }
        }
        // g F <i> : Get average flicker error
        else if (c0 == 'g' && sscanf(cmd_buf, "g F %d", &i) == 1) {
            if (i == 1 || i == 0) {
                Serial.print("F "); Serial.print(i); Serial.print(" "); Serial.println(perf_tracker.getAverageFlicker(), 4);
            } else { Serial.println("err"); }
        }
        // s <x> <i> : Start stream
        else if (c0 == 's' && sscanf(cmd_buf, "s %c %d", &val_c, &i) == 2) {
            if ((i == 1 || i == 0) && (val_c == 'y' || val_c == 'u' || val_c == 'r')) {
                if (val_c == 'y') stream_y = true;
                else if (val_c == 'u') stream_u = true;
                else              stream_r = true;
            } else { Serial.println("err"); }
        }
        // S <x> <i> : Stop stream
        else if (c0 == 'S' && sscanf(cmd_buf, "S %c %d", &val_c, &i) == 2) {
            if ((i == 1 || i == 0) && (val_c == 'y' || val_c == 'u' || val_c == 'r')) {
                if (val_c == 'y') stream_y = false;
                else if (val_c == 'u') stream_u = false;
                else              stream_r = false;
                Serial.println("ack");
            } else { Serial.println("err"); }
        }
        // g b <x> <i> : Get buffer (Table 1 format: b <x> <i> <val1>,<val2>,...<val_n>)
        else if (c0 == 'g' && sscanf(cmd_buf, "g b %c %d", &val_c, &i) == 2) {
            if ((i == 1 || i == 0) && (val_c == 'y' || val_c == 'u')) {
                dataBuffer.startVarDump(val_c, i);
            } else { Serial.println("err"); }
        }
    }
};

#endif // SERIAL_COMMAND_PARSER_H