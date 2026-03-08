#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <Arduino.h>

struct Sample {
  unsigned long timestamp_ms;  // millis() at time of push
  float lux;
  float duty;
  float reference;
};

class DataBuffer {
private:
  static const int CAPACITY = 6000; // 1 minute at 100 Hz
  Sample buffer[CAPACITY];
  int head;
  int count;

  // Non-blocking full dump state (B command)
  bool  dumping;
  int   dump_cursor;    // how many samples have been printed so far
  int   dump_start;     // starting index in the ring buffer

  // Non-blocking variable dump state (g b <x> <i> command)
  bool  var_dumping;
  char  var_dump_type;  // 'y' for lux, 'u' for duty
  int   var_dump_cursor;
  int   var_dump_start;

public:
  DataBuffer() : head(0), count(0),
                 dumping(false), dump_cursor(0), dump_start(0),
                 var_dumping(false), var_dump_type('\0'), var_dump_cursor(0), var_dump_start(0) {}

  void push(float lux, float duty, float ref) {
    buffer[head] = {millis(), lux, duty, ref};
    head = (head + 1) % CAPACITY;
    if (count < CAPACITY) count++;
  }

  // Start a non-blocking dump. Call dumpChunk() each loop iteration.
  void startDump() {
    Serial.println("i,t_ms,lux,duty,ref");
    dumping = true;
    dump_cursor = 0;
    dump_start = (count < CAPACITY) ? 0 : head;
  }

  // Print up to `chunk_size` lines per call. Returns true when done.
  // Call this from loop() to avoid blocking the control loop.
  bool dumpChunk(int chunk_size = 100) {
    if (!dumping) return true;
    int end = min(dump_cursor + chunk_size, count);
    for (int i = dump_cursor; i < end; i++) {
      int idx = (dump_start + i) % CAPACITY;
      Serial.print(i);
      Serial.print(',');
      Serial.print(buffer[idx].timestamp_ms);
      Serial.print(',');
      Serial.print(buffer[idx].lux, 2);
      Serial.print(',');
      Serial.print(buffer[idx].duty, 4);
      Serial.print(',');
      Serial.println(buffer[idx].reference, 2);
    }
    dump_cursor = end;
    if (dump_cursor >= count) {
      Serial.print("Total samples: ");
      Serial.println(count);
      dumping = false;
      return true;
    }
    return false;
  }

  bool isDumping() const { return dumping; }

  // --- Variable-specific dump for Table 1: g b <x> <i> ---
  // Response format: b <x> <i> <val1>,<val2>,...,<val_n>\n
  void startVarDump(char var, int desk) {
    Serial.print("b "); Serial.print(var); Serial.print(" "); Serial.print(desk); Serial.print(" ");
    var_dump_type = var;
    var_dumping = true;
    var_dump_cursor = 0;
    var_dump_start = (count < CAPACITY) ? 0 : head;
  }

  // Print a chunk of comma-separated values per call. Returns true when done.
  bool varDumpChunk(int chunk_size = 200) {
    if (!var_dumping) return true;
    int end = min(var_dump_cursor + chunk_size, count);
    for (int i = var_dump_cursor; i < end; i++) {
      int idx = (var_dump_start + i) % CAPACITY;
      if (i > 0) Serial.print(',');
      if (var_dump_type == 'y') Serial.print(buffer[idx].lux, 2);
      else                      Serial.print(buffer[idx].duty, 4);
    }
    var_dump_cursor = end;
    if (var_dump_cursor >= count) {
      Serial.println();  // terminate line
      var_dumping = false;
      return true;
    }
    return false;
  }

  bool isVarDumping() const { return var_dumping; }
  int  size() const { return count; }
};

#endif
