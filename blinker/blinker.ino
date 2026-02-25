#include <Arduino.h>

const int LED_PIN = 15;

void setup() {
  // Start the Serial communication
  Serial.begin(115200);
  
  // Explicitly set the LED pin to behave as an output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // Turn the LED on
  Serial.println("LED is ON");  // Send confirmation to the computer
  delay(1000);                  // Wait for 1 second (1000 milliseconds)
  
  digitalWrite(LED_PIN, LOW);   // Turn the LED off
  Serial.println("LED is OFF"); // Send confirmation to the computer
  delay(1000);                  // Wait for 1 second
}