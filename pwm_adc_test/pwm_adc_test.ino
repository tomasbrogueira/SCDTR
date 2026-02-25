#include <Arduino.h>

const int LED_PIN = 15;
const int DAC_RANGE = 4096;
int counter = 0;
void setup() {// the setup function runs once 
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }
  
  analogReadResolution(12);    //default is 10 
  analogWriteResolution(12);
  analogWriteFreq(60000);      
  analogWriteRange(DAC_RANGE); //100% duty cycle

}
void loop() {// the loop function runs cyclically
  int read_adc;
  analogWrite(LED_PIN, counter); // set led PWM
  delay(10);                      
  //delay 10ms
  read_adc = analogRead(A0); // read analog voltage
  counter = counter + 5;
  if (counter > DAC_RANGE){ // if counter saturates    
    counter = 0;
  }   
  //format that Serial Plotter likes 
  Serial.print(0); Serial.print(" "); 
  Serial.print(DAC_RANGE); Serial.print(" "); 
  Serial.print(read_adc); Serial.print(" "); 
  Serial.print(counter); Serial.println(); 
}