#include <Arduino.h>
// define variable here
 
#define linedetector       22
#define LinethresholdLOW      800
#define LinethresholdHIGH     1000

const int ledPin = LED_BUILTIN;
u_int16_t voltageReader=0;

void readLinedetector(void);

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  pinMode(linedetector, INPUT);
  Serial.begin(96000);
}

void loop() {
  // put your main code here, to run repeatedly:
  readLinedetector();
}

void readLinedetector(void){
  voltageReader = analogRead(linedetector);
  // Serial.println(voltageReader);
  if(voltageReader>LinethresholdLOW && voltageReader<LinethresholdHIGH){
    Serial.println(voltageReader);
    Serial.println("Watch out, You reach the boundary!'");
  }
  else{
    // Serial.println("I am free");
  }
}