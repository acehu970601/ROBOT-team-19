#include <Arduino.h>
// define variable here
 
#define linedetector          22
#define LinethresholdLOW      800
#define LinethresholdHIGH     830
#define Errorthreshold        1000

const int ledPin = LED_BUILTIN;
u_int16_t voltageReader=0;

bool eventLineDetected(void);

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  pinMode(linedetector, INPUT);
  Serial.begin(96000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(eventLineDetected()){
    Serial.println("Watch out, You reach the boundary!'");
  }
}

bool eventLineDetected(void){
  voltageReader = analogRead(linedetector);
  //Serial.println(voltageReader);
  static int lineThres = LinethresholdHIGH;
  static int prevVol = 0;
  bool event = false;
  if(voltageReader > Errorthreshold){
    Serial.println("Error!");
    return false;
  }
  if(voltageReader > lineThres && prevVol <= lineThres){
    event = true;
    lineThres = LinethresholdLOW;
  }
  if(voltageReader <= lineThres && prevVol > lineThres){
    lineThres = LinethresholdHIGH;
  }
  prevVol = voltageReader;
  return event;
}