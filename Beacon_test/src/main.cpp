//!!!! Note that for this code to work, one beacon must be placed in the front center of //the robot!!!

#include <Arduino.h>

//Pin definitions

  //We want an analog pin that is  able to withstand up to 20mA of current for photoTransistor detection circuit 
  #define photoTransistorPin       21
  #define beaconThreshold          220
  #define signal_diff              5
  #define count_slope              5

  const int ledPin = LED_BUILTIN; 
//Variables
  u_int16_t photoTransistorVoltage = 0;

  //The peak height of the most recent 1mS of scanning voltages
  u_int16_t newPeakHeight = 1000;

  //The peak height of the second most recent 1mS of scanning voltages
  u_int16_t oldPeakHeight = 0;
  // count how many times the difference in voltage in small
  u_int16_t count=0;

  bool beaconDetected = false;
  bool beginningBeaconSearch = true;

//Timers
  //calls the peakHeightComparison function once every milisecond
  IntervalTimer peakTracker;

//Functions

  void findBeacon(void);
  void peakHeightComparison(void);



void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  pinMode(photoTransistorPin, INPUT);
  Serial.begin(960000);
}

void loop() {
  // put your main code here, to run repeatedly:
  findBeacon();
  if (beaconDetected) {
    Serial.println("Beacon Detected");
  }
}

//In first run through, it begins the peakTracker timer. Besides this, if the beacon is not detected,
//then the function sets the newPeakHeight variable to the photoTransistorVoltage only if the newPeakHeight
//variable is greater than its previous version. This way, by the end of the 1 mS interval, newPeakHeight
//will equal the maximum peak height for that 1mS interval.
void findBeacon(void) {
  if (beginningBeaconSearch) {
    peakTracker.begin(peakHeightComparison, 1000000);
    beginningBeaconSearch = false;
  }
  if (beaconDetected == false) {
    photoTransistorVoltage = analogRead(photoTransistorPin);
    if (photoTransistorVoltage < newPeakHeight) {
      newPeakHeight = photoTransistorVoltage;
    }
  }
 
}

//if newpeakHeight is greater than or equal to the oldpeakHeight, then beacon is not found and 
//the oldPeakHeight is set to equal the newPeakHeight as the new cycle begins. If the newPeakHeight
//is less than the oldPeakHeight, then the beacon is found 
void peakHeightComparison(void) {
  Serial.println(newPeakHeight);
  // if (newPeakHeight >= oldPeakHeight) {
  //   oldPeakHeight = newPeakHeight;
  // } else {
  //   beaconDetected = true;
  // }

  // case when signal about to hit the max
  if (abs(newPeakHeight-oldPeakHeight)<signal_diff && newPeakHeight<beaconThreshold){
    count++;
  }
  else{
    oldPeakHeight=newPeakHeight;
  }
  if (count>=count_slope){
    beaconDetected = true;
  }

}