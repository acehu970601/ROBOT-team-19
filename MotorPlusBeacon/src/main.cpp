#include <Arduino.h>
#include <Metro.h>

//Pins
#define ledPin                   13
#define leftMotorEnPin           3
#define rightMotorEnPin          4
#define leftMotorDirPin1         5
#define leftMotorDirPin2         6
#define rightMotorDirPin1        7
#define rightMotorDirPin2        8
#define photoTransistorPin       16

//Threshold values
#define beaconValueThre          220
#define beaconDiffThre           5
#define beaconCountThre          5

//State
typedef enum {
  STATE_FORWARD, STATE_REST, STATE_BEACON, STATE_BACKWARD
} States_t;

//Timer
IntervalTimer peakTracker;
static Metro restTimer = Metro(2000);
static Metro backTimer = Metro(1000);

//Variables
States_t state;
//Motor
const int forCoeff = 50;
const int backCoeff = 50;
const int turnCoeff = 30;
//Beacon
unsigned int photoTransistorVoltage = 2000;
unsigned int peakHeight = 2000;
unsigned int oldPeakHeight = 2000;
unsigned int beaconCount=0;

//Functions
//State transit
bool restTimerExpired(void);
bool beaconDetected(void);
bool backTimerExpired(void);
void startDetectingBeacon(void);
void moveBackward(void);
void rest(void);
//Motor
void leftMotorFor(int);
void rightMotorFor(int);
void leftMotorBack(int);
void rightMotorBack(int);
void leftMotorStop(void);
void rightMotorStop(void);
//Beacon
void detectBeacon(void);
void peakHeightComparison(void);

void setup() {
  //Turn on LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  //Motor
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);
  pinMode(rightMotorDirPin2, OUTPUT);
  //Beacon
  pinMode(photoTransistorPin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  switch (state) {
    case STATE_REST:
      if(restTimerExpired()) startDetectingBeacon();
      break;
    case STATE_BEACON:
      detectBeacon();
      if(beaconDetected()) moveBackward();
      break;
    case STATE_BACKWARD:
      if(backTimerExpired()) rest();
      break;
    default:
      break;    // Should never get into an unhandled state
  }
}

bool restTimerExpired(){
  return restTimer.check();
}

void startDetectingBeacon(){
  state = STATE_BEACON;
  leftMotorBack(turnCoeff);
  rightMotorFor(turnCoeff);
  peakTracker.begin(peakHeightComparison, 1000);
};

bool beaconDetected(){
  static int prevBeaconCount = 0;
  bool eventBeacon = false;
  if(beaconCount >= beaconCountThre && prevBeaconCount < beaconCountThre){
    eventBeacon = true;
  }
  prevBeaconCount = beaconCount;
  return eventBeacon;
};

void moveBackward(){
  state = STATE_BACKWARD;
  peakTracker.end();
  backTimer.reset();
  leftMotorBack(backCoeff);
  rightMotorBack(backCoeff);
}

bool backTimerExpired(){
  return backTimer.check();
}

void rest(){
  state = STATE_REST;
  restTimer.reset();
  leftMotorStop();
  rightMotorStop();
}

void leftMotorFor(int coeff){
  int dutyCycle = coeff*256*0.01;
  analogWrite(leftMotorEnPin, dutyCycle);
  digitalWrite(leftMotorDirPin1, HIGH);
  digitalWrite(leftMotorDirPin2, LOW);
};

void leftMotorBack(int coeff){
  int dutyCycle = coeff*256*0.01;
  analogWrite(leftMotorEnPin, dutyCycle);
  digitalWrite(leftMotorDirPin1, LOW);
  digitalWrite(leftMotorDirPin2, HIGH);
};

void rightMotorFor(int coeff){
  int dutyCycle = coeff*256*0.01;
  analogWrite(rightMotorEnPin, dutyCycle);
  digitalWrite(rightMotorDirPin1, HIGH);
  digitalWrite(rightMotorDirPin2, LOW);
};

void rightMotorBack(int coeff){
  int dutyCycle = coeff*256*0.01;
  analogWrite(rightMotorEnPin, dutyCycle);
  digitalWrite(rightMotorDirPin1, LOW);
  digitalWrite(rightMotorDirPin2, HIGH);
};

void leftMotorStop(){
  digitalWrite(leftMotorDirPin1, LOW);
  digitalWrite(leftMotorDirPin2, LOW);
};

void rightMotorStop(){
  digitalWrite(rightMotorDirPin1, LOW);
  digitalWrite(rightMotorDirPin2, LOW);
};

void detectBeacon(){
  photoTransistorVoltage = analogRead(photoTransistorPin);
  if(photoTransistorVoltage < peakHeight){
    peakHeight = photoTransistorVoltage;
  }
}

void peakHeightComparison(){
  if (abs(peakHeight-oldPeakHeight) < beaconDiffThre && peakHeight < beaconValueThre){
    beaconCount++;
  }
  oldPeakHeight=peakHeight;
  peakHeight = 2000;
};