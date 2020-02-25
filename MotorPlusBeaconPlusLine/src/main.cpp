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

#define photoTransistorPin       17

#define rightBackLinePin         20
#define rightFrontLinePin        21
#define leftBackLinePin          22
#define leftFrontLinePin         23

//Threshold values
#define beaconValueThre          400
#define beaconDiffThre           35
#define beaconCountThre          3

#define LinethresholdLOW         900
#define LinethresholdHIGH        930
#define Errorthreshold           1000

//State
typedef enum {
  STATE_FORWARD, STATE_READY, STATE_BEACON, STATE_BACKWARD, STATE_FIRSTTURN, STATE_REST, STATE_PUSHLEFT
} States_t;

//Timer
IntervalTimer peakTracker;
static Metro readyTimer = Metro(3000);
static Metro firstTurnTimer = Metro(580);
static Metro pushLeftTimer = Metro(3000);

//Variables
States_t state = STATE_READY;
//Motor
const int forCoeff = 70;
const int backCoeff = 45; 
const int turnCoeff = 45;
//Beacon
unsigned int photoTransistorVoltage = 2000;
unsigned int peakHeight = 2000;
unsigned int oldPeakHeight = 2000;
unsigned int beaconCount=0;
//Line
int leftFrontLineVoltage=0;
int leftBackLineVoltage=0;
int rightFrontLineVoltage=0;
int rightBackLineVoltage=0;

//Functions
//Events
bool readyTimerExpired(void);
bool firstTurnTimerExpired(void);
bool beaconDetected(void);
bool leftFrontLineDetected(void);
bool leftBackLineDetected(void);
bool rightFrontLineDetected(void);
bool rightBackLineDetected(void);
bool pushLeftTimerExpired(void);
//Actions
void startDetectingBeacon(void);
void moveBackward(void);
void startFirstTurn(void);
void pushLeft(void);
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
  //Line
  pinMode(leftFrontLinePin, INPUT);
  pinMode(leftBackLinePin, INPUT);
  pinMode(rightFrontLinePin, INPUT);
  pinMode(rightBackLinePin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  switch (state) {
    case STATE_READY:
      if(readyTimerExpired()) startDetectingBeacon();
      break;
    case STATE_BEACON:
      detectBeacon();
      if(beaconDetected()) moveBackward();
      break;
    case STATE_BACKWARD:
//      rightBackLineVoltage = analogRead(rightBackLinePin);
//      Serial.println(rightBackLineVoltage);
      if(leftBackLineDetected()) startFirstTurn();
      if(rightBackLineDetected()) startFirstTurn();
      break;
    case STATE_FIRSTTURN:
      if(firstTurnTimerExpired()) pushLeft();
      break;
    case STATE_PUSHLEFT:
      if(pushLeftTimerExpired()) rest();
    case STATE_REST:
      break;
    default:
      break;    // Should never get into an unhandled state
  }
}

bool readyTimerExpired(){
  return readyTimer.check();
}

bool beaconDetected(){
  static int prevBeaconCount = 0;
  bool eventBeacon = false;
  if(beaconCount >= beaconCountThre && prevBeaconCount < beaconCountThre){
    eventBeacon = true;
  }
  prevBeaconCount = beaconCount;
  return eventBeacon;
};

bool firstTurnTimerExpired(){
  return firstTurnTimer.check();
}

bool leftFrontLineDetected(){
  leftFrontLineVoltage = analogRead(leftFrontLinePin);
  static int lineThres = LinethresholdHIGH;
  static int prevVol = 0;
  bool event = false;
  if(leftFrontLineVoltage > Errorthreshold){
    Serial.println("leftFrontLineError!");
    return false;
  }
  if(leftFrontLineVoltage > lineThres && prevVol <= lineThres){
    event = true;
    lineThres = LinethresholdLOW;
  }
  if(leftFrontLineVoltage <= lineThres && prevVol > lineThres){
    lineThres = LinethresholdHIGH;
  }
  prevVol = leftFrontLineVoltage;
  return event;
}

bool leftBackLineDetected(){
  leftBackLineVoltage = analogRead(leftBackLinePin);
  static int lineThres = LinethresholdHIGH;
  static int prevVol = 0;
  bool event = false;
  if(leftBackLineVoltage > Errorthreshold){
    Serial.println("leftBackLineError!");
    return false;
  }
  if(leftBackLineVoltage > lineThres && prevVol <= lineThres){
    event = true;
    lineThres = LinethresholdLOW;
  }
  if(leftBackLineVoltage <= lineThres && prevVol > lineThres){
    lineThres = LinethresholdHIGH;
  }
  prevVol = leftBackLineVoltage;
  return event;
}

bool rightFrontLineDetected(){
  rightFrontLineVoltage = analogRead(rightFrontLinePin);
  static int lineThres = LinethresholdHIGH;
  static int prevVol = 0;
  bool event = false;
  if(rightFrontLineVoltage > Errorthreshold){
    Serial.println("rightFrontLineError!");
    return false;
  }
  if(rightFrontLineVoltage > lineThres && prevVol <= lineThres){
    event = true;
    lineThres = LinethresholdLOW;
  }
  if(rightFrontLineVoltage <= lineThres && prevVol > lineThres){
    lineThres = LinethresholdHIGH;
  }
  prevVol = rightFrontLineVoltage;
  return event;
}

bool rightBackLineDetected(){
  rightBackLineVoltage = analogRead(rightBackLinePin);
  static int lineThres = LinethresholdHIGH;
  static int prevVol = 0;
  bool event = false;
  if(rightBackLineVoltage > Errorthreshold){
    Serial.println("rightBackLineError!");
    return false;
  }
  if(rightBackLineVoltage > lineThres && prevVol <= lineThres){
    event = true;
    lineThres = LinethresholdLOW;
  }
  if(rightBackLineVoltage <= lineThres && prevVol > lineThres){
    lineThres = LinethresholdHIGH;
  }
  prevVol = rightBackLineVoltage;
  return event;
}

bool pushLeftTimerExpired(){
  return pushLeftTimer.check();
}

void startDetectingBeacon(){
  state = STATE_BEACON;
  leftMotorBack(turnCoeff);
  rightMotorFor(turnCoeff);
  peakTracker.begin(peakHeightComparison, 1000);
};

void moveBackward(){
  state = STATE_BACKWARD;
  peakTracker.end();
  leftMotorBack(backCoeff);
  rightMotorBack(backCoeff);
}

void startFirstTurn(){
  state = STATE_FIRSTTURN;
  firstTurnTimer.reset();
  leftMotorBack(turnCoeff);
  rightMotorFor(turnCoeff);
};

void pushLeft(){
  state = STATE_PUSHLEFT;
  pushLeftTimer.reset();
  leftMotorFor(forCoeff);
  rightMotorFor(forCoeff);
}

void rest(){
  state = STATE_REST;
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
//  Serial.println(peakHeight);
  if (abs(peakHeight-oldPeakHeight) < beaconDiffThre && peakHeight < beaconValueThre){
    beaconCount++;
  }
  oldPeakHeight=peakHeight;
  peakHeight = 2000;
};