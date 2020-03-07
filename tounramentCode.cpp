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

#define leftFrontUltraTrigPin     13
#define leftFrontUltraEchoPin     12
#define rightFrontUltraTrigPin    14
#define rightFrontUltraEchoPin    11
#define leftBackUltraTrigPin      15
#define leftBackUltraEchoPin      10
#define rightBackUltraTrigPin     16
#define rightBackUltraEchoPin     9

//Threshold values
#define beaconDiffThre           35
#define beaconCountThre          1

#define LinethresholdLOW         900
#define LinethresholdHIGH        930
#define Errorthreshold           1000

#define wallThresHigh            80
#define wallThresLow             90



//State
typedef enum {
  STATE_FORWARD, STATE_READY,STATE_FIND_THRESHOLD, STATE_BEACON, STATE_BACKWARD, STATE_FIRSTTURN, STATE_REST,
  ATTACKING_LEFT_WALL,MOVING_FORWARD_TO_ATTACK,TURNING_CLOCKWISE_TO_ATTACK,
  ATTACKING_RIGHT_WALL,MOVING_BACKWARD_TO_ATTACK,TURNING_COUNTER_CLOCKWISE_TO_ATTACK
} States_t;
States_t innerState=MOVING_FORWARD_TO_ATTACK;

//Timer
IntervalTimer peakTracker;
IntervalTimer unitTimer;
static Metro readyTimer = Metro(2000);
static Metro firstTurnTimer = Metro(3000);
static Metro checkThresholdTimer = Metro(7000);
static Metro pushLeftTimer = Metro(3000);
static Metro wallTimer=Metro(1000);
static Metro totalTime = Metro(50000);
static Metro rightWallTimer = Metro(1000);

//Variables
States_t state = STATE_READY;
//Motor
const int forCoeff = 60;
const int backCoeff = 40;
const int findCoeff = 30;
const int turnCoeff = 30;
//Beacon
unsigned int photoTransistorVoltage = 2000;
unsigned int peakHeight = 2000;
unsigned int prevVoltage = 0;
unsigned int currVoltage = 2000;
unsigned int beaconValueThre= 2000;
unsigned int oldPeakHeight = 2000;
unsigned int beaconCount=0;
unsigned int thresholdCount=0;
unsigned int MaxCountThreshold=4;
unsigned int rightWallDetectionCounter = 0;
//Line
int leftFrontLineVoltage=0;
int leftBackLineVoltage=0;
int rightFrontLineVoltage=0;
int rightBackLineVoltage=0;
//Wall
bool leftWallDetected=false;
bool rightWallDetected=false;
//Ultrasonic
const int trigPeriod = 20000;
int timer = 0;
int leftFrontEchoRiseTime = 0;
int leftFrontEchoFallTime = 0;
int rightFrontEchoRiseTime = 0;
int rightFrontEchoFallTime = 0;
int leftBackEchoRiseTime = 0;
int leftBackEchoFallTime = 0;
int rightBackEchoRiseTime = 0;
int rightBackEchoFallTime = 0;
int leftFrontDis = 8888888;
int rightFrontDis = 8888888;
int leftBackDis = 8888888;
int rightBackDis = 8888888;

//Functions
//Events
bool readyTimerExpired(void);
bool firstTurnTimerExpired(void);
bool checkThresholdExpired(void);
bool checkwallTimerexpired(void);
bool beaconDetected(void);
bool leftFrontLineDetected(void);
bool leftBackLineDetected(void);
bool rightFrontLineDetected(void);
bool rightBackLineDetected(void);
bool eventFrontWallTouched(void);
bool eventBackWallTouched(void);
bool wallIsClose(void);
bool rightWallIsClose(void);
void checkRightWallTimer(void);

//Actions
void startDetectingThreshold(void);
void startDetectingBeacon(void);
void moveBackward(void);
void startFirstTurn(void);
void rest(void); 

//Line
void checkFrontRightLine(void);
void checkFrontLeftLine(void);
void checkBackRightLine(void);
void checkBackLeftLine(void);

//Motor
void leftMotorFor(int);
void rightMotorFor(int);
void leftMotorBack(int);
void rightMotorBack(int);
void leftMotorStop(void);
void rightMotorStop(void);
void changeTurnDirection(int coeff);
//Beacon
void detectBeacon(void);
void detectBeaconThreshold(void);
void peakHeightComparison(void);
void findBeaconThreshold(void);
//Wall
void checkWall(void);
void startAttackingRightWall(void);
void startAttackingLeft(void);
void startAttackingRight(void);
void checkRightWall(void);

//Ultrasonoic
void timerCount(void);
void leftFrontEchoRise(void);
void leftFrontEchoFall(void);
void rightFrontEchoRise(void);
void rightFrontEchoFall(void);
void leftBackEchoRise(void);
void leftBackEchoFall(void);
void rightBackEchoRise(void);
void rightBackEchoFall(void);

void checkTotalTime(void);



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
  //Ultrasonic
  pinMode(leftFrontUltraTrigPin, OUTPUT);
  pinMode(rightFrontUltraTrigPin, OUTPUT);
  pinMode(leftBackUltraTrigPin, OUTPUT);
  pinMode(rightBackUltraTrigPin, OUTPUT);
  digitalWrite(leftFrontUltraTrigPin, LOW);  
  digitalWrite(rightFrontUltraTrigPin, LOW);
  digitalWrite(leftBackUltraTrigPin, LOW);
  digitalWrite(rightBackUltraTrigPin, LOW);
  pinMode(leftFrontUltraEchoPin, INPUT);
  pinMode(rightFrontUltraEchoPin, INPUT);
  pinMode(leftBackUltraEchoPin, INPUT);
  pinMode(rightBackUltraEchoPin, INPUT);

  unitTimer.begin(timerCount, 5);
  attachInterrupt(digitalPinToInterrupt(leftFrontUltraEchoPin), leftFrontEchoRise, RISING);
  attachInterrupt(digitalPinToInterrupt(leftFrontUltraEchoPin), leftFrontEchoFall, FALLING);
  attachInterrupt(digitalPinToInterrupt(rightFrontUltraEchoPin), rightFrontEchoRise, RISING);
  attachInterrupt(digitalPinToInterrupt(rightFrontUltraEchoPin), rightFrontEchoFall, FALLING); 
  attachInterrupt(digitalPinToInterrupt(leftBackUltraEchoPin), leftBackEchoRise, RISING);
  attachInterrupt(digitalPinToInterrupt(leftBackUltraEchoPin), leftBackEchoFall, FALLING);
  attachInterrupt(digitalPinToInterrupt(rightBackUltraEchoPin), rightBackEchoRise, RISING);
  attachInterrupt(digitalPinToInterrupt(rightBackUltraEchoPin), rightBackEchoFall, FALLING); 
  
  Serial.begin(9600);
  
}

void loop() {
  checkTotalTime();
  switch (state) {
    case STATE_READY:
      if(readyTimerExpired()) startDetectingThreshold();
      break;
    case STATE_FIND_THRESHOLD:
      detectBeaconThreshold();
      if(checkThresholdExpired()) {
        startDetectingBeacon();
      }
      break;
    case STATE_BEACON:
      detectBeacon();
      if(beaconDetected()) moveBackward();
      // if(beaconDetected()) {
      //   rest();
      //   }
      break;
    case STATE_BACKWARD:
      if(leftBackLineDetected() || rightBackLineDetected()) startFirstTurn();
      // if(leftBackLineDetected()) rest();
      // if(rightBackLineDetected()) rest();
      break;
    case STATE_FIRSTTURN:
      if(leftFrontLineDetected()) startAttackingLeft();
      break;
    case ATTACKING_LEFT_WALL:
      switch (innerState) {
        case MOVING_FORWARD_TO_ATTACK:
          checkFrontRightLine();
          checkWall();
          break;
        case TURNING_CLOCKWISE_TO_ATTACK:
          checkFrontLeftLine();
          checkWall();
          break;
      }
      //  CAUTION:might expire before reset!!
      if (leftWallDetected == true && checkwallTimerexpired())  startAttackingRight();
      break;
    case ATTACKING_RIGHT_WALL:
      switch (innerState) {
        case MOVING_BACKWARD_TO_ATTACK:
          checkBackRightLine();
          //checkRightWall();
          break;
        case TURNING_COUNTER_CLOCKWISE_TO_ATTACK:
          checkBackLeftLine();
          //checkRightWall();
          break;
      }
      break;  
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
  // static int prevBeaconCount = 0;
  bool eventBeacon = false;
  if(beaconCount >= beaconCountThre){
    eventBeacon = true;
  }
  return eventBeacon;
};

bool firstTurnTimerExpired(){
  return firstTurnTimer.check();
}
bool checkThresholdExpired(){
  return checkThresholdTimer.check();
}
bool checkwallTimerexpired(){
  return wallTimer.check();
}
bool leftFrontLineDetected(){
  leftFrontLineVoltage = analogRead(leftFrontLinePin);
  static int lineThres = LinethresholdHIGH;
  static int prevVol = 0;
  bool event = false;
  Serial.println(leftFrontLineVoltage);
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
  Serial.println(rightFrontLineVoltage);
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

// bool pushLeftTimerExpired(){
//   return pushLeftTimer.check();
// }

void startDetectingThreshold(){
  state= STATE_FIND_THRESHOLD;
  leftMotorFor(findCoeff);
  rightMotorBack(findCoeff);
  checkThresholdTimer.reset();

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
  // firstTurnTimer.reset();
  leftMotorBack(turnCoeff);
  rightMotorFor(turnCoeff);
};

// void pushLeft(){
//   state = STATE_PUSHLEFT;
//   pushLeftTimer.reset();
//   leftMotorFor(forCoeff);
//   rightMotorFor(forCoeff);
// }

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

void changeTurnDirection(int coeff){
  int dutyCycle = coeff*256*0.01;
  analogWrite(leftMotorEnPin, dutyCycle);
  analogWrite(rightMotorEnPin, dutyCycle);
  if (digitalRead(leftMotorDirPin1)==LOW && digitalRead(leftMotorDirPin2)==HIGH && digitalRead(rightMotorDirPin1)==HIGH && digitalRead(rightMotorDirPin2)==LOW){
    digitalWrite(leftMotorDirPin1,HIGH);
    digitalWrite(leftMotorDirPin2,LOW);
    digitalWrite(rightMotorDirPin1,LOW);
    digitalWrite(rightMotorDirPin2,HIGH);
  }
  if (digitalRead(leftMotorDirPin1)==HIGH && digitalRead(leftMotorDirPin2)==LOW && digitalRead(rightMotorDirPin1)==LOW && digitalRead(rightMotorDirPin2)==HIGH){
    digitalWrite(leftMotorDirPin1,LOW);
    digitalWrite(leftMotorDirPin2,HIGH);
    digitalWrite(rightMotorDirPin1,HIGH);
    digitalWrite(rightMotorDirPin2,LOW);
  }
}

void rightMotorStop(){
  digitalWrite(rightMotorDirPin1, LOW);
  digitalWrite(rightMotorDirPin2, LOW);
};

void detectBeacon(){
  photoTransistorVoltage = analogRead(photoTransistorPin);
  if(photoTransistorVoltage < peakHeight){
    peakHeight = photoTransistorVoltage;
    Serial.println(peakHeight);
  }
}

void detectBeaconThreshold(){
  currVoltage = analogRead(photoTransistorPin);
  if(prevVoltage>currVoltage) thresholdCount++;
  else {thresholdCount=0;}

  if(currVoltage < beaconValueThre){
    beaconValueThre = currVoltage;
  }
  prevVoltage=currVoltage;
  if (thresholdCount>MaxCountThreshold){
    changeTurnDirection(findCoeff);
    thresholdCount=0;
  }
}

void peakHeightComparison(){
  if (peakHeight < beaconValueThre+30){
    beaconCount=2;
  }
};

void startAttackingLeft(){
  state = ATTACKING_LEFT_WALL;
  innerState = MOVING_FORWARD_TO_ATTACK;
  leftMotorFor(forCoeff);
  rightMotorFor(forCoeff);
}
void startAttackingRight(){
  state = ATTACKING_RIGHT_WALL;
  innerState = MOVING_BACKWARD_TO_ATTACK;
  leftMotorBack(backCoeff);
  rightMotorBack(backCoeff);
}
void checkFrontRightLine() {
  if (rightFrontLineDetected()) {
    innerState = TURNING_CLOCKWISE_TO_ATTACK;
    leftMotorFor(turnCoeff);
    rightMotorBack(turnCoeff);
  }
}

/*
If the wall has not yet been detected, then this function checks if the wall is close.
If the wall is close, then the wallTimer begins for 1.5 seconds and the wallDetected
variable is set to "true". If the wall has been detected, then the function does nothing.
*/
void checkWall() {
  if (leftWallDetected == false) {
    //check if wall is close
    if (wallIsClose()) {
      pinMode(ledPin,LOW);
      wallTimer.reset();
      leftWallDetected = true;
      leftMotorFor(forCoeff);
      rightMotorFor(forCoeff);      
    }
  } 
}

bool wallIsClose(){
  bool wallTouched= false;
  if (eventFrontWallTouched()){
    wallTouched=true;
  }
  return wallTouched;
}

bool rightWallIsClose(){
  bool wallTouched= false;
  if (eventBackWallTouched()){
    wallTouched=true;
  }
  return wallTouched;
}

void checkFrontLeftLine(void) {
  if (leftFrontLineDetected()) {
    innerState = MOVING_FORWARD_TO_ATTACK;
    leftMotorFor(forCoeff);
    rightMotorFor(forCoeff);
  }
}

void timerCount(){
  timer ++;
  if(timer >= trigPeriod){
    timer = 0;
    digitalWrite(leftFrontUltraTrigPin, HIGH);
    digitalWrite(rightFrontUltraTrigPin, HIGH);
    digitalWrite(leftBackUltraTrigPin, HIGH);
    digitalWrite(rightBackUltraTrigPin, HIGH);
  }
  if(timer == 2){
    digitalWrite(leftFrontUltraTrigPin, LOW);
    digitalWrite(rightFrontUltraTrigPin, LOW);
    digitalWrite(leftBackUltraTrigPin, LOW);
    digitalWrite(rightBackUltraTrigPin, LOW);
  }
}

void leftFrontEchoRise(){
  leftFrontEchoRiseTime = timer;
}

void leftFrontEchoFall(){
  leftFrontEchoFallTime = timer;
  leftFrontDis = (leftFrontEchoFallTime - leftFrontEchoRiseTime)*0.85 - 80; //mm
}

void rightFrontEchoRise(){
  rightFrontEchoRiseTime = timer;
}

void rightFrontEchoFall(){
  rightFrontEchoFallTime = timer;
  rightFrontDis = (rightFrontEchoFallTime - rightFrontEchoRiseTime)*0.85 - 80; //mm
}

void leftBackEchoRise(){
  leftBackEchoRiseTime = timer;
}

void leftBackEchoFall(){
  leftBackEchoFallTime = timer;
  leftBackDis = (leftBackEchoFallTime - leftBackEchoRiseTime)*0.85 - 80; //mm
}

void rightBackEchoRise(){
  rightBackEchoRiseTime = timer;
}

void rightBackEchoFall(){
  rightBackEchoFallTime = timer;
  rightBackDis = (rightBackEchoFallTime - rightBackEchoRiseTime)*0.85 - 80; //mm
}

bool eventFrontWallTouched(){
  static int frontWallThres = wallThresLow;
  static int prevFrontDis = 8888888;
  bool event = false;
  if(leftFrontDis < frontWallThres && prevFrontDis >= frontWallThres){
    event = true;
    frontWallThres = wallThresHigh;
  }
  if(leftFrontDis >= frontWallThres && prevFrontDis < frontWallThres){
    frontWallThres = wallThresLow;
  }
  prevFrontDis = leftFrontDis;
  return event;
}

bool eventBackWallTouched(){
  static int backWallThres = wallThresLow;
  static int prevBackDis = 8888888;
  bool event = false;
  if(leftBackDis < backWallThres && prevBackDis >= backWallThres){
    event = true;
    backWallThres = wallThresHigh;
  }
  if(leftBackDis >= backWallThres && prevBackDis < backWallThres){
    backWallThres = wallThresLow;
  }
  prevBackDis = leftBackDis;
  return event;
}


/*
If the back right line sensor detects a line, then the innerState should be changed
to TURNING_COUNTER_CLOCKWISE_TO_ATTACK and the right motor should go forward.
*/
void checkBackRightLine(void) {
  if (rightBackLineDetected()) {
    innerState = TURNING_COUNTER_CLOCKWISE_TO_ATTACK;
    rightMotorFor(turnCoeff);
    leftMotorBack(turnCoeff);
  }
}

/*
If the back left line sensor detects a line, then the innerState should be changed
to MOVING_BACKWARD_TO_ATTACK and the right motor should reverse.
*/
void checkBackLeftLine(void) {
  if (leftBackLineDetected()) {
    innerState = MOVING_BACKWARD_TO_ATTACK;
    rightMotorBack(backCoeff);
    leftMotorBack(backCoeff);
  }
}

void checkTotalTime(void) {
  if (totalTime.check() == true) {
    rest();
    state = STATE_REST;
  }
}
void checkRightWall(void) {
  if (rightWallIsClose() && rightWallDetectionCounter == 0) {
    rightWallDetected = true;
    rightWallDetectionCounter ++;
    rightWallTimer.reset();
  }
  checkRightWallTimer();
}

void checkRightWallTimer(void) {
  if (rightWallTimer.check() == true && rightWallDetectionCounter > 0) {
    rest();
    state = STATE_REST;
  }
}
