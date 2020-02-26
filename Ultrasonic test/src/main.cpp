#include <Arduino.h>

#define leftFrontUltraTrigPin     13
#define leftFrontUltraEchoPin     12
#define rightFrontUltraTrigPin    14
#define rightFrontUltraEchoPin    11
#define leftBackUltraTrigPin      15
#define leftBackUltraEchoPin      10
#define rightBackUltraTrigPin     16
#define rightBackUltraEchoPin     9

const int trigPeriod = 20000;
const int wallThresHigh = 50;
const int wallThresLow = 40;
IntervalTimer unitTimer;

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

void timerCount(void);
void leftFrontEchoRise(void);
void leftFrontEchoFall(void);
void rightFrontEchoRise(void);
void rightFrontEchoFall(void);
void leftBackEchoRise(void);
void leftBackEchoFall(void);
void rightBackEchoRise(void);
void rightBackEchoFall(void);
bool eventFrontWallTouched(void);
bool eventBackWallTouched(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Hello, world!");
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
}

void loop() {
  // put your main code here, to run repeatedly:
  if(eventFrontWallTouched()){
    Serial.println("Start pushing front wall!");
  }
  if(eventBackWallTouched()){
    Serial.println("Start pushing back wall!");
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
  static int wallThres = wallThresLow;
  static int prevDis = 8888888;
  bool event = false;
  if(leftFrontDis < wallThres && prevDis >= wallThres){
    event = true;
    wallThres = wallThresHigh;
  }
  if(leftFrontDis >= wallThres && prevDis < wallThres){
    wallThres = wallThresLow;
  }
  prevDis = leftFrontDis;
  return event;
}

bool eventBackWallTouched(){
  static int wallThres = wallThresLow;
  static int prevDis = 8888888;
  bool event = false;
  if(leftBackDis < wallThres && prevDis >= wallThres){
    event = true;
    wallThres = wallThresHigh;
  }
  if(leftBackDis >= wallThres && prevDis < wallThres){
    wallThres = wallThresLow;
  }
  prevDis = leftBackDis;
  return event;
}