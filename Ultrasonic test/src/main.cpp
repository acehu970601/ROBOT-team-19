#include <Arduino.h>

const int trigPin = 19;
const int echoPin = 20;
const int trigPeriod = 20000;
const int wallThresHigh = 45;
const int wallThresLow = 40;
IntervalTimer unitTimer;

int timer = 0;
int echoRiseTime = 0;
int echoFallTime = 0;
int dis = 8888888;
bool wallStatus = false;

void timerCount(void);
void echoRise(void);
void echoFall(void);
void checkWall(void);
bool eventWallTouched(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Hello, world!");
  
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);

  unitTimer.begin(timerCount, 5);
  attachInterrupt(digitalPinToInterrupt(echoPin), echoRise, RISING);
  attachInterrupt(digitalPinToInterrupt(echoPin), echoFall, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  checkWall();
  if(eventWallTouched()){
    Serial.println("Start pushing wall!");
  }
}

void timerCount(){
  timer ++;
  if(timer >= trigPeriod){
    timer = 0;
    digitalWrite(trigPin, HIGH);
  }
  if(timer == 2){
    digitalWrite(trigPin, LOW);
  }
}

void echoRise(){
  echoRiseTime = timer;
}

void echoFall(){
  echoFallTime = timer;
  dis = (echoFallTime - echoRiseTime)*0.85 - 80; //mm
}

void checkWall(){
  static int wallThres = wallThresLow;
  if(dis < wallThres){
    wallStatus = true;
    wallThres = wallThresHigh;
  }
  if(dis > wallThres){
    wallStatus = false;
    wallThres = wallThresLow;
  }
}

bool eventWallTouched(){
  static bool prevWallStatus = false;
  bool event = false;
  if(prevWallStatus == false && wallStatus == true){
    event = true;
  }
  prevWallStatus = wallStatus;
  return event;
}