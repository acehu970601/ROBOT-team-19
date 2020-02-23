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

bool eventWallTouched(){
  static int wallThres = wallThresLow;
  static int prevDis = 8888888;
  bool event = false;
  if(dis < wallThres && prevDis >= wallThres){
    event = true;
    wallThres = wallThresHigh;
  }
  if(dis >= wallThres && prevDis < wallThres){
    wallThres = wallThresLow;
  }
  prevDis = dis;
  return event;
}