#include <Arduino.h>
#include <Metro.h>

const int forTime = 2000;
const int restTime = 1000;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

const int ledPin = LED_BUILTIN;
const int leftMotorDirPin1 = 5;
const int leftMotorDirPin2 = 6;
const int leftMotorEnPin = 3;
const int rightMotorDirPin1 = 7;
const int rightMotorDirPin2 = 8;
const int rightMotorEnPin = 4;

typedef enum {
  STATE_FORWARD, STATE_REST
} States_t;

States_t state;
int forCoeff = 50;
int refDutyCycle = 230;

static Metro restTimer = Metro(restTime);
static Metro forTimer = Metro(forTime);

bool restTimerExpired(void);
bool forTimerExpired(void);
void moveForward(void);
void rest(void);
void leftMotorFor(int);
void rightMotorFor(int);
void leftMotorStop(void);
void rightMotorStop(void);

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);
  pinMode(rightMotorDirPin2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (state) {
    case STATE_REST:
      if(restTimerExpired()) moveForward();
      break;
    case STATE_FORWARD:
      if(forTimerExpired()) rest();
      break;
    default:
      break;    // Should never get into an unhandled state
  }
}

 bool restTimerExpired(){
   return restTimer.check();
 }

 bool forTimerExpired(){
   return forTimer.check();
}

void moveForward(){
  state = STATE_FORWARD;
  forTimer.reset();
  leftMotorFor(forCoeff);
  rightMotorFor(forCoeff);
}

void rest(){
  state = STATE_REST;
  restTimer.reset();
  leftMotorStop();
  rightMotorStop();
}

void leftMotorFor(int coeff){
  int dutyCycle = coeff*refDutyCycle*0.01;
  analogWrite(leftMotorEnPin, dutyCycle);
  digitalWrite(leftMotorDirPin1, HIGH);
  digitalWrite(leftMotorDirPin2, LOW);
};

void rightMotorFor(int coeff){
  int dutyCycle = coeff*refDutyCycle*0.01;
  analogWrite(rightMotorEnPin, dutyCycle);
  digitalWrite(rightMotorDirPin1, HIGH);
  digitalWrite(rightMotorDirPin2, LOW);
};

void leftMotorStop(){
  digitalWrite(leftMotorDirPin1, LOW);
  digitalWrite(leftMotorDirPin2, LOW);
};

void rightMotorStop(){
  digitalWrite(rightMotorDirPin1, LOW);
  digitalWrite(rightMotorDirPin2, LOW);
};