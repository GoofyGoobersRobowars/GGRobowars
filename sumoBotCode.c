#include <Arduino.h>
#include <math.h>

// MICROSTART START PIN
#define START_PIN 38

// SHARP IR SENSOR PINS
#define IR_LEFT_PIN    2
#define IR_FORWARD_PIN 20
#define IR_RIGHT_PIN   19

// MOTOR PINS
// Back Left
#define FBL 18
#define BBL 17
#define PWM_BL 5

// Back Right
#define FBR 3
#define BBR 8
#define PWM_BR 9

// Front Right
#define FFR 13
#define BFR 14
#define PWM_FR 10

// Front Left
#define FFL 11
#define BFL 12
#define PWM_FL 16

// STBY Pins
#define STBY1 47
#define STBY2 21

void motorSetup() {
  pinMode(FBL, OUTPUT); pinMode(BBL, OUTPUT); pinMode(PWM_BL, OUTPUT);
  pinMode(FBR, OUTPUT); pinMode(BBR, OUTPUT); pinMode(PWM_BR, OUTPUT);
  pinMode(FFR, OUTPUT); pinMode(BFR, OUTPUT); pinMode(PWM_FR, OUTPUT);
  pinMode(FFL, OUTPUT); pinMode(BFL, OUTPUT); pinMode(PWM_FL, OUTPUT);
  pinMode(STBY1, OUTPUT); digitalWrite(STBY1, HIGH);
  pinMode(STBY2, OUTPUT); digitalWrite(STBY2, HIGH);

  ledcAttach(PWM_BL, 1000, 8);
  ledcAttach(PWM_BR, 1000, 8);
  ledcAttach(PWM_FR, 1000, 8);
  ledcAttach(PWM_FL, 1000, 8);
}

void setSpeed(int speed) {
  ledcWrite(PWM_BL, speed);
  ledcWrite(PWM_BR, speed);
  ledcWrite(PWM_FR, speed);
  ledcWrite(PWM_FL, speed);
}

void stopMotors() {
  digitalWrite(FBL, LOW); digitalWrite(BBL, LOW);
  digitalWrite(FBR, LOW); digitalWrite(BBR, LOW);
  digitalWrite(FFR, LOW); digitalWrite(BFR, LOW);
  digitalWrite(FFL, LOW); digitalWrite(BFL, LOW);
  setSpeed(0);
}

void spinLeft() {
  setSpeed(255);
  digitalWrite(FBL, LOW); digitalWrite(BBL, HIGH);
  digitalWrite(FFL, HIGH); digitalWrite(BFL, LOW);
  digitalWrite(FBR, HIGH); digitalWrite(BBR, LOW);
  digitalWrite(FFR, LOW); digitalWrite(BFR, HIGH);
  delay(10);
  stopMotors();
}

void spinRight() {
  setSpeed(255);
  digitalWrite(FBL, HIGH); digitalWrite(BBL, LOW);
  digitalWrite(FFL, LOW); digitalWrite(BFL, HIGH);
  digitalWrite(FBR, LOW); digitalWrite(BBR, HIGH);
  digitalWrite(FFR, HIGH); digitalWrite(BFR, LOW);
  delay(10);
  stopMotors();
}

void moveForward() {
  setSpeed(255);
  digitalWrite(FBL, HIGH); digitalWrite(BBL, LOW);
  digitalWrite(FFL, HIGH); digitalWrite(BFL, LOW);
  digitalWrite(FBR, HIGH); digitalWrite(BBR, LOW);
  digitalWrite(FFR, HIGH); digitalWrite(BFR, LOW);
}

float readDistance(int pin) {
  int adc = analogRead(pin);
  float voltage = adc * (3.3 / 4095.0);
  float distance = 27.86 / pow(voltage, 1.15);
  if (distance > 100) distance = 100;
  return distance;
}

void setup() {
  Serial.begin(115200);
  pinMode(START_PIN, INPUT);
  motorSetup();
  delay(10000);
  /*while (digitalRead(START_PIN) == LOW) {
    stopMotors();
    delay(10);
  }*/
}

void loop() {
  
  /*if (digitalRead(START_PIN) == LOW) {
    stopMotors();
    return;
  }*/

  float distL = readDistance(IR_LEFT_PIN);
  float distF = readDistance(IR_FORWARD_PIN);
  float distR = readDistance(IR_RIGHT_PIN);
  float minDist = min(distL, min(distF, distR));

  if (minDist > 70) {
    stopMotors();
    return;
  }

  if (abs(distF - minDist) < 2.0) {
    moveForward();
    delay(10);
  } else if (abs(distL - minDist) < 2.0) {
    spinLeft();
  } else {
    spinRight();
  }
  delay(10);
}
